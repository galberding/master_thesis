#include "ga_path_generator.h"

// #ifdef __DEBUG__
// #undef __DEBUG__
// #define __DEBUG__ true
// #endif
///////////////////////////////////////////////////////////////////////////////
//                              HelperFunctions                               //
///////////////////////////////////////////////////////////////////////////////

using namespace ga;

bool ga::compareFitness(const struct genome &genA, const struct genome &genB){
  return genA.fitness < genB.fitness;
}


ga::genome ga::roulettWheelSelection(ga::Genpool &currentPopulation, std::uniform_real_distribution<float> selDistr, std::mt19937 generator){
  // Calculate Probabilities for all individuals
  // std::default_random_engine generator;
  // std::uniform_real_distribution<float> distribution(0.0,1.0);

  float totalFitness = 0;
  for(auto &gen :currentPopulation){
    totalFitness += gen.fitness;
  }

  float rand = selDistr(generator);
  // cout << "Wheel: " << rand << endl;
  float offset = 0.0;
  genome gen;

  // TODO: use Iterater

  for(auto it = currentPopulation.begin(); it != currentPopulation.end(); it++){
    offset += it->fitness / totalFitness;

    if(rand < offset){
      // gen = genome(*it);
      gen.fitness = it->fitness;
      gen.actions = it->actions;
      // debug("THE SELECTED ONE: ", gen.actions.size());
      currentPopulation.erase(it);
      break;
    }

  }

  // for(int i=0; i < currentPopulation.size(); i++){
  //   offset += currentPopulation.at(i).fitness / totalFitness;

  //   if(rand < offset){
  //     gen = currentPopulation.at(i);
  //     // debug("THE SELECTED ONE: ", gen.actions.size());
  //     currentPopulation.erase(currentPopulation.begin() + i);
  //     break;
  //   }
  // }
  return gen;
}

int ga::randRange(int lower, int upper){
  return (rand() % (upper -lower) + lower);
}


void ga::validateGen(genome &gen){
 for(auto it = gen.actions.begin(); it != gen.actions.end(); it++){
   // What is needed to validate the gens?
   // Gens are not validated after initialization!
   // Gens need to be falidated after crossover to fix the crossing
   // Consequently each gen that is flagged as modified needs to be adapted or its changes need to be
   // propagated to the next Gen
   if(!(*it)->modified) continue;

   // Assumptions:
   // What about newly created actions? -> generate waypoints by adding!
   assertm((*it)->wps.size() >= 1, "Not enough gens to validate!");
   // Ensures that we can always access the previous action without segfault
   assertm(!((*it)->modified && ((*it)->type == PAT::Start)), "Not allowed!, Start points cannot be modified!");
   assertm(!((*it)->modified && ((*it)->type == PAT::End)), "Not allowed!, End points cannot be modified!");


    (*it)->applyModifications();
    // Change the configuration of the consecutive action
    // Propagate changes until properly applied
    // We do not want to override any changes here!
    auto it_current = it;
    auto it_next = next(it, 1);
    while(!(*it_next)->mendConfig(*it_current)){
      // warn("Validation: Remove Action ", (*it_next)->pa_id, " because no distance!");
      // Cannot mend means that the current changes are applied and we need to mend the consecutive action
      it_current = it_next;
      it_next++;
      // TODO: goto end or just return because all changes had been applied
      if(it_next == gen.actions.end()) return;


    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//                             Mutation Functions                            //
///////////////////////////////////////////////////////////////////////////////

void ga::addAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  // debug("Add Action!");
  // Copy action at Index
  // Add offset to angle and draw distance from distribution
  // insert action at i+1
  // flag current action as modified? -> will trigger mend in evaluateGen

  int idx = randRange(1, gen.actions.size()-1);
  // debug("Index: ", idx, " Size: ", gen.actions.size());

  auto it = next(gen.actions.begin(), idx);
  assertm((*it)->wps.size() >= 1, "Cannot add action when there is no waypoint in the previous one");

  PA_config conf = (*it)->getConfig();
  distanceDist(generator);
  conf[PAP::Distance] = distanceDist(generator);
  conf[PAP::Angle] += angleDist(generator);

  PAT type = (*it)->type;
  AheadAction aa(type,conf);
  aa.generateWPs((*it)->wps.back());
  assertm(aa.wps.size() >= 2, "Newly added actions has not generated waypoints properly!");
  aa.modified = true;
  // Insert after the current position
  it = gen.actions.insert(next(it,1),make_shared<AheadAction>(aa));
}

//TODO:
void ga::removeAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  if (gen.actions.size() < 4){
    return;
  }
  assertm(gen.actions.size() >= 4, "Too few actions left to remove further actions");

  int idx = randRange(2, gen.actions.size()-2);
  auto it = gen.actions.erase(next(gen.actions.begin(), idx));
  (*prev(it,1))->modified = true;
}


// void modufie(genome &gen, PA_config config){

// }

//TODO:
void ga::addAngleOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  int idx = randRange(1, gen.actions.size()-1);
  float offset = angleDist(generator);
  auto it = next(gen.actions.begin(), idx);
  // We do not want to modify a already modified actions!
  if((*it)->modified) return;
  assertm((*it)->wps.size() >= 2, "Actions has no waypoints -- muation while initialization not allowed!");
  // debug(("Type: ", (int) next(gen.actions.begin(), idx)->type));
  // debug("Angle offset: ", offset);
  (*it)->mod_config[PAP::Angle] += offset;
  (*it)->mod_config[PAP::AngleOffset] += offset;
  (*it)->modified = true;

}

//TODO:
void ga::addDistanceOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  int idx = randRange(1, gen.actions.size()-1);
  float offset = distanceDist(generator);
  auto it = next(gen.actions.begin(), idx);
  // We do not want to modify a already modified actions!
  if((*it)->modified) return;
  assertm((*it)->wps.size() >= 2, "Actions has no waypoints -- muation while initialization not allowed!");
  // debug(("Type: ", (int) next(gen.actions.begin(), idx)->type));
  // debug("Angle offset: ", offset);
  (*it)->mod_config[PAP::Distance] += offset;
  (*it)->mod_config[PAP::DistanceOffset] += offset;
  (*it)->modified = true;
}

//TODO: or remove
void ga::swapRandomAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  int idx1 = randRange(1, gen.actions.size()-1);
  int idx2 = randRange(1, gen.actions.size()-1);

  auto item1 = *next(gen.actions.begin(), idx1);

  *next(gen.actions.begin(), idx1) = *next(gen.actions.begin(), idx2);
  *next(gen.actions.begin(), idx2) = item1;

}


///////////////////////////////////////////////////////////////////////////////
//                                     GA                                    //
///////////////////////////////////////////////////////////////////////////////

void ga::GA::populatePool(Genpool &currentPopuation, Position start, WPs endpoints, int individuals, int initialActions){

  for(int j=0; j<individuals; j++){
    PAs actions;

    actions.push_back(make_shared<StartAction>(StartAction(start)));
    for(int i=0; i<initialActions; i++){
      PA_config config{{PAP::Angle,angleDistr(generator)}, {PAP::Distance, distanceDistr(generator)}};
      actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
    }
    actions.push_back(make_shared<EndAction>(EndAction(endpoints)));
    currentPopuation.push_back(genome(actions));
  }
}


void ga::GA::selection(ga::Genpool& currentPopuation, ga::Genpool& selectionPool, int individuals, int keepBest) {
  // debug("Selected!!!");
  Genpool keep;
  if(keepBest > 0){
    assertm(keepBest < currentPopuation.size(), "Cannot keep more individuals than in the pool");
    keep.insert(keep.begin(), prev(currentPopuation.end(), keepBest), currentPopuation.end());
  }

  sort(currentPopuation.begin(), currentPopuation.end());

  // Perform turnament selection:
  for(int i=0; i<individuals; i++){
    if(currentPopuation.size() > 0)
      selectionPool.push_back(roulettWheelSelection(currentPopuation, selectionDist, generator));
    else{
      warn("Not enough individuals left in the pool for mation ...");
      break;
    }
  }

  currentPopuation.clear();

  // assertm()

  if(keepBest > 0){
    currentPopuation.insert(currentPopuation.begin(), keep.begin(), keep.end());
  }

}


void copyActions(PAs::iterator begin, PAs::iterator end, PAs &child){
  for(begin; begin != end; begin++){
    // debug("Type: ", (int) (*begin)->type);
    switch((*begin)->type){
    case PAT::Start:{
      StartAction sa((*begin)->wps.front());
      child.push_back(make_shared<StartAction>(sa));
      break;
    }
    case PAT::Ahead: case PAT::CAhead:{
      AheadAction aa((*begin)->type, (*begin)->mod_config);
      aa.generateWPs((*begin)->wps.front());
      child.push_back(make_shared<AheadAction>(aa));
      break;
    }
    case PAT::End:
      child.push_back(make_shared<EndAction>(EndAction((*begin)->wps)));
    }
    // child1.push_back(make_shared<typename _Tp>(_Args &&__args...))
  }
}

void ga::GA::mating(genome &par1, genome &par2, Genpool& newPopulation){
  // debug("Start Mating");
  PAs parent1, parent2, child1, child2;
  assertm(par1.actions.size() > 3, "Parent1 has too few actions");
  assertm(par1.actions.size() > 3, "Parent2 has too few actions");

  if(par1.actions.size() > par2.actions.size()){
    parent1 = par2.actions;
    parent2 = par1.actions;
  }else{
    parent1 = par1.actions;
    parent2 = par2.actions;
  }

  int idx1 = randRange(2, parent1.size() - 2);
  // int idx2 = randRange(1, parent2.size() - 2);
  int idx2 = idx1;
  // int idx = 5;
  // debug("Index: ", idx);
  // child1.insert(child1.begin(), parent1.begin(), parent1.begin() + idx);
  // create deep copy of parents:
  assertm(parent1.size() > 4, "Parent1 has too few actions");
  assertm(parent1.size() > 4, "Parent2 has too few actions");
  copyActions(parent1.begin(), next(parent1.begin(), idx1), child1);
  copyActions(parent2.begin(), next(parent2.begin(), idx2), child2);
  // child2.insert(child2.begin(), parent2.begin(), std::next(parent2.begin(), idx));

  copyActions(std::next(parent2.begin(), idx2), parent2.end(), child1);
  copyActions(std::next(parent1.begin(), idx1), parent1.end(), child2);
  // child2.insert(child2.end(), std::next(parent1.begin(), idx), parent1.end());
  // child1.insert(child1.begin(), parent1.begin(), std::next(parent1.begin(), idx));
  // child2.insert(child2.begin(), parent2.begin(), std::next(parent2.begin(), idx));

  // child1.insert(child1.end(), std::next(parent2.begin(), idx), parent2.end());


  // Mark crossing PAs as modufied to connect the two pieces
  (*next(child1.begin(), idx1-1))->modified = true;
  (*next(child2.begin(), idx2-1))->modified = true;
  genome child_gen1(child1);
  genome child_gen2(child2);



  // Calidate the gens
  validateGen(child_gen1);
  validateGen(child_gen2);

  // Insert to new Population
  newPopulation.push_back(child_gen1);
  newPopulation.push_back(child_gen2);

}

void ga::GA::crossover(ga::Genpool& currentSelection, ga::Genpool& newPopulation) {
  // Single point Crossover
  assertm(currentSelection.size() >= 2, "Not enough individuals for crossover in pool!");

  // TODO: Switch to iterator
  // auto par2 = next(currentSelection.begin(), 1);
  // for(auto par1 = currentSelection.begin(); par1 != prev(currentSelection.end(), 2); par1++){
  //   while(par2 != )
  // }


  for(int i=0; i<currentSelection.size(); i++){
    genome par1 = currentSelection.back();
    currentSelection.pop_back();
    for (auto par2 : currentSelection){
      // debug("Parent size1: ", par1.actions.size(), " Parent size2: ", par2.actions.size());
      assertm(par1.actions.size() > 3, "Parent1 has not enough actions for mating!");
      assertm(par2.actions.size() > 3, "Parent2 has not enough actions for mating!");
      mating(par1, par2, newPopulation);
    }
  }
  currentSelection.clear();
  // debug("Cross done!");
}


void ga::GA::mutation(Genpool& currentPopulation, Mutation_conf& muat_config) {
  for (auto &gen : currentPopulation){
    for(auto &[k, v] : muat_config){
      int proba = randRange(0,100);
      // debug("Mutation Proba: ", proba, " Config Proba: ", v.second);
      if(v.second >= proba){
	// debug("Mutate: ", k);
	// execute mutation strategy
	// info("Execute: ", k);
	// info("Action size befor: ", gen.actions.size());
	// info("Fitness: ", gen.fitness);
	v.first(gen, angleDistr, distanceDistr, generator);
	validateGen(gen);
	// info("Action size after: ", gen.actions.size());
	// debug("Done!");
      }
    }

  }
}


void ga::GA::evalFitness(Genpool &currentPopulation, path::Robot &rob){
  // First evaluate action on map
  // Sum multiple covered regions
  // Sum traveled Distance

  for(Genpool::iterator it = currentPopulation.begin(); it != currentPopulation.end();){
    if(rob.evaluateActions(it->actions)){

      float Cdistance = 0; // Cleand distance
      float distance = 0; // uncleand distance
      float occ = 0;
      int crossed = 0;
      // iterate over all actions
      for(auto action : it->actions){
	auto conf = action->c_config;
	// Check if action did a move
	if(conf.count(Counter::StepCount) > 0){
	  if(action->type == PAT::CAhead){
	    // Add cross count (default is 1)
	    crossed += conf[Counter::CrossCount];
	    // crossed += conf[PAP::CrossCount] - 1;
	    Cdistance += conf[Counter::StepCount];
 	  }else{
	    distance += conf[Counter::StepCount];
	  }
	}else{
	  // assertm(false, "Action did not make a move and is still listed");
	}
      }
      // debug("Crossed: ", crossed, " dist: ", Cdistance);
      assertm(crossed <= Cdistance, "CrossCount is bigger than traveled distance!");
      // crossed = crossed / it->actions.size();
      it->fitness = calFitness(
			       Cdistance,
			       distance,
			       crossed,
			       rob.getConfig()[RP::Clean_speed_cm_s] / 100,
			       rob.getConfig()[RP::Drive_speed_cm_s] / 100,
			       rob.getFreeArea());

      it++;
    }else{
      warn("Erase Gen!");
      assertm(false, "Attempt to erase a gen!!");
      it = currentPopulation.erase(it);
    }
  }
}


float ga::GA::calFitness(float cdist,
			  float dist,
			  int crossed,
			  float cSpeed_m_s,
			  float speed_m_s,
			  int freeSpace){
  // Use the different speeds for time calculation
  // debug("Crossed ", crossed);
  // debug("cdist ", cdist);
  // debug("speed", cSpeed_m_s);

  assertm(cdist >= crossed, "");

  float actual_time = cdist + dist;
  float optimal_time = (cdist - crossed);
  float final_time = optimal_time / actual_time;


  // Calculate the final occ based on the
  float current_occ = cdist - crossed;
  // if (current_occ < 0){
  //   current_occ = crossed;
  // }
  float optimal_occ = cdist + crossed;

  float final_occ = current_occ / optimal_occ ;

  // Area coverage
  float ac = current_occ / freeSpace;
  assertm(freeSpace >= current_occ , "No space to cover");

  // debug("Actual time: ", actual_time);
  // debug("Optimal time: ", optimal_time);
  // debug("Final time: ", final_time);
  // debug("final_occ: ", final_occ);
  // debug("Space relation: ", current_occ / freeSpace);

  float weight = eConf.fitnessWeight;
  float fitness = ((1-weight)*(final_time + final_occ) + weight*ac) / 3;
  if(!eConf.fitnessName.empty()){
    Logger(
	   argsToCsv(eConf.currentIter,cdist, dist, crossed, freeSpace, actual_time, optimal_time, final_time, current_occ, optimal_occ, final_occ, ac, fitness),
	   eConf.logDir,
	   eConf.fitnessName);
  }

  return fitness;

}
#define logg(msg, config) logging::Logger(msg, config.logDir, config.logName)

void ga::GA::optimizePath() {

  // Use configuration and obstacle map to create robot and optimize the path
  // log all runtime information here provided by the conf
  // logg("------Start-Training------", eConf);
  if(!eConf.logName.empty()){
    Logger("Iteration,Best_fit,Worst_fit,Max_len,Min_len", eConf.logDir, eConf.logName);
  }

  if(!eConf.fitnessName.empty()){
    Logger(
	   argsToCsv("CurrentIter","cdist", "dist", "crossed", "freeSpace", "actual_time", "optimal_time", "final_time", "current_occ", "optimal_occ", "final_occ", "ac", "fitness"),
	   eConf.logDir,
	   eConf.fitnessName);
  }
  // Robot

  Robot rob(eConf.rob_conf, eConf.gmap, "map");

  Genpool pool ,selected;

  // Populate pool
  populatePool(pool, eConf.start, eConf.ends, eConf.initIndividuals, eConf.initActions);
  // Get initial fitness calculation
  evalFitness(pool, rob);

  // First selection
  selection(pool, selected, eConf.selectIndividuals, eConf.selectKeepBest);

  // Main loop
  int lowest = 1000;
  int highest = 0;
  for (int i = 0; i < eConf.maxIterations; ++i) {
    eConf.currentIter = i;

    crossover(selected, pool);
    // TODO: Either create config here or provide it through eConf
    // [x] provide it through eConf
    mutation(pool, eConf.muta);
    evalFitness(pool, rob);
    selection(pool, selected, eConf.selectIndividuals, eConf.selectKeepBest);

    for (auto &gen : pool){
	// cout << "Gens: " << gen.fitness << " ";
	int a_size = gen.actions.size();
	if(a_size < lowest){
	  lowest = a_size;
	}
	if(a_size > highest){
	  highest = a_size;
	}
      }
    // How to log the overall fitness values??
    // Log all to individual files and use iteration to merge all components

    if(!eConf.logName.empty()){
      string msg =to_string(eConf.currentIter)+","
	+ to_string(pool.back().fitness) + ","
	+ to_string(pool.front().fitness) + ","
	+ to_string(highest) + ","
	+ to_string(lowest) + "\n";
    Logger(msg, eConf.logDir, eConf.logName);
    }


  }
  // logg("------End-Training------", eConf);






}
