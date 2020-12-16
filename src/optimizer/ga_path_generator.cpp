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
  for(int i=0; i < currentPopulation.size(); i++){
    offset += currentPopulation.at(i).fitness / totalFitness;

    if(rand < offset){
      gen = currentPopulation.at(i);
      // debug("THE SELECTED ONE: ", gen.actions.size());
      currentPopulation.erase(currentPopulation.begin() + i);
      break;
    }
  }
  return gen;
}

int ga::randRange(int lower, int upper){
  return (rand() % (upper -lower) + lower);
}


void ga::validateGen(genome &gen){
  for(auto it = gen.actions.begin(); it != gen.actions.end(); it++){
    if(!(*it)->modified) continue;
    // Action is modified so try to apply the current changes
    if(!(*it)->applyMods()){
      // warn("No startpoint to apply changes to!, type; ", int(it->type));
      //
      // Case if PA is newly added
      // Moduification could not be applied because no waypoints have been generated yet
      // This will automativally apply the new changes
      // debug("No waypoints --> regenerate ...");
      (*it)->generateWPs((*prev(it, 1))->get_wps().back());
    }else{

      // debug("Changes applied!");
    }
    // Change the configuration of the consecutive action
    auto it_next = next(it, 1);
    while(!(*it_next)->mend(**it) && it_next != gen.actions.end()){
      warn("Validation: Remove Action ", (*it_next)->pa_id, " because no distance!");
      it_next = gen.actions.erase(it_next);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//                             Mutation Functions                            //
///////////////////////////////////////////////////////////////////////////////

void ga::addAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  int idx = randRange(1, gen.actions.size()-1);
  // debug("Index: ", idx, " Size: ", gen.actions.size());

  PA_config conf = (*next(gen.actions.begin(), idx))->getConfig();
  distanceDist(generator);
  conf[PAP::Distance] = distanceDist(generator);
  // debug("NewDist: ", conf[PAP::Distance]);
  conf[PAP::Angle] += angleDist(generator);

  PAT type = (*next(gen.actions.begin(), idx))->type;
  // debug("Action length before : ", gen.actions.size());
  AheadAction aa(type, conf);
  debug("Adding Gen: ", aa.pa_id);
  gen.actions.insert(next(gen.actions.begin(), idx),make_shared<AheadAction>(aa));

  // debug("Action length after: ", gen.actions.size());
}

void ga::removeAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  if (gen.actions.size() < 4){
    return;
  }
  int idx = randRange(1, gen.actions.size()-2);
  auto it = gen.actions.erase(next(gen.actions.begin(), idx));
  (*prev(it,1))->modified = true;
}


// void modufie(genome &gen, PA_config config){

// }

void ga::addAngleOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  int idx = randRange(1, gen.actions.size()-1);
  float offset = angleDist(generator);
  auto it = next(gen.actions.begin(), idx);
  // debug(("Type: ", (int) next(gen.actions.begin(), idx)->type));
  // debug("Angle offset: ", offset);
  (*it)->mod_config[PAP::Angle] += offset;
  (*it)->mod_config[PAP::AngleOffset] += offset;
  (*it)->modified = true;

}

void ga::addDistanceOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  int idx = randRange(1, gen.actions.size()-1);
  float offset = distanceDist(generator);
  auto it = next(gen.actions.begin(), idx);
  // debug(("Type: ", (int) next(gen.actions.begin(), idx)->type));
  // debug("Angle offset: ", offset);
  (*it)->mod_config[PAP::Distance] += offset;
  (*it)->mod_config[PAP::DistanceOffset] += offset;
  (*it)->modified = true;
}

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

  // for(auto &gen : currentPopuation){
  //   gen.actions.begin()->generateWPs(Position(0,0));
  //   for (auto it = next(gen.actions.begin(), 1); it != gen.actions.end(); it++){
  //     it->generateWPs(prev(it, 1)->get_wps().back());
  //     // debug("gen");
  //   }
  // }

}


void ga::GA::selection(ga::Genpool& currentPopuation, ga::Genpool& selectionPool, int individuals) {
  // Sort descending according to fitness
  // debug("Start sort");
  sort(currentPopuation.begin(), currentPopuation.end(), compareFitness);
  // debug("End sort");

  // Perform turnament selection:
  for(int i=0; i<individuals; i++){
    selectionPool.push_back(roulettWheelSelection(currentPopuation, selectionDist, generator));
  }
}


void copyActions(PAs::iterator begin, PAs::iterator end, PAs &child){
  for(auto it = begin; it != end; it++){
    switch((*it)->type){
    case PAT::Start:{
      StartAction sa((*it)->wps.front());
      child.push_back(make_shared<StartAction>(sa));
      break;
    }
    case PAT::Ahead: case PAT::CAhead:{
      AheadAction aa((*it)->type, (*it)->mod_config);
      aa.generateWPs((*it)->wps.front());
      child.push_back(make_shared<AheadAction>(aa));
      break;
    }
    case PAT::End:
      child.push_back(make_shared<EndAction>(EndAction((*it)->wps)));
    }
    // child1.push_back(make_shared<typename _Tp>(_Args &&__args...))
  }
}

void ga::GA::mating(genome &par1, genome &par2, Genpool& newPopulation){
  // debug("Start Mating");
  PAs parent1, parent2, child1, child2;
  if(par1.actions.size() > par2.actions.size()){
    parent1 = par2.actions;
    parent2 = par1.actions;
  }else{
    parent1 = par1.actions;
    parent2 = par2.actions;
  }

  int idx1 = randRange(1, parent1.size() - 2);
  // int idx2 = randRange(1, parent2.size() - 2);
  int idx2 = idx1;
  // int idx = 5;
  // debug("Index: ", idx);
  // child1.insert(child1.begin(), parent1.begin(), parent1.begin() + idx);
  // create deep copy of parents:
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
  for(int i=0; i<currentSelection.size(); i++){
    genome par1 = currentSelection.back();
    currentSelection.pop_back();
    for (auto par2 : currentSelection){
      // debug("Parent size1: ", par1.actions.size(), " Parent size2: ", par2.actions.size());
      mating(par1, par2, newPopulation);
    }
  }
  // debug("Cross done!");
}


void ga::GA::mutation(Genpool& currentPopulation, Mutation_conf& muat_config) {
  for (auto gen : currentPopulation){
    for(auto &[k, v] : muat_config){
      int proba = randRange(0,100);
      // debug("Mutation Proba: ", proba, " Config Proba: ", v.second);
      if(v.second <= proba){
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
  float weight = 0.6;
  return ((1-weight)*(final_time + final_occ) + weight*ac) / 3;

}
