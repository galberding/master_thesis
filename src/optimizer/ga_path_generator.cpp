#include "ga_path_generator.h"

// #ifdef __DEBUG__
// #undef __DEBUG__
// #define __DEBUG__ true
// #endif
///////////////////////////////////////////////////////////////////////////////
//                              HelperFunctions                               //
///////////////////////////////////////////////////////////////////////////////

using namespace ga;

int ga::genome::gen_id = 0;

bool ga::compareFitness(const struct genome &genA, const struct genome &genB){
  return genA.fitness < genB.fitness;
}


ga::genome ga::roulettWheelSelection(ga::Genpool &currentPopulation, float totalFitness, std::mt19937 generator, bool erase){
  // Calculate Probabilities for all individuals
  std::uniform_real_distribution<float> selDistr(0.0,1);

  float rand = selDistr(generator);
  // cout << "Wheel: " << rand << endl;
  float offset = 0.0;
  genome gen;

  for(auto it = currentPopulation.begin(); it != currentPopulation.end(); it++){
    offset += it->fitness / totalFitness;
    if(rand < offset){
      gen = *it;
      if(erase)
	it = currentPopulation.erase(it);
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
    // debug("Type: ", int((*it_next)->type));
    while(!(*it_next)->mendConfig(*it_current)){
      assertm(!((*it_current)->modified && ((*it)->type == PAT::Start)), "Not allowed!, Start points cannot be modified!");
      assertm(!((*it_next)->modified && ((*it)->type == PAT::End)), "Not allowed!, End points cannot be modified!");
      // warn("Validation: Remove Action ", (*it_next)->pa_id, " because no distance!");
      // Cannot mend means that the current changes are applied and we need to mend the consecutive action
      it_current = it_next;
      assertm((*it_next)->type != PAT::End, "Try to access type after end!");
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
  // uniform_int_distribution<int> distDist(0,distanceDist.max());
  float offset = angleDist(generator);
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

// void ga::addDistanceOffsetPositive(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
//   int idx = randRange(1, gen.actions.size()-1);
//   uniform_int_distribution<int> distDist(0,distanceDist.max());
//   float offset = distDist(generator);
//   auto it = next(gen.actions.begin(), idx);
//   // We do not want to modify a already modified actions!
//   if((*it)->modified) return;
//   assertm((*it)->wps.size() >= 2, "Actions has no waypoints -- muation while initialization not allowed!");
//   // debug(("Type: ", (int) next(gen.actions.begin(), idx)->type));
//   // debug("Angle offset: ", offset);
//   (*it)->mod_config[PAP::Distance] += offset;
//   (*it)->mod_config[PAP::DistanceOffset] += offset;
//   (*it)->modified = true;
// }


//TODO: or remove
void ga::swapRandomAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator){
  int idx1 = randRange(1, gen.actions.size()-1);
  int idx2 = randRange(1, gen.actions.size()-1);

  auto item1 = *next(gen.actions.begin(), idx1);

  *next(gen.actions.begin(), idx1) = *next(gen.actions.begin(), idx2);
  *next(gen.actions.begin(), idx2) = item1;

}

void ga::addOrthogonalAngleOffset(genome& gen, executionConfig& eConf, std::mt19937& generator) {
  // Add orthogonal angle {-90, 90}

  uniform_real_distribution<float> probaDist(0,1);
  if(eConf.mutaOrtoAngleProba < probaDist(generator)) return;

  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_int_distribution<int> changeDistro(0,1);

  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(generator));
  (*action)->mod_config[PAP::Angle] += changeDistro(generator) ? 90 : -90;
  (*action)->modified = true;
}

void ga::addRandomAngleOffset(genome& gen, executionConfig& eConf, std::mt19937& generator) {
  uniform_real_distribution<float> probaDist(0,1);
  if(eConf.mutaRandAngleProba < probaDist(generator)) return;

  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_int_distribution<int> changeDistro(0,360);

  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(generator));
  (*action)->mod_config[PAP::Angle] += changeDistro(generator);
  (*action)->modified = true;
}

void ga::addPositiveDistanceOffset(genome& gen, executionConfig& eConf, std::mt19937& generator) {

  uniform_real_distribution<float> probaDist(0,1);
  if(eConf.mutaPosDistProba < probaDist(generator)) return;

  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_real_distribution<> changeDistro(0,eConf.mutaPosDistMax);

  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(generator));
  (*action)->mod_config[PAP::Distance] += changeDistro(generator);
  (*action)->modified = true;
}

void ga::addNegativeDistanceOffset(genome& gen, executionConfig& eConf, std::mt19937& generator) {


  uniform_real_distribution<float> probaDist(0,1);
  if(eConf.mutaPosDistProba < probaDist(generator)) return;

  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_real_distribution<> changeDistro(0,eConf.mutaPosDistMax);

  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(generator));
  float offset = changeDistro(generator);
  if((*action)->mod_config[PAP::Distance] > offset){
    (*action)->mod_config[PAP::Distance] -= offset;
    (*action)->modified = true;
  }
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


void ga::GA::selection(ga::Genpool& currentPopulation, ga::Genpool& selectionPool, int individuals, int keepBest) {
  // debug("Selected!!!");
  Genpool keep;

  sort(currentPopulation.begin(), currentPopulation.end());
  secretBest = currentPopulation.back();
  if(keepBest > 0){
    assertm(keepBest < currentPopulation.size(), "Cannot keep more individuals than in the pool");
    keep.insert(keep.begin(), prev(currentPopulation.end(), keepBest), currentPopulation.end());
  }

  // TODO: Did anything break??
  // Perform roulettwheel selection:
  for(int i=0; i<individuals; i++){
    if(currentPopulation.size() > 0){
        float totalFitness = 0;
	for(auto &gen :currentPopulation){
	  totalFitness += gen.fitness;
	}
      genome gen = roulettWheelSelection(currentPopulation, totalFitness, generator);
      if(gen.fitness == 0){
	warn("Selected gen with fitness 0, it will be skipped!");
	continue;
      }
      selectionPool.push_back(gen);
    } else {
      warn("Not enough individuals left in the pool for mating ...");
      break;
    }
  }

  currentPopulation.clear();

  if(keepBest > 0){
    currentPopulation.insert(currentPopulation.begin(), keep.begin(), keep.end());
  }

}

// TODO: Preserve IDs
void copyActions(PAs::iterator begin, PAs::iterator end, PAs &child, bool modify=false){
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
      aa.modified = modify;
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
    for (auto &par2 : currentSelection){
      // debug("Parent size1: ", par1.actions.size(), " Parent size2: ", par2.actions.size());
      assertm(par1.actions.size() > 3, "Parent1 has not enough actions for mating!");
      assertm(par2.actions.size() > 3, "Parent2 has not enough actions for mating!");
      mating(par1, par2, newPopulation);
     //  for(auto &gen : newPopulation){
     // 	assertm(gen.actions.size() > 3, "Gen after mating has not enough actions!");
     // }
    }
  }
  currentSelection.clear();
  for(auto &gen : newPopulation){
	assertm(gen.actions.size() > 3, "Gen after mating has not enough actions!");
     }
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

  eConf.fitnessAvg = 0;
  eConf.fitnessMax = 0;
  eConf.fitnessMin = 1;
  eConf.fitnessAvgTime = 0;
  eConf.fitnessAvgOcc = 0;
  eConf.fitnessAvgCoverage = 0;
  eConf.actionLenMax = 0;
  eConf.actionLenMin = 100000;
  eConf.actionLenAvg = 0;
  for(Genpool::iterator it = currentPopulation.begin(); it != currentPopulation.end();){
    assertm(it->actions.size() > 0, "Not enough actions");
    if(rob.evaluateActions(it->actions)){
      assertm(it->actions.size() > 0, "Not enough actions");
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
      float fitness = calFitness(
			       Cdistance,
			       distance,
			       crossed,
			       rob.getConfig()[RP::Clean_speed_cm_s] / 100,
			       rob.getConfig()[RP::Drive_speed_cm_s] / 100,
			       rob.getFreeArea());
      it->fitness = fitness;

      // Logging of fittnessvalues
      if(fitness > eConf.fitnessMax) eConf.fitnessMax = fitness;
      if(fitness < eConf.fitnessMin) eConf.fitnessMin = fitness;
      auto size = it->actions.size();
      if(size > eConf.actionLenMax) eConf.actionLenMax = size;
      if(size < eConf.actionLenMin) eConf.actionLenMin = size;
      eConf.fitnessAvg += fitness;
      eConf.actionLenAvg += it->actions.size();

      it++;
    }else{
      warn("Erase Gen!");
      assertm(false, "Attempt to erase a gen!!");
      it = currentPopulation.erase(it);
    }
  }
  eConf.fitnessAvg /= currentPopulation.size();
  eConf.fitnessAvgTime /= currentPopulation.size();
  eConf.fitnessAvgOcc /= currentPopulation.size();
  eConf.fitnessAvgCoverage /= currentPopulation.size();
  eConf.actionLenAvg /= currentPopulation.size();
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

  assert(cdist >= crossed);

  float actual_time = cdist + dist;
  float optimal_time = (cdist - crossed);
  float final_time = optimal_time / actual_time;


  // Calculate the final occ based on the
  // TODO: rename fitness parameter, sth like crossFactor or overlapping
  float current_occ = cdist - crossed;
  // if (current_occ < 0){
  //   current_occ = crossed;
  // }
  float optimal_occ = cdist + crossed;

  float final_occ = current_occ / optimal_occ ;

  // Area coverage
  float final_coverage = current_occ / freeSpace;
  assertm(freeSpace >= current_occ , "No space to cover");
  // Ensure that the gen is not selected for crossover by setting the fitness to 0
  if(isnan(final_time) || isnan(final_occ) || isnan(final_coverage)) return 0;

  eConf.fitnessAvgTime += final_time;
  eConf.fitnessAvgOcc += final_occ;
  eConf.fitnessAvgCoverage += final_coverage;


  float weight = eConf.fitnessWeight;
  // float fitness = ((1-weight)*(final_time + final_occ) + weight*final_coverage) / 3;
  float fitness = eConf.fitnessWeights[0]*final_time
    + eConf.fitnessWeights[1]*final_occ
    + eConf.fitnessWeights[2]*final_coverage;

  // Fitness calculation -> coverage directly influences Time
  // if coverage improves time gets more important and reverse

  //
  // float fitness = (final_coverage)*(eConf.fitnessWeights[0]*final_time)
  //   + eConf.fitnessWeights[1]*final_occ
  //   + (1-final_coverage)*(eConf.fitnessWeights[2]*final_coverage);

  // if(!eConf.fitnessName.empty()){
  //   (*eConf.fitnessStr) << argsToCsv(eConf.currentIter,
  // 				  cdist,
  // 				  dist,
  // 				  crossed,
  // 				  freeSpace,
  // 				  actual_time,
  // 				  optimal_time,
  // 				  final_time,
  // 				  current_occ,
  // 				  optimal_occ,
  // 				  final_occ,
  // 				  final_coverage,
  // 				  fitness);
  // }

  return fitness;

}
#define logg(msg, config) logging::Logger(msg, config.logDir, config.logName)

void ga::GA::optimizePath(bool display) {

  // Use configuration and obstacle map to create robot and optimize the path
  // log all runtime information here provided by the conf
  // logg("------Start-Training------", eConf);
  Logger(eConf.config_to_string(), eConf.logDir, eConf.logName);

  if(!eConf.logName.empty()){
    Logger("Iteration,FitAvg,FitMax,FitMin,AvgTime,AvgOcc,AvgCoverage,ActionLenAvg,ActionLenMax,ActionLenMin", eConf.logDir, eConf.logName);
  }

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
    cout << "Path Length: ";
    for (auto it = pool.begin(); it != pool.end(); ++it) {
      cout << (*it).getPathLen();
      for (auto it2 = next(it, 1); it2 != pool.end(); ++it2) {
	if(it->getPathLen() == it2->getPathLen()){
	  warn("Equal genome length detected ...");
	  debug("Gen1:");
	  for (auto ac = it->actions.begin(); ac != it->actions.end(); ++ac) {
	    cout << (*ac)->mod_config[PAP::Distance] << "|";
	  }
	  cout << endl;
	  debug("gen2");
	  for (auto ac = it2->actions.begin(); ac != it2->actions.end(); ++ac) {
	    cout << (*ac)->mod_config[PAP::Distance] << "|";
	  }
	  cout << endl;

	  // assert(it->getPathLen() != it2->getPathLen());
	}
      }
    }

    // debug("Poolsize: ", pool.size());
    // TODO: Either create config here or provide it through eConf
    // [x] provide it through eConf
    mutation(pool, eConf.muta);
    // debug("Poolsize Mut: ", pool.size());
    evalFitness(pool, rob);
    if(!eConf.logName.empty()){
      *(eConf.logStr)  << argsToCsv(
				    eConf.currentIter,
				    eConf.fitnessAvg,
				    eConf.fitnessMax,
				    eConf.fitnessMin,
				    eConf.fitnessAvgTime,
				    eConf.fitnessAvgOcc,
				    eConf.fitnessAvgCoverage,
				    eConf.actionLenAvg,
				    eConf.actionLenMax,
				    eConf.actionLenMin
				    );
    }
    selection(pool, selected, eConf.selectIndividuals, eConf.selectKeepBest);
    if(secretBest.id > 0 && display){
      debug(eConf.currentIter, ", Fitness: ", secretBest.fitness," : ", argsToCsv(eConf.fitnessAvgTime, eConf.fitnessAvgOcc, eConf.fitnessAvgCoverage));
      rob.evaluateActions(secretBest.actions);
      cv::imshow("Current Run ", rob.gridToImg("map"));
      cv::waitKey(1);
    }
    // pool.clear();

  }

  Logger(eConf.logStr->str(), eConf.logDir, eConf.logName);
  Logger(eConf.fitnessStr->str(), eConf.logDir, eConf.fitnessName);
  // logg("------End-Training------", eConf);

}
///////////////////////////////////////////////////////////////////////////////
//                            Dual Point Crossover                           //
///////////////////////////////////////////////////////////////////////////////

void ga::_Dual_Point_Crossover::selection(Genpool &currentPopulation, Genpool &selectionPool, int individuals, int keepBest){
  Genpool keep;


  selectionPool.clear();
  sort(currentPopulation.begin(), currentPopulation.end());

  // Keep the best individuals
  secretBest = currentPopulation.back();
  if(keepBest > 0){
    assertm(keepBest < currentPopulation.size(), "Cannot keep more individuals than in the pool");
    selectionPool.insert(selectionPool.begin(), prev(currentPopulation.end(), keepBest), currentPopulation.end());
  }


  // Perform roulettwheel selection:
  // Fill the selection Pool with the new Population
  float totalFitness = 0;
  for(auto &gen :currentPopulation){
    totalFitness += gen.fitness;
  }
  assert(totalFitness > 0);
  // Fill selection pool
  while(selectionPool.size() < (individuals + keepBest)){
    if(currentPopulation.size() > 0){
      genome gen1 = roulettWheelSelection(currentPopulation, totalFitness, generator, false);
      genome gen2 = roulettWheelSelection(currentPopulation, totalFitness, generator, false);
      if(gen1.fitness == 0 && gen2.fitness == 0){
	warn("Selected gen with fitness 0, it will be skipped!");
	continue;
      }

      // Crossover will be not performed
      if(selectionDist(generator) > eConf.crossoverProba){
	if(std::find(selectionPool.begin(), selectionPool.end(), gen1) != selectionPool.end()) selectionPool.push_back(gen1);
	if(std::find(selectionPool.begin(), selectionPool.end(), gen2) != selectionPool.end()) selectionPool.push_back(gen2);
	continue;
      }
      // Create offset of the selected gens
      assertm(gen1.actions.size() > 3, "Parent1 has not enough actions for mating!");
      assertm(gen2.actions.size() > 3, "Parent2 has not enough actions for mating!");
      mating(gen1, gen2, selectionPool);
    } else {
      warn("Not enough individuals left in the pool for mating ...");
      assertm(false, "No idividuals in population!");
      break;
    }
  }

  if(keepBest > 0){
    selectionPool.insert(selectionPool.begin(), keep.begin(), keep.end());
  }
}



void ga::_Dual_Point_Crossover::mating(genome &par1, genome &par2, Genpool& newPopulation){
  // mate two parents
  // estimate the individual Length
  // debug("Parent length: ", par1.actions.size(), " ", par2.actions.size());
  PAs parent1, parent2, child1, child2;
  int maxlen1 = static_cast<int>((par1.actions.size() - 3) * eConf.crossLength);
  int maxlen2 = static_cast<int>((par2.actions.size() - 3) * eConf.crossLength);
  assertm(maxlen1 > 0, "Cross length is too small!");
  assertm(maxlen2 > 0, "Cross length is too small!");
  uniform_int_distribution<int> lendist1(3, maxlen1);
  uniform_int_distribution<int> lendist2(3, maxlen2);

  int len1 = lendist1(generator);
  int len2 = lendist2(generator);

  // Ensure that the generated index in still in range
  uniform_int_distribution<int> dist1(2,par1.actions.size() - (len1+1));
  uniform_int_distribution<int> dist2(2,par2.actions.size() - (len2+1));
  // debug("Dist1 bound: ", dist1.max());
  // debug("Dist2 bound: ", dist2.max());
  // calculate the start Index
  int sIdx1 = dist1(generator);
  int sIdx2= dist2(generator);
  // debug("Index1: ", sIdx1, " len: ", len1);
  // debug("Index2: ", sIdx2, " len: ", len2);

  // Cut out the part
  parent1 = par1.actions;
  parent2 = par2.actions;

  // Copy first part of parent 1
  copyActions(par1.actions.begin(), next(par1.actions.begin(), sIdx1), child1);
  // Insert cross over part from parent 2
  copyActions(next(par2.actions.begin(), sIdx2), next(par2.actions.begin(), (sIdx2+len2)), child1, true);
  // Copy what remains of parent 1
  copyActions(next(par1.actions.begin(), sIdx1 +len1), par1.actions.end(), child1);


    // Copy first part of parent 2 to child 2
  copyActions(par2.actions.begin(), next(par2.actions.begin(), sIdx2), child2);
  // Insert cross over part from parent 1
  copyActions(next(par1.actions.begin(), sIdx1), next(par1.actions.begin(), (sIdx1+len1)), child2, true);
  // Copy what remains of parent 1
  copyActions(next(par2.actions.begin(), sIdx2 +len2), par2.actions.end(), child2);

  assertm(child1.size() == (par1.actions.size() - len1 + len2), "Child length after crossover operation does not match!");
  assertm(child2.size() == (par2.actions.size() + len1 - len2), "Child length after crossover operation does not match!");

  // Mark crossings as modified
  (*next(child1.begin(), sIdx1-1))->modified = true;
  (*next(child1.begin(), sIdx1+len2-1))->modified = true;
  (*next(child2.begin(), sIdx2-1))->modified = true;
  (*next(child2.begin(), sIdx2+len1-1))->modified = true;

  // Insert children in pool
  genome child_gen1(child1);
  genome child_gen2(child2);

  assertm(child_gen1.actions.back()->type == PAT::End, "Gen does not end with end action!");
  assertm(child_gen2.actions.back()->type == PAT::End, "Gen does not end with end action!");

  // Calidate the gens
  validateGen(child_gen1);
  validateGen(child_gen2);

  // Insert to new Population
  newPopulation.push_back(child_gen1);
  newPopulation.push_back(child_gen2);
  // debug("Sizes: Par1: ", par1.actions.size(), " Par2: ", par2.actions.size(), " Ch1: ", child1.size(), " Ch2: ", child2.size());

}


void ga::_Dual_Point_Crossover::crossover(ga::Genpool& currentSelection, ga::Genpool& newPopulation) {
  // Single point Crossover
  assertm(currentSelection.size() >= 2, "Not enough individuals for crossover in pool!");

  // TODO: Switch to iterator
  // debug("Crossover");
  for(auto par1 = currentSelection.begin(); par1 != currentSelection.end(); par1++){
    for(auto par2 = next(par1, 1); par2 != currentSelection.end(); par2++){
      ;
      if(selectionDist(generator) > eConf.crossoverProba){
	// Crossover will be not performed
	if(std::find(newPopulation.begin(), newPopulation.end(), *par1) != newPopulation.end()) newPopulation.push_back(*par1);
	if(std::find(newPopulation.begin(), newPopulation.end(), *par2) != newPopulation.end()) newPopulation.push_back(*par2);
	continue;
      }
      assertm(par1->actions.size() > 3, "Parent1 has not enough actions for mating!");
      assertm(par2->actions.size() > 3, "Parent2 has not enough actions for mating!");
      mating(*par1, *par2, newPopulation);
    }
  }
  currentSelection.clear();
}


///////////////////////////////////////////////////////////////////////////////
//                             Mutation Procedure                            //
///////////////////////////////////////////////////////////////////////////////

// void ga::_Dual_Point_Crossover::mutation(Genpool& currentPopulation, Mutation_conf& muat_conf){
//   // debug("Mutate");
//   for (auto &gen : currentPopulation) {
//     addOrthogonalAngleOffset(gen, eConf, generator);
//     // addRandomAngleOffset(gen, eConf, generator);
//     addPositiveDistanceOffset(gen, eConf, generator);
//     // addNegativeDistanceOffset(gen, eConf, generator);
//   }

void ga::_Dual_Point_Crossover::mutation(Genpool& currentPopulation, Genpool& nextPopulation){
  nextPopulation.clear();
  for (auto &gen : currentPopulation) {
    addOrthogonalAngleOffset(gen, eConf, generator);
    // addRandomAngleOffset(gen, eConf, generator);
    addPositiveDistanceOffset(gen, eConf, generator);
    // addNegativeDistanceOffset(gen, eConf, generator);
    nextPopulation.push_back(gen);
  }
}

void ga::_Dual_Point_Crossover::optimizePath(bool display) {

  // Use configuration and obstacle map to create robot and optimize the path
  // log all runtime information here provided by the conf
  // logg("------Start-Training------", eConf);
  Logger(eConf.config_to_string(), eConf.logDir, eConf.logName);

  if(!eConf.logName.empty()){
    Logger("Iteration,FitAvg,FitMax,FitMin,AvgTime,AvgOcc,AvgCoverage,ActionLenAvg,ActionLenMax,ActionLenMin", eConf.logDir, eConf.logName);
  }

  Robot rob(eConf.rob_conf, eConf.gmap, "map");

  Genpool currentPool ,selPool;

  // Populate pool
  populatePool(currentPool, eConf.start, eConf.ends, eConf.initIndividuals, eConf.initActions);
  // Get initial fitness calculation
  evalFitness(currentPool, rob);

  // First selection
  // The population will be in the selPool
  // Crossover is already performed
  // selection(currentPool, selPool, eConf.selectIndividuals, eConf.selectKeepBest);

  // Main loop
  int lowest = 1000;
  int highest = 0;
  for (int i = 0; i < eConf.maxIterations; ++i) {
    eConf.currentIter = i;

    selection(currentPool, selPool, eConf.selectIndividuals, eConf.selectKeepBest);
    // crossover(selPool, currentPool);
    // cout << "Path Length: ";
    // for (auto it = currentPool.begin(); it != currentPool.end(); ++it) {
    //   cout << (*it).getPathLen();
    //   for (auto it2 = next(it, 1); it2 != currentPool.end(); ++it2) {
    // 	if(it->getPathLen() == it2->getPathLen()){
    // 	  warn("Equal genome length detected ...");
    // 	  debug("Gen1:");
    // 	  for (auto ac = it->actions.begin(); ac != it->actions.end(); ++ac) {
    // 	    cout << (*ac)->mod_config[PAP::Distance] << "|";
    // 	  }
    // 	  cout << endl;
    // 	  debug("gen2");
    // 	  for (auto ac = it2->actions.begin(); ac != it2->actions.end(); ++ac) {
    // 	    cout << (*ac)->mod_config[PAP::Distance] << "|";
    // 	  }
    // 	  cout << endl;

    // 	  // assert(it->getPathLen() != it2->getPathLen());
    // 	}
    //   }
    // }
    // Pass the selected and crossed individuals towards the current population
    // Apply mutations
    mutation(selPool, currentPool);
    // debug("Poolsize Mut: ", pool.size());
    evalFitness(currentPool, rob);
    if(!eConf.logName.empty()){
      *(eConf.logStr)  << argsToCsv(
				    eConf.currentIter,
				    eConf.fitnessAvg,
				    eConf.fitnessMax,
				    eConf.fitnessMin,
				    eConf.fitnessAvgTime,
				    eConf.fitnessAvgOcc,
				    eConf.fitnessAvgCoverage,
				    eConf.actionLenAvg,
				    eConf.actionLenMax,
				    eConf.actionLenMin
				    );
    }
    selection(currentPool, selPool, eConf.selectIndividuals, eConf.selectKeepBest);
    if(secretBest.id > 0 && display){
      debug(eConf.currentIter, ", Fitness: ", secretBest.fitness," : ", argsToCsv(eConf.fitnessAvgTime, eConf.fitnessAvgOcc, eConf.fitnessAvgCoverage));
      rob.evaluateActions(secretBest.actions);
      cv::imshow("Current Run ", rob.gridToImg("map"));
      cv::waitKey(1);
    }
    // pool.clear();

  }

  Logger(eConf.logStr->str(), eConf.logDir, eConf.logName);
  Logger(eConf.fitnessStr->str(), eConf.logDir, eConf.fitnessName);
  // logg("------End-Training------", eConf);

}
