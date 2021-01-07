#include "optimizer.h"

bool op::applyAction(float proba, executionConfig eConf){
  uniform_real_distribution<float> probaDist(0,1);
  return proba > probaDist(eConf.generator);
}

/////////////////////////////////////////////////////////////////////////////
//                               InitStrategy                              //
/////////////////////////////////////////////////////////////////////////////
void op::InitStrategy::operator()(Genpool& pool, executionConfig& eConf){
  for(int i=0; i<eConf.initIndividuals;i++){
    genome gen;
    (*this)(gen, eConf.initActions, eConf);
    pool.push_back(gen);
  }
}

void op::InitStrategy::operator()(genome &gen, int len, executionConfig& eConf){
  PAs actions;
  uniform_int_distribution<> angleDist(0,360);
  uniform_real_distribution<float> distanceDist(0,50);
  actions.push_back(make_shared<StartAction>(StartAction(eConf.start)));
    for(int i=0; i<len; i++){
      PA_config config{{PAP::Angle,angleDist(eConf.generator)}, {PAP::Distance, distanceDist(eConf.generator)}};
      actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
    }
    actions.push_back(make_shared<EndAction>(EndAction(eConf.ends)));
    gen.actions = actions;
}

/////////////////////////////////////////////////////////////////////////////
//                            SelectionStrategy                             //
/////////////////////////////////////////////////////////////////////////////
void op::SelectionStrategy::operator()(Genpool& currentPool, SelectionPool& selPool, executionConfig& eConf){
  Genpool keep;
  selPool.clear();
  // Ensure population is not empty
  assert(currentPool.size() > 0);
  assert(eConf.selectKeepBest < currentPool.size());
  // Keep the best individuals in the population for the next generation
  sort(currentPool.begin(), currentPool.end());
  keep.insert(keep.begin(),
	      prev(currentPool.end(), eConf.selectKeepBest),
	      currentPool.end());

  // Start selection process
  // TODO: Remember select individuals is now the value of the selected pairs!
  while(selPool.size() < eConf.selectIndividuals){
    // Draw individuals
    auto couple = make_pair(selection(currentPool, eConf),
	      selection(currentPool, eConf));
    // Check if fitness is high enough
    // 0 fitness indicates a defect gen
    if(couple.first.fitness == 0
       || couple.second.fitness == 0){
      warn("Selection: skip gen with fitness 0");
      continue;
    }

    // Insert into the selection Pool
    selPool.push_back(couple);
  }

  currentPool.clear();
  // Insert preserved gens to the pool
  currentPool.insert(currentPool.begin(),
		     keep.begin(),
		     keep.end());

}

genome op::RouletteWheelSelection::selection(ga::Genpool &currentPopulation, executionConfig &eConf){
  // Calculate the total fitness value

  float totalFitness = eConf.fitnessAvg * currentPopulation.size();
  //TODO: Test if the above formula holds true
  float testSum = 0;
  for (auto it = currentPopulation.begin(); it != currentPopulation.end(); ++it) {
    testSum += it->fitness;
  }
  assert(testSum == totalFitness);

  std::uniform_real_distribution<float> selDistr(0.0,1);

  float rand = selDistr(eConf.generator);
  // cout << "Wheel: " << rand << endl;
  float offset = 0.0;
  genome gen;

  for(auto it = currentPopulation.begin(); it != currentPopulation.end(); it++){
    offset += it->fitness / totalFitness;
    if(rand < offset){
      gen = *it;
      break;
    }
  }
  return gen;
}
/////////////////////////////////////////////////////////////////////////////
//                            CrossoverStrategy                             //
/////////////////////////////////////////////////////////////////////////////
void op::DualPointCrossover::operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf){
  assert(selPool.size() > 0);
  for (auto it = selPool.begin(); it != selPool.end(); ++it) {
    if(!applyAction(eConf.crossoverProba, eConf)){
      // Add gens to pool if they are not already inside
      if(std::find(nextPool.begin(), nextPool.end(), it->first)
	 != nextPool.end())
	nextPool.push_back(it->first);
      if(std::find(nextPool.begin(), nextPool.end(), it->second)
	 != nextPool.end())
	nextPool.push_back(it->second);
	continue;
    }
    assert(it->first.actions.size() > 3);
    assert(it->second.actions.size() > 3);
    mating(it->first, it->second, nextPool, eConf);
  }
}

void op::CrossoverStrategy::copyActions(PAs::iterator begin, PAs::iterator end, PAs &child, bool modify){
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
  }
}


void op::DualPointCrossover::mating(genome &par1, genome &par2, Genpool& newPopulation, executionConfig eConf){
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

  int len1 = lendist1(eConf.generator);
  int len2 = lendist2(eConf.generator);
  // Ensure that the generated index in still in range
  uniform_int_distribution<int> dist1(2,par1.actions.size() - (len1+1));
  uniform_int_distribution<int> dist2(2,par2.actions.size() - (len2+1));
  // calculate the start Index
  int sIdx1 = dist1(eConf.generator);
  int sIdx2= dist2(eConf.generator);
  // Cut out the part
  // Copy first part of parent 1
  copyActions(par1.actions.begin(),
	      next(par1.actions.begin(), sIdx1),
	      child1);
  // Insert cross over part from parent 2
  // Mark the inserted part as modified to recalculate waypoints
  copyActions(next(par2.actions.begin(), sIdx2),
	      next(par2.actions.begin(), (sIdx2+len2)),
	      child1, true);
  // Append remaining part
  copyActions(next(par1.actions.begin(), sIdx1 +len1),
	      par1.actions.end(),
	      child1);
    // Copy first part of parent 2 to child 2
  copyActions(par2.actions.begin(),
	      next(par2.actions.begin(), sIdx2),
	      child2);
  copyActions(next(par1.actions.begin(), sIdx1),
	      next(par1.actions.begin(), (sIdx1+len1)),
	      child2, true);
  copyActions(next(par2.actions.begin(), sIdx2 +len2),
	      par2.actions.end(),
	      child2);

  // Check if new length match
  assert(child1.size() == (par1.actions.size() - len1 + len2));
  assert(child2.size() == (par2.actions.size() + len1 - len2));

  // Mark crossings as modified
  (*next(child1.begin(), sIdx1-1))->modified = true;
  (*next(child1.begin(), sIdx1+len2-1))->modified = true;
  (*next(child2.begin(), sIdx2-1))->modified = true;
  (*next(child2.begin(), sIdx2+len1-1))->modified = true;

  // Insert children in pool
  genome child_gen1(child1);
  genome child_gen2(child2);
  assert(child_gen1.actions.back()->type == PAT::End
	 && child_gen1.actions.back()->type == PAT::End);

  // Validate the gens
  validateGen(child_gen1);
  validateGen(child_gen2);

  // Insert to new Population
  newPopulation.push_back(child_gen1);
  newPopulation.push_back(child_gen2);
}

/////////////////////////////////////////////////////////////////////////////
//                             MutationStrategy                            //
/////////////////////////////////////////////////////////////////////////////

void op::MutationStrategy::operator()(Genpool& currentPool, executionConfig& eConf){
  for (auto &gen : currentPool) {
    addOrthogonalAngleOffset(gen, eConf);
    addRandomAngleOffset(gen, eConf);
    addPositiveDistanceOffset(gen, eConf);
    addNegativeDistanceOffset(gen, eConf);
  }
}

void op::MutationStrategy::addOrthogonalAngleOffset(genome& gen, executionConfig& eConf){
  if(!applyAction(eConf.mutaOrtoAngleProba, eConf)) return;
  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_int_distribution<int> changeDistro(0,1);

  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  (*action)->mod_config[PAP::Angle] += changeDistro(eConf.generator) ? 90 : -90;
  (*action)->modified = true;
}

void op::MutationStrategy::addRandomAngleOffset(genome& gen, executionConfig& eConf) {
  if(!applyAction(eConf.mutaRandAngleProba, eConf)) return;

  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_int_distribution<int> changeDistro(0,360);
  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  (*action)->mod_config[PAP::Angle] += changeDistro(eConf.generator);
  (*action)->modified = true;
}

void op::MutationStrategy::addPositiveDistanceOffset(genome& gen, executionConfig& eConf) {

  if(!applyAction(eConf.mutaPosDistProba, eConf)) return;
  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_real_distribution<> changeDistro(0,eConf.mutaPosDistMax);
  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  (*action)->mod_config[PAP::Distance] += changeDistro(eConf.generator);
  (*action)->modified = true;
}

void op::MutationStrategy::addNegativeDistanceOffset(genome& gen, executionConfig& eConf) {

  if(!applyAction(eConf.mutaNegDistProba, eConf)) return;
  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_real_distribution<> changeDistro(0,eConf.mutaPosDistMax);
  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  float offset = changeDistro(eConf.generator);
  if((*action)->mod_config[PAP::Distance] > offset){
    (*action)->mod_config[PAP::Distance] -= offset;
    (*action)->modified = true;
  }
}

///////////////////////////////////////////////////////////////////////////////
//                              Fitness strategy                         //
///////////////////////////////////////////////////////////////////////////////
void op::FitnessStrategy::operator()(Genpool &currentPool, path::Robot &rob, executionConfig eConf){
   eConf.fitnessAvg = 0;
  eConf.fitnessMax = 0;
  eConf.fitnessMin = 1;
  eConf.fitnessAvgTime = 0;
  eConf.fitnessAvgOcc = 0;
  eConf.fitnessAvgCoverage = 0;
  eConf.actionLenMax = 0;
  eConf.actionLenMin = 100000;
  eConf.actionLenAvg = 0;
  for(Genpool::iterator it = currentPool.begin(); it != currentPool.end();){
    assertm(it->actions.size() > 0, "Not enough actions");
    if(rob.evaluateActions(it->actions)){
      assertm(it->actions.size() > 0, "Not enough actions");
      float Cdistance = 0; // Cleand distance
      float distance = 0; // uncleand distance
      float occ = 0;
      int crossed = 0;
      // iterate over all actions
      for(auto &action : it->actions){
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
      float fitness = calculation(
				  Cdistance,
				  distance,
				  crossed,
				  rob.getConfig()[RP::Clean_speed_cm_s] / 100,
				  rob.getConfig()[RP::Drive_speed_cm_s] / 100,
				  rob.getFreeArea(),
				  eConf);
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
      it = currentPool.erase(it);
    }
  }
  eConf.fitnessAvg /= currentPool.size();
  eConf.fitnessAvgTime /= currentPool.size();
  eConf.fitnessAvgOcc /= currentPool.size();
  eConf.fitnessAvgCoverage /= currentPool.size();
  eConf.actionLenAvg /= currentPool.size();
}

float op::FitnessStrategy::calculation(float cdist, float dist, int crossed, float cSpeed_m_s, float speed_m_s, int freeSpace, executionConfig eConf){
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
  return fitness;
}

/////////////////////////////////////////////////////////////////////////////
//                                Optimizer                                 //
/////////////////////////////////////////////////////////////////////////////
