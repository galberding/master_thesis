#include "optimizer.h"

bool op::applyAction(float proba, executionConfig& eConf){
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
  // gen.actions.clear();
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

void op::InitStrategy::replaceZeroGensWithRandom(Genpool& pool){
  debug("Replace phase: ");


}

/////////////////////////////////////////////////////////////////////////////
//                            SelectionStrategy                             //
/////////////////////////////////////////////////////////////////////////////

// void op::SelectionStrategy::operator()(Genpool& currentPool, FamilyPool& selPool, executionConfig& eConf){
//   Genpool keep;
//   selPool.clear();
//   // Ensure population is not empty
//   assert(currentPool.size() > 0);
//   assert(eConf.selectKeepBest < currentPool.size());
//   // Keep the best individuals in the population for the next generation
//   sort(currentPool.begin(), currentPool.end());
//   eConf.best = currentPool.back();
//   keep.insert(keep.begin(),
// 	      prev(currentPool.end(), eConf.selectKeepBest),
// 	      currentPool.end());

//   // Start selection process
//   // TODO: Remember select individuals is now the value of the selected pairs!
//   while(selPool.size() < eConf.selectIndividuals / 2){
//     // Draw individuals
//     vector<genome> couple = {selection(currentPool, eConf),
// 	      selection(currentPool, eConf)};
//     // Check if fitness is high enough
//     // 0 fitness indicates a defect gen
//     if(couple[0].fitness == 0
//        || couple[1].fitness == 0){
//       warn("Selection: skip gen with fitness 0");
//       continue;
//     }

//     // Insert into the selection Pool
//     selPool.push_back(couple);
//   }

//   currentPool.clear();
//   // Insert preserved gens to the pool
//   currentPool.insert(currentPool.begin(),
// 		     keep.begin(),
// 		     keep.end());

// }

void op::SelectionStrategy::operator()(Genpool& currentPool, SelectionPool& selPool, executionConfig& eConf){
  Genpool keep;
  selPool.clear();
  // Ensure population is not empty
  assert(currentPool.size() > 0);
  assert(eConf.selectKeepBest < currentPool.size());
  // Keep the best individuals in the population for the next generation
  sort(currentPool.begin(), currentPool.end());
  eConf.best = currentPool.back();
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

void op::SelectionStrategy::uniformSelectionWithoutReplacement(Genpool &pool, FamilyPool &fPool, executionConfig &eConf){
  fPool.clear();
  // Shuffle will reorder the elements in random order
  shuffle(pool.begin(), pool.end(), eConf.generator);
  // sort(pool.begin(), pool.end());
  for (auto it = pool.begin(); it != pool.end();) {
    auto itn = next(it, 1);
    if(it != pool.end() && itn != pool.end()){
      fPool.push_back({*it, *itn});
      it = next(itn, 1);
    }else{
      break;
    }
  }
  pool.clear();
}

genome op::SelectionStrategy::tournamentSelection(Genpool &pool, executionConfig &eConf){

  assert(pool.size() > eConf.tournamentSize);
  deque<genome> turn;
  shuffle(pool.begin(), pool.end(), eConf.generator);
  turn.insert(turn.begin(), pool.begin(), next(pool.begin(), eConf.tournamentSize));
  sort(turn.begin(), turn.end());
  return turn.back();
}


void op::SelectionStrategy::elitistSelection(FamilyPool& fPool, Genpool& pool){
  // select best two out of four
  // we expect that all individuals in the family pool have their fitness calculated
  // pool.clear();
  for(auto &family : fPool){
    if(family.size() == 2) continue;
    assert(family.size() >= 4);
    sort(family.begin(), family.end());
    pool.push_back(family.back());
    family.pop_back();
    pool.push_back(family.back());
  }

}

genome op::SelectionStrategy::selection(Genpool &currentPopulation,
					executionConfig &eConf) {return genome();}


genome op::RouletteWheelSelection::selection(Genpool &currentPopulation, executionConfig &eConf){
  // Calculate the total fitness value

  float totalFitness = eConf.fitnessAvg * currentPopulation.size();
  //TODO: Test if the above formula holds true
  float testSum = 0;
  for (auto it = currentPopulation.begin(); it != currentPopulation.end(); ++it) {
    testSum += it->fitness;
  }

  std::uniform_real_distribution<float> selDistr(0.0,1);

  float rand = selDistr(eConf.generator);
  // debug("Random val: ", rand);
  // cout << "Wheel: " << rand << endl;
  float offset = 0.0;
  genome gen;

  for(auto it = currentPopulation.begin(); it != currentPopulation.end(); it++){
    offset += it->fitness / totalFitness;
    if(rand < offset){
      gen = *it;
      // debug("ID: ", gen.id);
      break;
    }
  }
  return gen;
}

genome op::TournamentSelection::selection(Genpool &currentPopulation, executionConfig &eConf) {
  return tournamentSelection(currentPopulation, eConf);
}

/////////////////////////////////////////////////////////////////////////////
//                            CrossoverStrategy                             //
/////////////////////////////////////////////////////////////////////////////
void op::DualPointCrossover::operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf){
  assert(selPool.size() > 0);
  for (auto it = selPool.begin(); it != selPool.end(); ++it) {
    if(!applyAction(eConf.crossoverProba, eConf) or !mating(it->first, it->second, nextPool, eConf)){
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
    // mating(it->first, it->second, nextPool, eConf);
  }
}

void op::DualPointCrossover::operator()(FamilyPool& fPool, Genpool& pool , executionConfig& eConf){
  assert(fPool.size() > 0);
  for (auto &family : fPool) {
    assert(family.size() == 2);
    // Check if crossover can be performed
    if(!applyAction(eConf.crossoverProba, eConf) or !mating(family[0], family[1], family, eConf)){
      // Add gens to pool if they are not already inside
      pool.push_back(family[0]);
      pool.push_back(family[1]);
	continue;
    }
    // assert(family[0].actions.size() > 3);
    // assert(family[1].actions.size() > 3);
    // mating(family[0], family[1], family, eConf);
    // assert(family.size() >= 4);
  }
}


void op::CrossoverStrategy::copyActions(PAs::iterator begin, PAs::iterator end, PAs &child, bool modify){
  for(begin; begin != end; begin++){
    // debug("Type: ", (int) (*begin)->type);
    assert((*begin)->wps.size() > 0);
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

genome op::DualPointCrossover::getChild(PAs par1, PAs par2, int sIdx[2], int len[2], bool move){
  PAs child;

  copyActions(par1.begin(),
	      next(par1.begin(), sIdx[0]),
	      child);
  // Insert cross over part from parent 2
  // Mark the inserted part as modified to recalculate waypoints
  copyActions(next(par2.begin(), sIdx[1]),
	      next(par2.begin(), (sIdx[1]+len[1])),
	      child, move); // if move is true we loose locality
  // Append remaining part
  copyActions(next(par1.begin(), sIdx[0] + len[0]),
	      par1.end(),
	      child);

  // Check if copy was successful
  assert(child.size() == (par1.size() - len[0] + len[1]));

  // Mark actions as
  (*next(child.begin(), sIdx[0] - 1))->modified = true;
  (*next(child.begin(), sIdx[0] + len[1] - 1))->modified = true;
  genome child_gen(child);
  validateGen(child_gen);
  assert(child_gen.actions.front()->type == PAT::Start
	 && child_gen.actions.back()->type == PAT::End);
  return child_gen;
}

template <typename T>
bool op::DualPointCrossover::mating(genome &par1, genome &par2, T& newPopulation, executionConfig& eConf){
  // mate two parents
  // estimate the individual Length
  int minActionCount = eConf.getMinGenLen();

  if (par1.actions.size() < minActionCount
      or par2.actions.size() < minActionCount)
    return false;
  PAs parent1, parent2, child1, child2, child3, child4;
  int maxlen1 = static_cast<int>((par1.actions.size() - 2) * eConf.crossLength);
  int maxlen2 = static_cast<int>((par2.actions.size() - 2) * eConf.crossLength);
  // debug("Parent1: ", par1.actions.size(), " Parent2: ", par2.actions.size(), " ", maxlen1, " ", maxlen2, " ", minActionCount);
  assertm(maxlen1 > 2, "Cross length is too small!");
  assertm(maxlen2 > 2, "Cross length is too small!");
  uniform_int_distribution<int> lendist1(2, maxlen1);
  uniform_int_distribution<int> lendist2(2, maxlen2);

  int len[2];
  int len1 = len[0] = lendist1(eConf.generator);
  int len2 = len[1] = lendist2(eConf.generator);
  // Ensure that the generated index in still in range
  uniform_int_distribution<int> dist1(2,par1.actions.size() - (len1+1));
  uniform_int_distribution<int> dist2(2,par2.actions.size() - (len2+1));
  // calculate the start Index
  int sIdx[2];
  int sIdx1 = sIdx[0] = dist1(eConf.generator);
  int sIdx2= sIdx[1] = dist2(eConf.generator);
  vector<genome> loc, glob;

  // Calculate children for first parent
  loc.push_back(getChild(par1.actions, par2.actions, sIdx, len, true));
  glob.push_back(getChild(par1.actions, par2.actions, sIdx, len, false));
  sIdx[0] = sIdx2;
  sIdx[1] = sIdx1;
  len[0] = len2;
  len[1] = len1;
  // Calculate children for second parent
  loc.push_back(getChild(par2.actions, par1.actions, sIdx, len, true));
  glob.push_back(getChild(par2.actions, par1.actions, sIdx, len, false));

  // Insert to new Population
  switch(eConf.crossChildSelector){
  case 0:{
    newPopulation.insert(newPopulation.end(), loc.begin(), loc.end());
    break;
  }
  case 1:{
    newPopulation.insert(newPopulation.end(), glob.begin(), glob.end());
    break;
  }
  case 2:{
    newPopulation.insert(newPopulation.end(), loc.begin(), loc.end());
    newPopulation.insert(newPopulation.end(), glob.begin(), glob.end());
    break;
  }
  default:{
    assertm(false, "Wrong value for child selector");
  }
  }
  return true;
}

/////////////////////////////////////////////////////////////////////////////
//                             MutationStrategy                            //
/////////////////////////////////////////////////////////////////////////////

void op::MutationStrategy::mutateGen(genome &gen, executionConfig &eConf) {
    addOrthogonalAngleOffset(gen, eConf);
    addRandomAngleOffset(gen, eConf);
    addPositiveDistanceOffset(gen, eConf);
    addNegativeDistanceOffset(gen, eConf);
    randomScaleDistance(gen, eConf);
    // randomReplaceGen(gen, eConf);
}

void op::MutationStrategy::operator()(Genpool& currentPool, executionConfig& eConf){
  for (auto &gen : currentPool) {
    mutateGen(gen, eConf);
  }
}

void op::MutationStrategy::operator()(FamilyPool& fPool, executionConfig& eConf){
  for (auto &family : fPool) {
    if(family.size() == 2) continue;
    assert(family.size() >= 4);
    for(int i=2; i<family.size(); i++){
      mutateGen(family[i], eConf);
    }
    // mutateGen(family[3], eConf);
  }
}


bool op::MutationStrategy::addOrthogonalAngleOffset(genome& gen, executionConfig& eConf){
  if(!applyAction(eConf.mutaOrtoAngleProba, eConf)) return false;
  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_int_distribution<int> changeDistro(0,1);

  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  (*action)->mod_config[PAP::Angle] += changeDistro(eConf.generator) ? 90 : -90;
  (*action)->modified = true;
  return true;
}

bool op::MutationStrategy::addRandomAngleOffset(genome& gen, executionConfig& eConf) {
  if(!applyAction(eConf.mutaRandAngleProba, eConf)) return false;

  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_int_distribution<int> changeDistro(0,360);
  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  (*action)->mod_config[PAP::Angle] += changeDistro(eConf.generator);
  (*action)->modified = true;
  return true;
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


bool op::MutationStrategy::randomScaleDistance(genome& gen, executionConfig& eConf) {

  if(!applyAction(eConf.mutaRandScaleDistProba, eConf)) return false;
  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_real_distribution<float> changeDistro(0.5,2);
  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  float offset = changeDistro(eConf.generator);
  // debug("Mut scale: ", offset);
  if((*action)->mod_config[PAP::Distance] > 0){
    (*action)->mod_config[PAP::Distance] *= offset;
    (*action)->modified = true;
  }else{
    // Use mean traveled distance to revive a zero action
    gen.updateGenParameter();
    (*action)->mod_config[PAP::Distance] = (gen.traveledDist / gen.actions.size()) * offset;
  }
  return true;
}

bool op::MutationStrategy::randomReplaceGen(genome& gen, executionConfig& eConf) {

  if(!applyAction(eConf.mutaReplaceGen, eConf)) return false;
  // debug("Insert Random!");
  InitStrategy init;
  init(gen, eConf.getMinGenLen() + 10, eConf);
  validateGen(gen);
  return true;
}

// void op::MutationStrategy::reviveZeroAction(genome& gen, executionConfig& eConf) {

//   if(!applyAction(eConf.mutaReviveAction, eConf)) return;
//   // debug("Insert Random!");
//   InitStrategy init;
//   init(gen, gen.actions.size(), eConf);
// }


// void op::MutationStrategy::addRandomDistanceOffset(genome& gen, executionConfig& eConf) {
//   if(!applyAction(eConf.mutaRandDistProba, eConf)) return;
//   uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
//   uniform_real_distribution<> changeDistro(0,eConf.mutaPosDistMax);

//   // Select action and add the offset
//   auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
//   float offset = changeDistro(eConf.generator);
//   if(offset < 0 && (*action)->mod_config[PAP::Distance] < abs(offset)) return;
//     (*action)->mod_config[PAP::Distance] -= offset;
//     (*action)->modified = true;

// }

///////////////////////////////////////////////////////////////////////////////
//                              Fitness strategy                         //
///////////////////////////////////////////////////////////////////////////////

void resetLoggingFitnessParameter(executionConfig& eConf){
  eConf.fitnessAvg = 0;
  eConf.fitnessMax = 0;
  eConf.fitnessMin = 1;
  eConf.fitnessAvgTime = 0;
  eConf.fitnessAvgOcc = 0;
  eConf.fitnessAvgCoverage = 0;
  eConf.actionLenMax = 0;
  eConf.actionLenMin = 100000;
  eConf.actionLenAvg = 0;
}


void trackFitnessParameter(genome& gen, executionConfig& eConf){
  float fitness = gen.fitness;
  // Logging of fittnessvalues
  if(fitness > eConf.fitnessMax) eConf.fitnessMax = fitness;
  if(fitness < eConf.fitnessMin) eConf.fitnessMin = fitness;
  auto size = gen.actions.size();
  if(size > eConf.actionLenMax) eConf.actionLenMax = size;
  if(size < eConf.actionLenMin) eConf.actionLenMin = size;
  eConf.fitnessAvg += fitness;
  eConf.actionLenAvg += gen.actions.size();
  eConf.fitnessAvgTime += gen.finalTime;
  eConf.fitnessAvgCoverage += gen.finalCoverage;
}


void finalizeFitnessLogging(int poolsize, executionConfig& eConf){
  assert(poolsize > 0);
  eConf.fitnessAvg /= poolsize;
  eConf.fitnessAvgTime /= poolsize;
  eConf.fitnessAvgOcc /= poolsize;
  eConf.fitnessAvgCoverage /= poolsize;
  eConf.actionLenAvg /= poolsize;
}


void trackPoolFitness(Genpool& pool, executionConfig& eConf){
  resetLoggingFitnessParameter(eConf);
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    trackFitnessParameter(*it, eConf);
  }

  finalizeFitnessLogging(pool.size(), eConf);

}



void op::FitnessStrategy::operator()(Genpool &currentPool, path::Robot &rob, executionConfig& eConf){
  resetLoggingFitnessParameter(eConf);
  // debug("Before cal");
  assert(currentPool.size() > 0);
  for(Genpool::iterator it = currentPool.begin(); it != currentPool.end(); ++it){
    estimateGen(*it, rob, eConf);
    (*it).trail = 1 * (*eConf.gmap)["map"];
  }
  finalizeFitnessLogging(currentPool.size(), eConf);
  // debug("After Finalize");
}

void op::FitnessStrategy::estimateGen(genome &gen, path::Robot &rob, executionConfig& eConf){
  assertm(gen.actions.size() > 0, "Not enough actions");
  if(rob.evaluateActions(gen.actions)){
    assertm(gen.actions.size() > 0, "Not enough actions");
    calculation(gen, rob.getFreeArea(), eConf);
    trackFitnessParameter(gen , eConf);
  }else{
    warn("Erase Gen!");
    assertm(false, "Attempt to erase a gen!!");
  }
}

void op::FitnessStrategy::estimateChildren(FamilyPool& fPool, path::Robot &rob, executionConfig& eConf) {
  // Deactivate logging -> recalculate with
  // resetLoggingFitnessParameter(eConf);
  for (auto &family : fPool) {
    if(family.size() > 2){
      for (int i = 2; i < family.size(); i++) {
        assertm(family[i].actions.size() > 0, "Not enough actions");
        assert(rob.evaluateActions(family[i].actions));
        // assertm(family[i].actions.size() > 0, "Not enough actions");
        calculation(family[i], rob.getFreeArea(), eConf);
        // trackFitnessParameter(family[i] , eConf);
      }
    }
  }
  // finalizeFitnessLogging(currentPool.size(), eConf);
}


float op::FitnessStrategy::calculation(genome& gen, int freeSpace, executionConfig &eConf){
  // prepare parameters
  // Check if the gen is valid -> returns false if gen has distance 0
  if(!gen.updateGenParameter()){
    debug("Update: ", gen.updateGenParameter());
    return 0;
  }
  // Time parameter:
  float actualTime = gen.traveledDist;
  // debug(log(10 + gen.cross));
  float optimalTime = gen.traveledDist - gen.cross;
  float finalTime = optimalTime / actualTime;

  float currentCoverage = gen.traveledDist - gen.cross;
  float totalCoverage = freeSpace;
  float finalCoverage = currentCoverage / totalCoverage;

  // finalTime *= finalCoverage;

  float weight = eConf.fitnessWeights[2];
  gen.finalCoverage = finalCoverage;
  gen.finalTime = finalTime;
  eConf.fitnessAvgTime += finalTime;
  eConf.fitnessAvgCoverage += finalCoverage;
  // eConf.fitnessAvgOcc += final_occ;
  float x = finalTime;
  float y = finalCoverage;
  // gen.fitness = (weight*finalTime + (1-weight)*finalCoverage) * (finalTime*finalCoverage);
  gen.fitness = (0.25*(0.5*x + 0.5*y) + 0.25*x*y + 0.25*(pow(x, 2) * pow(y, 2)) + 0.25*(0.5*pow(x, 2) + 0.5*pow(y, 2)))*(pow(x, 5)*pow(y, 4));
  // gen.fitness = (pow(x, 5)*pow(y, 4));
  // gen.fitness = (0.5*x + 0.5*y)*(pow(x, 3)*pow(y, 2));
  // gen.fitness = 0.5*(0.5*x + 0.5*y) + 0.5*(x*y); // 370~600 -> turning point
  return gen.fitness;
}

float op::FitnessStrategy::calculation(float cdist, float dist, int crossed, float cSpeed_m_s, float speed_m_s, int freeSpace, executionConfig& eConf){
assert(cdist >= crossed);

  float actual_time = cdist + dist;
  float optimal_time = (cdist - crossed);
  float final_time = optimal_time / actual_time;


  // Calculate the final occ based on the
  // TODO: rename fitness parameter, sth like crossFactor or overlapping
  float actual_occ = cdist - crossed;
  // if (current_occ < 0){
  //   current_occ = crossed;
  // }
  float optimal_occ = cdist + crossed;

  float final_occ = actual_occ / optimal_occ ;

  // Area coverage
  float final_coverage = actual_occ / freeSpace;
  assertm(freeSpace >= actual_occ , "No space to cover");
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

void op::Optimizer::logAndSnapshotPool(executionConfig& eConf, float zeros){

  // Write initial logfile
  if(eConf.currentIter == 0){
      *eConf.logStr << "Iteration,FitAvg,FitMax,FitMin,AvgTime,AvgCoverage,ActionLenAvg,ActionLenMax,ActionLenMin,Zeros,BestTime,BestCov\n";
      logging::Logger(eConf.logStr->str(), eConf.logDir, eConf.logName);
      eConf.logStr->str("");
  }
   if(!eConf.logName.empty()){
      *(eConf.logStr)  << argsToCsv(
				    eConf.currentIter,
				    eConf.fitnessAvg,
				    eConf.fitnessMax,
				    eConf.fitnessMin,
				    eConf.fitnessAvgTime,
				    // eConf.fitnessAvgOcc,
				    eConf.fitnessAvgCoverage,
				    eConf.actionLenAvg,
				    eConf.actionLenMax,
				    eConf.actionLenMin,
				    zeros,
				    // eConf.best.fitness,
				    eConf.best.finalTime,
				    eConf.best.finalCoverage
				    );
    }
    if(eConf.takeSnapshot && (eConf.currentIter % eConf.takeSnapshotEvery == 0)){
      // debug("Take snapshot to: ", eConf.tSnap);
      snapshotPopulation(eConf);

      logging::Logger(eConf.logStr->str(), eConf.logDir, eConf.logName, true);
      eConf.logStr->str("");
    }
}

void op::Optimizer::printRunInformation(executionConfig& eConf, float zeroPercent, bool display){
  if(eConf.best.id > 0 && display){
      debug(eConf.currentIter, ", MaxFitness: ",
	    eConf.best.fitness, " (", eConf.best.finalTime, ", ", eConf.best.finalCoverage,") : ",
	    argsToCsv(eConf.fitnessAvgTime,
		      // eConf.fitnessAvgOcc,
		      eConf.fitnessAvgCoverage,
		      eConf.actionLenAvg, zeroPercent));
      if(eConf.visualize){
	// cv::Mat src;
	rob->evaluateActions(eConf.best.actions);
	// eConf.best.trail = (*eConf.gmap)["map"];
	// cv::eigen2cv(eConf.best.trail, src);
	cv::imshow("Current Run ", rob->gridToImg("map"));
	cv::waitKey(1);
      }
    }
}


void op::Optimizer::restorePopulationFromSnapshot(const string path){
  vector<PAs> pp;
  pa_serializer::readActrionsFromFile(pp, path);
  for (auto it = pp.begin(); it != pp.end(); ++it) {
    pool.push_back(genome(*it));
  }
}

void op::Optimizer::snapshotPopulation(const string path){
  vector<PAs> pp;
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    pp.push_back(it->actions);
  }
  pa_serializer::writeActionsToFile(pp, path);
}

void op::Optimizer::snapshotPopulation(executionConfig& eConf){
  // Take the current iteration into account
  string iter = to_string(eConf.currentIter);
  // Save Genpool:
  string popName = eConf.logDir + "/" + iter + "_" + eConf.tSnap;
  string performanceName = iter + "_" + eConf.tPerformanceSnap;
  // Store gen information
  ostringstream perform;
  vector<PAs> pp;
  perform << argsToCsv("fitness", "traveledDist", "cross", "fTime", "fCoverage", "#actions");

  for (auto it = pool.begin(); it != pool.end(); ++it) {
    pp.push_back(it->actions);
    perform << argsToCsv(it->fitness, it->traveledDist, it->cross, it->finalTime, it->finalCoverage, it->actions.size());
  }
  logging::Logger(perform.str(), eConf.logDir, performanceName);
  pa_serializer::writeActionsToFile(pp, popName);
}


void getBestGen(Genpool& pool, executionConfig& eConf){
  eConf.best = pool.front();
  bool foundBest = false;
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    if(it->fitness >  eConf.best.fitness){
      eConf.best = *it;
      // debug("New best");
      if(it->fitness > eConf.crossBestFit){
	eConf.crossBestFit = it->fitness;
	foundBest = true;
      }
    }
  }
  if(foundBest){
    eConf.crossAdapter = 0;
  }else{
    eConf.crossAdapter++;
  }
}

void adaptCrossover(executionConfig& eConf){
  float lower = 0.4;
  float upper = 0.85;

  // if(eConf.crossAdapter < 25){
  //   eConf.crossoverProba -= 0.01;
  // }else if(eConf.crossAdapter < 50){
  //   eConf.crossoverProba += 0.01;
  // }
  // if(eConf.crossoverProba < lower)
  //   eConf.crossoverProba = lower;

  // if(eConf.crossoverProba > upper)
  //   eConf.crossoverProba = upper;

  if(eConf.currentIter < 1000){

    // eConf.crossoverProba += 0.0005;
    eConf.crossLength -= 0.0003;
  }


}

void clearZeroPAs(Genpool& pool, executionConfig& eConf){
  if(eConf.clearZeros > 0 and eConf.currentIter % eConf.clearZeros == 0)
    genome_tools::removeZeroPAs(pool);
}




///////////////////////////////////////////////////////////////////////////////
//                         Elitist Selection Scenario                        //
///////////////////////////////////////////////////////////////////////////////

void op::Optimizer::optimizePath(bool display){
  if(!eConf.restore){
    (*init)(pool, eConf);
  } else {
    pool.clear();
    restorePopulationFromSnapshot(eConf.snapshot);
  }

  // Main loop
  (*calFitness)(pool, *rob, eConf);
  genome_tools::removeZeroPAs(pool);
  // debug("Before while");
  while(eConf.currentIter <= eConf.maxIterations){
    // debug("Inside");
    // Fitness calculation
    // TODO: only useful for statistic evaluation -> all parameters are already recalculated
    getBestGen(pool, eConf);
    adaptCrossover(eConf);
    // resetLoggingFitnessParameter(eConf);
    // genome_tools::removeZeroPAs(pool);
    trackPoolFitness(pool, eConf);
    float zeroPercent = calZeroActionPercent(pool);

    clearZeroPAs(pool, eConf);
    logAndSnapshotPool(eConf, zeroPercent);
    printRunInformation(eConf, zeroPercent, display);

    // Selection
    select->uniformSelectionWithoutReplacement(pool, fPool, eConf);

    // Crossover
    (*cross)(fPool, pool, eConf);
    // Mutation
    (*mutate)(fPool, eConf);
    calFitness->estimateChildren(fPool, *rob, eConf);
    select->elitistSelection(fPool, pool);
    // Second mutation stage:
    sort(pool.begin(), pool.end());

    for (auto it = pool.begin(); it != next(pool.begin(), pool.size() - 2); ++it) {
      bool mutated = mutate->randomReplaceGen(*it, eConf);
      if(not mutated){
	mutated |= mutate->addRandomAngleOffset(*it, eConf);
	mutated |= mutate->addOrthogonalAngleOffset(*it, eConf);
	mutated |= mutate->randomScaleDistance(*it, eConf);
      }
      if(mutated){
	calFitness->estimateGen(*it, *rob, eConf);
	it->trail = 1 * (*eConf.gmap)["map"];
      }
    }

    // Increase Iteration
    eConf.currentIter++;
  }
  // Log Fitnessvalues for all iterations
  logging::Logger(eConf.logStr->str(), eConf.logDir, eConf.logName, true);
}


///////////////////////////////////////////////////////////////////////////////
//                       Tournament Selection Scenario                       //
///////////////////////////////////////////////////////////////////////////////


void op::Optimizer::optimizePath_s_tourn_c_dp(bool display){

  TournamentSelection tSelect;

  if(!eConf.restore){
    (*init)(pool, eConf);
  } else {
    pool.clear();
    restorePopulationFromSnapshot(eConf.snapshot);
  }
  // Main loop
  (*calFitness)(pool, *rob, eConf);
  // genome_tools::removeZeroPAs(pool);
  // debug("Before while");
  while(eConf.currentIter <= eConf.maxIterations){
    // debug("Inside");
    // Fitness calculation
    // TODO: only useful for statistic evaluation -> all parameters are already recalculated
    getBestGen(pool, eConf);
    // resetLoggingFitnessParameter(eConf);
    // genome_tools::removeZeroPAs(pool);
    trackPoolFitness(pool, eConf);
    float zeroPercent = calZeroActionPercent(pool);

    clearZeroPAs(pool, eConf);
    logAndSnapshotPool(eConf, zeroPercent);
    printRunInformation(eConf, zeroPercent, display);
    // Selection
    // (*select)(pool, sPool, eConf);
    // select->uniformSelectionWithoutReplacement(pool, fPool, eConf);
    tSelect(pool, sPool, eConf);
    // TODO: introduce modified parameter to reduce useless recalculation of gens
    // TODO: put the statistic calculation of the fitness outside, all values are already exposed due to the gens

    // Crossover
    (*cross)(sPool, pool, eConf);

    for (auto it = pool.begin(); it != pool.end(); ++it) {
      bool mutated = mutate->randomReplaceGen(*it, eConf);
      if(not mutated){
	mutated |= mutate->addRandomAngleOffset(*it, eConf);
	mutated |= mutate->addOrthogonalAngleOffset(*it, eConf);
	mutated |= mutate->randomScaleDistance(*it, eConf);
      }
      // if(mutated){
      calFitness->estimateGen(*it, *rob, eConf);

      // }
    }
    pool.push_back(eConf.best);
    // Increase Iteration
    eConf.currentIter++;
  }
  // Log Fitnessvalues for all iterations
  logging::Logger(eConf.logStr->str(), eConf.logDir, eConf.logName, true);
}


///////////////////////////////////////////////////////////////////////////////
//                      Roulettewheel Selection Scenario                     //
///////////////////////////////////////////////////////////////////////////////


void op::Optimizer::optimizePath_s_roulette_c_dp(bool display){

  RouletteWheelSelection rSelect;

  if(!eConf.restore){
    (*init)(pool, eConf);
  } else {
    pool.clear();
    restorePopulationFromSnapshot(eConf.snapshot);
  }
  // Main loop
  (*calFitness)(pool, *rob, eConf);

  while(eConf.currentIter <= eConf.maxIterations){
    getBestGen(pool, eConf);
    trackPoolFitness(pool, eConf);
    float zeroPercent = calZeroActionPercent(pool);

    clearZeroPAs(pool, eConf);
    logAndSnapshotPool(eConf, zeroPercent);
    printRunInformation(eConf, zeroPercent, display);
    // Selection
    rSelect(pool, sPool, eConf);
    // Crossover
    (*cross)(sPool, pool, eConf);

    for (auto it = pool.begin(); it != pool.end(); ++it) {
      bool mutated = mutate->randomReplaceGen(*it, eConf);
      if(not mutated){
	mutated |= mutate->addRandomAngleOffset(*it, eConf);
	mutated |= mutate->addOrthogonalAngleOffset(*it, eConf);
	mutated |= mutate->randomScaleDistance(*it, eConf);
      }
      // if(mutated){
      calFitness->estimateGen(*it, *rob, eConf);
      // }
    }
    pool.push_back(eConf.best);

    // Increase Iteration
    eConf.currentIter++;
  }
  // Log Fitnessvalues for all iterations
  logging::Logger(eConf.logStr->str(), eConf.logDir, eConf.logName, true);
}
