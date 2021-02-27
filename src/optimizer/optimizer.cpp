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
  // PAs actions;
  gen.actions.clear();
  uniform_int_distribution<> angleDist(0,360);
  uniform_real_distribution<float> distanceDist(0,50);
  gen.actions.push_back(make_shared<StartAction>(StartAction(eConf.start)));
    for(int i=0; i<len; i++){
      PA_config config{{PAP::Angle,angleDist(eConf.generator)}, {PAP::Distance, distanceDist(eConf.generator)}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
    }
    gen.actions.push_back(make_shared<EndAction>(EndAction(eConf.ends)));
    // gen.actions = actions;
}

void op::InitStrategy::replaceZeroGensWithRandom(Genpool& pool){
  debug("Replace phase: ");


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
  eConf.best = currentPool.back();
  keep.insert(keep.begin(),
	      prev(currentPool.end(), eConf.selectKeepBest),
	      currentPool.end());

  // Start selection process
  // TODO: Remember select individuals is now the value of the selected pairs!
  // int timeout = 0;
  while(selPool.size() < eConf.selectIndividuals){
    // Draw individuals
    auto couple = make_pair(selection(currentPool, eConf),
	      selection(currentPool, eConf));
    // Check if fitness is high enough
    // 0 fitness indicates a defect gen
    // if((couple.first.fitness == 0
    //    || couple.second.fitness == 0) and timeout < SEL_TIMEOUT){
    //   warn("Selection: skip gen with fitness 0");
    //   timeout++;
    //   // continue;
    // }
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

// TODO: Move to own selection procedure class
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


genome op::RWS::selection(Genpool &currentPopulation, executionConfig &eConf){
  // Calculate the total fitness value

  float totalFitness = eConf.fitnessAvg * currentPopulation.size();
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

genome op::RankedRWS::selection(Genpool &currentPopulation, executionConfig &eConf){
  // debug("Ranked!");
  float SP = eConf.selPressure;
  assert(SP >= 1);
  assert(SP <= 2);
  // genome gen;
  float totalSum = 0;
  int idx = 0;
  std::uniform_real_distribution<float> rouletteWheel(2-SP, currentPopulation.size());
  float wheelValue = rouletteWheel(eConf.generator);
  for(int i=0; i<currentPopulation.size(); i++){
    float rank = 2.0-SP+2.0*(SP-1.0)*((i-1.0) / (currentPopulation.size()-1.0));
    // debug("Rank: ", rank);
    totalSum += rank;
    // debug(totalSum, "--", wheelValue);
    if(wheelValue <= totalSum){
      idx = i;
      break;
    }
  }
  return currentPopulation.at(idx);
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
  if(pool.size() > 0)
    sort(pool.begin(), pool.end());

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


  uniform_int_distribution<int> dist1(1,par1.actions.size() - (len1+1));
  uniform_int_distribution<int> dist2(1,par2.actions.size() - (len2+1));
  // calculate the start Index
  int sIdx[2];
  int sIdx1 = sIdx[0] = dist1(eConf.generator);
  int sIdx2= sIdx[1] = dist2(eConf.generator);

  // // Choose the same start index
  // if (par1.actions.size() >= par2.actions.size())
  //   sIdx1 = sIdx[0] = sIdx[1];
  // else
  //   sIdx2 = sIdx[1] = sIdx[0];

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

  // Apply mutation if a apply action is true or if a gen is dead
  if(not(applyAction(eConf.mutaReplaceGen, eConf) or gen.actions.size() < eConf.getMinGenLen())) return false;
  InitStrategy init;

  // Check if average length if bigger than minimal allowed length
  // if(eConf.actionLenAvg < eConf.getMinGenLen())
  init(gen, eConf.getMinGenLen() * 2, eConf);
  // else
    // init(gen, eConf.actionLenAvg, eConf);
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
  eConf.fitnessMaxTime = 0;
  eConf.fitnessMinTime = 1;

  eConf.fitnessAvgCoverage = 0;
  eConf.fitnessMaxCoverage = 0;
  eConf.fitnessMinCoverage = 1;
  eConf.fitnessAvgAngleCost = 0;
  eConf.fitnessMaxAngleCost = 0;
  eConf.fitnessMinAngleCost = 1;
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
  if(gen.finalTime > eConf.fitnessMaxTime)
    eConf.fitnessMaxTime = gen.finalTime;
  if(gen.finalTime < eConf.fitnessMaxTime)
    eConf.fitnessMaxTime = gen.finalTime;
  if(gen.finalCoverage > eConf.fitnessMaxCoverage)
    eConf.fitnessMaxCoverage = gen.finalCoverage;
  if(gen.finalCoverage < eConf.fitnessMinCoverage)
    eConf.fitnessMinCoverage = gen.finalCoverage;
  if(gen.finalAngleCost > eConf.fitnessMaxAngleCost)
    eConf.fitnessMaxAngleCost = gen.finalAngleCost;
  if(gen.finalAngleCost < eConf.fitnessMinAngleCost)
    eConf.fitnessMinAngleCost = gen.finalAngleCost;


  eConf.fitnessAvg += fitness;
  eConf.actionLenAvg += gen.actions.size();
  eConf.fitnessAvgTime += gen.finalTime;
  eConf.fitnessAvgCoverage += gen.finalCoverage;
  eConf.fitnessAvgAngleCost += gen.finalAngleCost;
}


void finalizeFitnessLogging(int poolsize, executionConfig& eConf){
  assert(poolsize > 0);
  eConf.fitnessAvg /= poolsize;
  eConf.fitnessAvgTime /= poolsize;

  eConf.fitnessAvgCoverage /= poolsize;
  eConf.fitnessAvgAngleCost /= poolsize;
  eConf.actionLenAvg /= poolsize;
}


void trackPoolFitness(Genpool& pool, executionConfig& eConf){
  resetLoggingFitnessParameter(eConf);
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    trackFitnessParameter(*it, eConf);
  }

  finalizeFitnessLogging(pool.size(), eConf);
  if(eConf.crossAdapter == 0)
    eConf.lastDmax = 0;
  eConf.adaptCrossover();
  eConf.adaptMutation();
  eConf.adaptCLen();

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
    gen.fitness = 0;
    return 0;
  }
  // Set calculated path
  gen.setPathSignature(eConf.gmap);

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
  gen.finalAngleCost =  gen.rotationCost / (gen.actions.size() - 1);
  if(eConf.funSelect == 0)
    gen.fitness = (pow(x, 2)*pow(y, 2));
  else if(eConf.funSelect == 1)
    gen.fitness = (pow(x, 5)*pow(y, 4));
  else if(eConf.funSelect == 2)
    gen.fitness = (0.5*x + 0.5*y)*(pow(x, 3)*pow(y, 2));
  else if(eConf.funSelect == 3)
    gen.fitness = (0.25*(0.5*x + 0.5*y) + 0.25*x*y + 0.25*(pow(x, 2) * pow(y, 2)) + 0.25*(0.5*pow(x, 2) + 0.5*pow(y, 2)))*(pow(x, 5)*pow(y, 4));
  else if(eConf.funSelect == 4)
    gen.fitness = (0.5*x + 0.5*y)*(pow(x, 4)*pow(y, 4));
  else if(eConf.funSelect == 5)
    gen.fitness = (0.45*x + 0.45*y + 0.1*(1-gen.finalAngleCost))*(pow(x, 4)*pow(y, 4));
  // gen.fitness = (pow(x, 5)*pow(y, 4));
  // gen.fitness = (0.5*x + 0.5*y)*(pow(x, 3)*pow(y, 2));
  // gen.fitness = 0.5*(0.5*x + 0.5*y) + 0.5*(x*y); // 370~600 -> turning point
  // Panelty for zero actions
  if(eConf.penalizeZeroActions)
    gen.fitness *= 1 - calZeroActionPercent(gen);

  // debug("Costs: ", gen.rotationCost);
  if(eConf.penalizeRotation and gen.rotationCost > 0){
    // assert(gen.rotationCost > 0);
    gen.fitness = 0.8*gen.fitness + 0.2*(1 - gen.finalAngleCost);
  }
  return gen.fitness;
}


// void resizePool(Genpool &pool, executionConfig &eConf){
//   if(eConf)
// }


/////////////////////////////////////////////////////////////////////////////
//                                Optimizer                                 //
/////////////////////////////////////////////////////////////////////////////

void op::Optimizer::logAndSnapshotPool(executionConfig& eConf){
  getDivMeanStd(pool, eConf.diversityMean, eConf.diversityStd);
  // Write initial logfile
  if(eConf.currentIter == 0){
      *eConf.logStr << "Iteration,FitAvg,FitMax,FitMin,TimeAvg,TimeMax,TimeMin,CovAvg,CovMax,CovMin,AngleAvg,AngleMax,AngleMin,AcLenAvg,AcLenMax,AcLenMin,ZeroAcPercent,DGens,BestTime,BestCov,BestAngle,BestLen,DivMean,DivStd\n";
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
				    eConf.fitnessMaxTime,
				    eConf.fitnessMinTime,
				    // eConf.fitnessAvgOcc,
				    eConf.fitnessAvgCoverage,
				    eConf.fitnessMaxCoverage,
				    eConf.fitnessMinCoverage,
				    eConf.fitnessAvgAngleCost,
				    eConf.fitnessMaxAngleCost,
				    eConf.fitnessMinAngleCost,
				    eConf.actionLenAvg,
				    eConf.actionLenMax,
				    eConf.actionLenMin,
				    eConf.zeroActionPercent,
				    eConf.deadGensCount,
				    eConf.best.finalTime,
				    eConf.best.finalCoverage,
				    eConf.best.finalAngleCost,
				    eConf.best.actions.size(),
				    eConf.diversityMean,
				    eConf.diversityStd
				    );
    }
    if(eConf.takeSnapshot && (eConf.currentIter % eConf.takeSnapshotEvery == 0)){
      // debug("Take snapshot to: ", eConf.tSnap);
      snapshotPopulation(eConf);

      logging::Logger(eConf.logStr->str(), eConf.logDir, eConf.logName, true);
      eConf.logStr->str("");
    }
}

void op::Optimizer::printRunInformation(executionConfig& eConf, bool display){
  if(eConf.best.id > 0 && display){
      debug(eConf.currentIter, ", MaxFitness: ",
	    eConf.best.fitness, " (", eConf.best.finalTime, ", ", eConf.best.finalCoverage,",  ", eConf.best.finalAngleCost,", ",eConf.best.actions.size(), ") : ",
	    argsToCsv(eConf.fitnessAvgTime,
		      // eConf.fitnessAvgOcc,
		      eConf.fitnessAvgCoverage,
		      eConf.actionLenAvg,
		      // eConf.zeroActionPercent,
		      // eConf.deadGensCount,
		      eConf.crossoverProba,
		      eConf.mutaReplaceGen,
		      eConf.crossLength,
		      " | ",
		      eConf.diversityMean,
		      eConf.diversityStd));
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
    while(eConf.currentIter <= eConf.maxIterations){

      // Logging
      getBestGen(pool, eConf);
      trackPoolFitness(pool, eConf);
      eConf.deadGensCount = countDeadGens(pool, eConf.getMinGenLen());
      eConf.zeroActionPercent = calZeroActionPercent(pool);
      clearZeroPAs(pool, eConf);
      logAndSnapshotPool(eConf);
      printRunInformation(eConf, display);
      // if(pool.size() - eConf.deadGensCount == 0){
      // 	logging::Logger(eConf.logStr->str(), eConf.logDir, eConf.logName, true);
      // 	assertm(false, "Population died!");
      // }

      // Selection
      select->uniformSelectionWithoutReplacement(pool, fPool, eConf);

      // Crossover
      (*cross)(fPool, pool, eConf);
      // Mutate remaining individuals in pool
      if (pool.size() > 2){

	for (auto it = pool.begin(); it != next(pool.begin(), pool.size() - 1); ++it) {
	  bool mutated = mutate->randomReplaceGen(*it, eConf);
	  // if(not mutated){
	  //   mutated |= mutate->addRandomAngleOffset(*it, eConf);
	  //   mutated |= mutate->addOrthogonalAngleOffset(*it, eConf);
	  //   mutated |= mutate->randomScaleDistance(*it, eConf);
	  // }
	  if(mutated){
	    calFitness->estimateGen(*it, *rob, eConf);
	    it->trail = 1 * (*eConf.gmap)["map"];
	  }
	}
      }
      // Mutation
      (*mutate)(fPool, eConf);
      calFitness->estimateChildren(fPool, *rob, eConf);
      select->elitistSelection(fPool, pool);
      // Second mutation stage:
      sort(pool.begin(), pool.end());


      // Increase Iteration
      eConf.currentIter++;
  }
  // Log Fitnessvalues for all iterations
  logging::Logger(eConf.logStr->str(), eConf.logDir, eConf.logName, true);
}


///////////////////////////////////////////////////////////////////////////////
//                       Tournament Selection Scenario                       //
///////////////////////////////////////////////////////////////////////////////


void op::Optimizer::optimizePath_Turn_RWS(bool display){

  SelectionStrategy *selection;
  TournamentSelection Tselection;
  RWS Rselection;
  RankedRWS RRselection;

  if(eConf.scenario == 1)
    selection = &Tselection;
  else if(eConf.scenario == 2)
    selection = &Rselection;
  else
    selection = &RRselection;

  Genpool mPool;

  if(!eConf.restore){
    (*init)(pool, eConf);
  } else {
    pool.clear();
    restorePopulationFromSnapshot(eConf.snapshot);
  }
  // Main loop
  (*calFitness)(pool, *rob, eConf);

  while(eConf.currentIter <= eConf.maxIterations){


    // Logging
    getBestGen(pool, eConf);
    eConf.deadGensCount = countDeadGens(pool, eConf.getMinGenLen());
    eConf.zeroActionPercent = calZeroActionPercent(pool);
    clearZeroPAs(pool, eConf);
    trackPoolFitness(pool, eConf);
    logAndSnapshotPool(eConf);
    printRunInformation(eConf, display);


    // Selection
    (*selection)(pool, sPool, eConf);

    // Crossover
    mPool.clear();
    (*cross)(sPool, mPool, eConf);

    // Mutation
    for (auto it = mPool.begin(); it != mPool.end(); ++it) {
      bool mutated = mutate->randomReplaceGen(*it, eConf);
      if(not mutated){
	mutated |= mutate->addRandomAngleOffset(*it, eConf);
	mutated |= mutate->addOrthogonalAngleOffset(*it, eConf);
	mutated |= mutate->randomScaleDistance(*it, eConf);
      }
      calFitness->estimateGen(*it, *rob, eConf);
    }

    pool.insert(pool.end(), mPool.begin(), mPool.end());
    // Keep best individual
    // pool.push_back(eConf.best);

    // Increase Iteration
    eConf.currentIter++;
  }
  // Log Fitnessvalues for all iterations
  logging::Logger(eConf.logStr->str(), eConf.logDir, eConf.logName, true);
}
