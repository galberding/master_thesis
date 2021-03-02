#include "selection.h"

/////////////////////////////////////////////////////////////////////////////
//                            SelectionStrategy                             //
/////////////////////////////////////////////////////////////////////////////


void sel::SelectionStrategy::operator()(Genpool& currentPool, SelectionPool& selPool, executionConfig& eConf){
  // Genpool keep;
  selPool.clear();
  // Ensure population is not empty
  assert(currentPool.size() > 0);
  assert(eConf.selectKeepBest < currentPool.size());
  // Keep the best individuals in the population for the next generation
  sort(currentPool.begin(), currentPool.end());
  eConf.best = currentPool.back();
  // keep.insert(keep.begin(),
  // 	      prev(currentPool.end(), eConf.selectKeepBest),
  // 	      currentPool.end());

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
  // currentPool.insert(currentPool.begin(),
  // 		     keep.begin(),
  // 		     keep.end());

}

void sel::SelectionStrategy::uniformSelectionWithoutReplacement(Genpool &pool, FamilyPool &fPool, executionConfig &eConf){
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
genome sel::SelectionStrategy::tournamentSelection(Genpool &pool, executionConfig &eConf){

  assert(pool.size() > eConf.tournamentSize);
  deque<genome> turn;
  shuffle(pool.begin(), pool.end(), eConf.generator);
  turn.insert(turn.begin(), pool.begin(), next(pool.begin(), eConf.tournamentSize));
  sort(turn.begin(), turn.end());
  return turn.back();
}


void sel::SelectionStrategy::elitistSelection(FamilyPool& fPool, Genpool& pool){
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

genome sel::SelectionStrategy::selection(Genpool &currentPopulation,
					executionConfig &eConf) {return genome();}


genome sel::RWS::selection(Genpool &currentPopulation, executionConfig &eConf){
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

genome sel::RankedRWS::selection(Genpool &currentPopulation, executionConfig &eConf){
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

genome sel::TournamentSelection::selection(Genpool &currentPopulation, executionConfig &eConf) {
  return tournamentSelection(currentPopulation, eConf);
}
