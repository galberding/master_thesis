#include "fitness.h"


///////////////////////////////////////////////////////////////////////////////
//                              Fitness strategy                         //
///////////////////////////////////////////////////////////////////////////////

void fit::resetLoggingFitnessParameter(executionConfig& eConf){
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



void fit::trackFitnessParameter(genome& gen, executionConfig& eConf){
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


void fit::finalizeFitnessLogging(int poolsize, executionConfig& eConf){
  assert(poolsize > 0);
  eConf.fitnessAvg /= poolsize;
  eConf.fitnessAvgTime /= poolsize;

  eConf.fitnessAvgCoverage /= poolsize;
  eConf.fitnessAvgAngleCost /= poolsize;
  eConf.actionLenAvg /= poolsize;
}


void fit::trackPoolFitness(Genpool& pool, executionConfig& eConf){
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



void fit::FitnessStrategy::operator()(Genpool &currentPool, path::Robot &rob, executionConfig& eConf){
  resetLoggingFitnessParameter(eConf);
  // debug("Before cal");
  assert(currentPool.size() > 0);
  for(Genpool::iterator it = currentPool.begin(); it != currentPool.end(); ++it){
    estimateGen(*it, rob, eConf);
  }
  applyPoolBias(currentPool, eConf);
  finalizeFitnessLogging(currentPool.size(), eConf);
}


void fit::FitnessStrategy::operator()(FamilyPool& fPool, path::Robot &rob, executionConfig& eConf) {
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
      applyPoolBias(family, eConf, true);
    }
  }
}


void fit::FitnessStrategy::estimateGen(genome &gen, path::Robot &rob, executionConfig& eConf){
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

float fit::FitnessStrategy::calculation(genome& gen, int freeSpace, executionConfig &eConf){
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
  // TODO: Actual time factor
  float actualTime = gen.traveledDist;

  // debug(log(10 + gen.cross));
  float optimalTime = gen.traveledDist - (gen.cross);
  // debug("Optimal Time: ", optimalTime);
  float finalTime = optimalTime / actualTime;

  float currentCoverage = (gen.traveledDist) - gen.cross;
  float totalCoverage = freeSpace;
  float finalCoverage = currentCoverage / totalCoverage;

  // finalTime *= finalCoverage;

  float weight = eConf.fitnessWeights[2];
  gen.finalCoverage = finalCoverage;
  gen.finalTime = finalTime;
  float x = finalTime;
  float y = finalCoverage;
  gen.finalAngleCost =  gen.rotationCost;


  if(eConf.funSelect == 0)
    gen.fitness = (pow(x, 2)*pow(y, 2));
  else if(eConf.funSelect == 1)
    gen.fitness = (pow(x, 5)*pow(y, 4));
  else if(eConf.funSelect == 2)
    gen.fitness = (0.5*x + 0.5*y)*(pow(x, 3)*pow(y, 3));
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
  // if(eConf.penalizeRotation and gen.rotationCost > 0){
  //   // assert(gen.rotationCost > 0);
  //   gen.fitness = 0.8*gen.fitness + 0.2*(1 - gen.finalAngleCost);
  // }
  // Genpool pool;
  // applyPoolBias(pool);
  return gen.fitness;
}


///////////////////////////////////////////////////////////////////////////////
//                           Fittness Rotation Bias                          //
///////////////////////////////////////////////////////////////////////////////


void fit::FitnessRotationBias::applyPoolBias(Genpool &pool, executionConfig &eConf, bool useGlobal){
  float rotMin = 1;
  if(useGlobal){
    rotMin = eConf.fitnessMinAngleCost;
  }else{
    for (auto it = pool.begin(); it != pool.end(); ++it) {
      if(it->rotationCost < rotMin)
	rotMin = it->rotationCost;
    }
  }

  for (auto it = pool.begin(); it != pool.end(); ++it) {
    float RotBias = rotMin / it->rotationCost;
    it->fitness *= RotBias;
  }
}
