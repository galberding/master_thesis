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
  eConf.fitnessMinAngleCost = eConf.fitnessMaxAngleCost;
  eConf.fitnessMaxAngleCost = 0;
  eConf.actionLenMin = eConf.actionLenMax;
  eConf.actionLenMax = 0;
  eConf.actionLenAvg = 0;
  eConf.fitnessMinObjCount = eConf.fitnessMaxObjCount;
  eConf.fitnessMaxObjCount = 0;
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
  if(gen.finalTime < eConf.fitnessMinTime)
    eConf.fitnessMinTime = gen.finalTime;
  if(gen.finalCoverage > eConf.fitnessMaxCoverage)
    eConf.fitnessMaxCoverage = gen.finalCoverage;
  if(gen.finalCoverage < eConf.fitnessMinCoverage)
    eConf.fitnessMinCoverage = gen.finalCoverage;

  if(gen.finalRotationTime > eConf.fitnessMaxAngleCost)
    eConf.fitnessMaxAngleCost = gen.finalRotationTime;
  if(gen.finalRotationTime < eConf.fitnessMinAngleCost)
    eConf.fitnessMinAngleCost = gen.finalRotationTime;

  if(gen.p_obj > eConf.fitnessMaxObjCount)
    eConf.fitnessMaxObjCount = gen.p_obj;
  if(gen.p_obj < eConf.fitnessMinObjCount)
    eConf.fitnessMinObjCount = gen.p_obj;



  eConf.fitnessAvg += fitness;
  eConf.actionLenAvg += gen.actions.size();
  eConf.fitnessAvgTime += gen.finalTime;
  eConf.fitnessAvgCoverage += gen.finalCoverage;
  eConf.fitnessAvgAngleCost += gen.finalRotationTime;
  eConf.fitnessAvgObjCount += gen.p_obj;
}


void fit::finalizeFitnessLogging(int poolsize, executionConfig& eConf){
  assert(poolsize > 0);
  eConf.fitnessAvg /= poolsize;
  eConf.fitnessAvgTime /= poolsize;

  eConf.fitnessAvgCoverage /= poolsize;
  eConf.fitnessAvgAngleCost /= poolsize;
  eConf.actionLenAvg /= poolsize;
  eConf.fitnessAvgObjCount /= poolsize;
}


void fit::trackPoolFitness(Genpool& pool, executionConfig& eConf){
  resetLoggingFitnessParameter(eConf);
  eConf.popSize = pool.size();
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    trackFitnessParameter(*it, eConf);
  }

  finalizeFitnessLogging(pool.size(), eConf);
  if(eConf.crossAdapter == 0)
    eConf.lastDmax = 0;
  eConf.adaptMutation();
  eConf.adaptCrossover();
  eConf.adaptCLen();
  eConf.adaptSelPressure();

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
        bool eva = rob.evaluateActions(family[i].actions);
	assert(eva);
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
    debug("Dead Gen detected!");
    gen.fitness = 0;
    return 0;
  }
  // Set calculated path --> Diversity estimation
  gen.setPathSignature(eConf.gmap);

  // Time parameter:

  float actualTime = (gen.traveledDist)* pow(eConf.mapResolution, 2) / eConf.Rob_speed;
  // float actualTime = (gen.p_obj + gen.traveledDist)* pow(eConf.mapResolution, 2) / eConf.Rob_speed;
  float optimalTime = (gen.traveledDist - (gen.cross + gen.covered)) * pow(eConf.mapResolution, 2) / eConf.Rob_speed;
  if(optimalTime < 0)
    optimalTime = 0;
  float finalTime = 0;
  if (eConf.penalizeRotation)
    finalTime = optimalTime / (actualTime + gen.rotations / eConf.Rob_angleSpeed);
  else
    finalTime = optimalTime / actualTime;

  // Coverage
  // gen.cross += gen.covered;
  float currentCoverage = (gen.traveledDist - (gen.cross + gen.covered)) * pow(eConf.mapResolution, 2);
  if(currentCoverage < 0)
    currentCoverage = 0;
  float totalCoverage = (freeSpace - gen.covered) * pow(eConf.mapResolution, 2);
  // float totalCoverage = (freeSpace + gen.p_obj) * pow(eConf.mapResolution, 2);
  float finalCoverage = currentCoverage / totalCoverage;



  gen.finalCoverage = finalCoverage;
  gen.finalTime = finalTime;
  float x = finalTime;
  float y = finalCoverage;
  gen.finalRotationTime =  gen.rotations;

  fitnessFun(gen, x, y, eConf);
  // Panelty for zero actions
  int zeros = countZeroActions(gen, eConf.mapResolution);
  // debug(zeros);
  if(eConf.penalizeZeroActions and zeros > 0)
    gen.fitness *= 1.0/(zeros * 2);

  if (gen.p_obj > 0)
    gen.fitness *= 1.0/(gen.p_obj * 2);

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


float fit::FitnessSemiContinuous::calculation(genome &gen, int freeSpace, executionConfig &eConf){
  // prepare parameters
  // Check if the gen is valid -> returns false if gen has distance 0
  if(!gen.updateGenParameter()){
    debug("Dead Gen detected!");
    gen.fitness = 0;
    return 0;
  }
  // Set calculated path
  gen.setPathSignature(eConf.gmap);

  float cross_p = gen.cross / gen.traveledDist;

  // Convert pixel values to [m²]
  float area = freeSpace * pow(eConf.Rob_width, 2);

  float cov = (gen.pathLengh * eConf.Rob_width) ;
  float cross_d = cov * cross_p;
  float finalCoverage = (cov - cross_d) / area;

  // Time parameter:
  float actualTime = gen.pathLengh * eConf.Rob_speed;
  // Penalize if enpoint is not reached

  float optimalTime = actualTime - (gen.pathLengh * cross_p) * eConf.Rob_speed;

  if (not gen.reachEnd)
    actualTime *= 2;

  // debug("Optimal Time: ", optimalTime);
  float rotation_time = gen.rotations / eConf.Rob_angleSpeed;
  float finalTime = optimalTime / (actualTime + rotation_time);


  // ###
  gen.finalCoverage = finalCoverage;
  gen.finalTime = finalTime;
  gen.finalRotationTime =  rotation_time;

  float x = finalTime;
  float y = finalCoverage;

  fitnessFun(gen, x, y, eConf);
  // debug(" cross_d: ",cross_d, " area: ",area," cov: ",cov," finalCoverage: ",finalCoverage," actualTime: ",actualTime," optimalTime: ",optimalTime," rotation_time: ",rotation_time, " genRot: ", gen.rotations, "finalTime: ",finalTime, " Fit: ", gen.fitness);
  if(eConf.penalizeZeroActions)
    gen.fitness *= 1.0 - calZeroActionPercent(gen, eConf.mapResolution);

  return gen.fitness;
}


// float fit::FitnessPoly::calculation(genome &gen, int freeSpace, executionConfig &eConf){
//    // prepare parameters
//   // Check if the gen is valid -> returns false if gen has distance 0
//   if(!gen.updateGenParameter()){
//     debug("Dead Gen detected!");
//     gen.fitness = 0;
//     return 0;
//   }
//   // Set calculated path
//   gen.setPathSignature(eConf.gmap);
//   // TODO: Time calculation behaves weired!
//   // Time parameter:
//   float pixelContrib = (gen.traveledDist - (gen.cross) - gen.p_obj)*pow(eConf.mapResolution, 2);
//   debug("Obj: ", gen.p_obj, " Len: ", gen.pathLengh);

//   float actualTime = gen.pathLengh / eConf.Rob_speed;
//   float optimalTime = pixelContrib / eConf.Rob_speed;

//   float finalTime = optimalTime / actualTime;

//   // Coverage
//   float currentCoverage = pixelContrib;
//   float totalCoverage = freeSpace*pow(eConf.mapResolution, 2);
//   float finalCoverage = currentCoverage / totalCoverage;

//   gen.finalCoverage = finalCoverage;
//   gen.finalTime = finalTime;
//   float x = finalTime;
//   float y = finalCoverage;
//   gen.finalRotationTime =  gen.rotations;

//   fitnessFun(gen, x, y, eConf);
//   // Panelty for zero actions
//   if(eConf.penalizeZeroActions)
//     gen.fitness *= 1 - calZeroActionPercent(gen);



//   return gen.fitness;

// }

///////////////////////////////////////////////////////////////////////////////
//                             Fitness Functions                             //
///////////////////////////////////////////////////////////////////////////////

void fit::fitnessFun(genome& gen, float x, float y, executionConfig& eConf){
  if(eConf.funSelect == 0) 	// Linear case
    gen.fitness = (0.5*x + 0.5*y);
  else if(eConf.funSelect == 1) // Multiply
    gen.fitness = x*y;
  else if(eConf.funSelect == 2) // Weighted
    gen.fitness = (0.5*x + 0.5*y)*x*y;
  else if(eConf.funSelect == 3)	// Sqrt
    gen.fitness = sqrt((0.5*x + 0.5*y)*x*y);
  else
    gen.fitness = pow(x,4)*pow(y,4);
    // assert(false);
  // else if(eConf.funSelect == 4)
  //   gen.fitness = (0.5*x + 0.5*y)*(pow(x, 4)*pow(y, 4));
  // else if(eConf.funSelect == 5)
  //   gen.fitness = (0.45*x + 0.45*y + 0.1*(1-gen.finalRotationTime))*(pow(x, 4)*pow(y, 4));
}
