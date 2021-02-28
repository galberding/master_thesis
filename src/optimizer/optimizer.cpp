#include "optimizer.h"

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


void op::getBestGen(Genpool& pool, executionConfig& eConf){
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

void op::adaptCrossover(executionConfig& eConf){
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

void op::clearZeroPAs(Genpool& pool, executionConfig& eConf){
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
