#ifndef __OPTI_ADAPTER__
#define __OPTI_ADAPTER__


#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
// #include <opencv2/core/eigen.hpp>


// #include "ga_path_generator.h"
#include "../tools/pa_serializer.h"
#include "ga/init.h"
#include "ga/selection.h"
#include "ga/crossover.h"
#include "ga/mutation.h"
#include "ga/fitness.h"

#define MIN_CROSS_LEN 4
#define SEL_TIMEOUT 5



namespace op {
  using namespace init;
  using namespace sel;
  using namespace cross;
  using namespace mut;
  using namespace fit;
  using namespace conf;


  void adaptCrossover(executionConfig& eConf);
  void clearZeroPAs(Genpool& pool, executionConfig& eConf);

  void getBestGen(Genpool& pool, executionConfig& eConf);


  /////////////////////////////////////////////////////////////////////////////
  //                                Optimizer                                 //
  /////////////////////////////////////////////////////////////////////////////


  struct Optimizer {
    executionConfig eConf;
    shared_ptr<FitnessStrategy> fitnessStrat;
    shared_ptr<MutationStrategy> mutate;
    shared_ptr<CrossoverStrategy> cross;
    shared_ptr<SelectionStrategy> select;
    shared_ptr<InitStrategy> init;
    Genpool pool, sel, elite;
    SelectionPool sPool;
    FamilyPool fPool;
    shared_ptr<Robot> rob;

    Optimizer(
	      shared_ptr<InitStrategy> init,
	      shared_ptr<SelectionStrategy> select,
	      shared_ptr<CrossoverStrategy> cross,
	      shared_ptr<MutationStrategy> mutate,
	      shared_ptr<FitnessStrategy> calFitness,
	      executionConfig eConf
	      ):init(init),
		select(select),
		cross(cross),
		mutate(mutate),
		fitnessStrat(calFitness),
		eConf(eConf){

      if(eConf.fitSselect == 1){
	// warn("Resolutions does not match!");
	if(eConf.mapResolution > eConf.Rob_width){
	  warn("Resolutions does not match!");
	  exit(-1);
	}
	rob = make_shared<PolyRobot>(PolyRobot(eConf.rob_conf,
				     eConf.gmap,
				     eConf.obstacleName));}
      else{
	if(eConf.mapResolution != eConf.Rob_width){
	  warn("Resolutions does not match!");
	  exit(-1);
	}
	rob = make_shared<Robot>(Robot(eConf.rob_conf,
				     eConf.gmap,
				     eConf.obstacleName));}


    }
    /**
       Continously optimize the path.
     If the current population is initialized -> size > 0,
     the initialization step will be omitted.
      */
    // private:
    //   Optimizer()
    void printRunInformation(executionConfig& eConf, bool display);
    void optimizePath(bool display = false);
    void optimizePath_Turn_RWS(bool display = false);
    void logAndSnapshotPool(executionConfig& eConf);
    void restorePopulationFromSnapshot(const string path);
    void snapshotPopulation(const string path);
    void snapshotPopulation(executionConfig& eConf);
    void saveBest(Genpool& pool, executionConfig& eConf, bool sortPool=true);
    void replaceWithBest(Genpool& pool, executionConfig& eConf);
    void insertBest(Genpool& pool, executionConfig& eConf);
    void balancePopulation(Genpool& pool, executionConfig& eConf);
    bool checkEndCondition();
  };
}


#endif //__OPTI_ADAPTER__
