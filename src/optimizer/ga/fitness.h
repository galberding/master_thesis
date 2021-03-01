#ifndef FITNESS_H
#define FITNESS_H

#include "../../tools/configuration.h"
#include "../../tools/debug.h"

namespace fit {
  using namespace conf;

  /////////////////////////////////////////////////////////////////////////////
  //                             FitnessStrategy                             //
  /////////////////////////////////////////////////////////////////////////////
  void resetLoggingFitnessParameter(executionConfig& eConf);
  void trackFitnessParameter(genome& gen, executionConfig& eConf);
  void finalizeFitnessLogging(int poolsize, executionConfig& eConf);
  void trackPoolFitness(Genpool& pool, executionConfig& eConf);
  struct FitnessStrategy {

    virtual void operator()(Genpool &currentPool, path::Robot &rob, executionConfig& eConf);
    virtual void operator()(FamilyPool& fPool, path::Robot &rob, executionConfig& eConf);

    virtual void estimateGen(genome &gen, path::Robot &rob, executionConfig& eConf);
    virtual float calculation(genome& gen, int freeSpace, executionConfig &eConf);
    virtual void applyPoolBias(Genpool& pool, executionConfig &eConf, bool useGlobal=false){debug("Blub");return;};
  };

  struct FitnessRotationBias : FitnessStrategy {
    /**
     * @brief      Calculate min rotation and use it alter the fitness in the pool.
     *
     * @details    First the min rotation cost in the pool is calculated.
     *             After that the bias for each gen is calculated
     *             based on $R_B = R_{min} / R_{i}$. Finally the bias
     *             is multiplied with the current fitness.
     *
     * @param      pool current Population
     *
     */
    virtual void applyPoolBias(Genpool &pool, executionConfig &eConf, bool useGlobal=false) override;
  };

  struct FitnesFollowPoolBest : FitnessStrategy{

  };

}

#endif /* FITNESS_H */
