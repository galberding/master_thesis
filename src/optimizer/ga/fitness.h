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
    void estimateChildren(FamilyPool& fPool, path::Robot &rob, executionConfig& eConf);

    virtual void estimateGen(genome &gen, path::Robot &rob, executionConfig& eConf);
    virtual float calculation(genome& gen, int freeSpace, executionConfig &eConf);
    // virtual float calculation(float cdist,
    // 			      float dist,
    // 			      int crossed,
    // 			      float cSpeed_m_s,
    // 			      float speed_m_s,
    // 			      int freeSpace,
    // 			      executionConfig& eConf);
  };

}

#endif /* FITNESS_H */
