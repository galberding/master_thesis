#ifndef MUTATION_H
#define MUTATION_H

#include "../../tools/configuration.h"
#include "../../tools/debug.h"
#include "init.h"

namespace mut {
  using namespace conf;
  using namespace init;
    /////////////////////////////////////////////////////////////////////////////
  //                             MutationStrategy                            //
  /////////////////////////////////////////////////////////////////////////////
  struct MutationStrategy {
    virtual void operator()(Genpool& currentPool, executionConfig& eConf);
    virtual void operator()(FamilyPool& fPool, executionConfig& eConf);
    void mutateGen(genome &gen, executionConfig &eConf);
    bool addOrthogonalAngleOffset(genome& gen, executionConfig& eConf);
    bool addRandomAngleOffset(genome& gen, executionConfig& eConf);
    void addPositiveDistanceOffset(genome& gen, executionConfig& eConf);
    void addNegativeDistanceOffset(genome& gen, executionConfig& eConf);
    bool randomScaleDistance(genome& gen, executionConfig& eConf);
    bool randomReplaceGen(genome& gen, executionConfig& eConf);

  };


}

#endif /* MUTATION_H */
