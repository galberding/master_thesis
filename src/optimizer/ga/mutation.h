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
    /**
     * @brief      Mutate children of a family.
     *
     * @details    It is iterated over each family in the fPool. The first two positions
     *             in the family are reserved for the parents and therefore skipped.
     *             This is done in order to preserve the parent solutions.
     *             It is expected that the positions > 2 are filled with offspring.
     *             To each individual of the offspring the mutations are applied.
     *
     * @param      fPool pool of Genpools, each representing 2 or more individuals
     * @param      eConf configuration
     */
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
