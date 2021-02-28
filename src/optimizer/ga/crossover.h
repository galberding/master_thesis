#ifndef CROSSOVER_H
#define CROSSOVER_H

#include "../../tools/configuration.h"
#include "../../tools/debug.h"
#include "init.h"

namespace cross {
  using namespace conf;
  using namespace init;

  /////////////////////////////////////////////////////////////////////////////
  //                            CrossoverStrategy                             //
  /////////////////////////////////////////////////////////////////////////////
  struct CrossoverStrategy {
    // Conditions:
    // - Selection pools needs to be filled
    // - Next Pool is expected to be already emptied
    virtual void operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf) = 0;
    virtual void operator()(FamilyPool& fPool, Genpool& pool , executionConfig& eConf) = 0;
    void copyActions(PAs::iterator begin, PAs::iterator end, PAs &child, bool modify=false);
  };

  struct DualPointCrossover : CrossoverStrategy {
    virtual void operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf) override;
    virtual void operator()(FamilyPool& fPool, Genpool& pool , executionConfig& eConf) override;
    template <typename T>
    bool mating(genome &par1, genome &par2, T& newPopulation, executionConfig& eConf);
    genome getChild(PAs par1, PAs par2, int sIdx[2], int len[2], bool move);
  };


}

#endif /* CROSSOVER_H */
