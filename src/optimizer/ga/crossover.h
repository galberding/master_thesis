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

  /**
   * @brief      Calculate start index based on given size and crossLength.
   *
   * @details    First the smaller of the two sizes passed is used to
   *             calculate the interval for the start index. Finally a
   *             uniform distribution is used to sample the start
   *             Index
   *
   * @param      s1 size of first gen
   * @param      s2 size of second gen
   * @param      eConf configuration containing the crossLength
   *
   * @return     int the start Index
   */
  int getsIdx(int s1, int s2, executionConfig &eConf);

  /**
   * @brief      Get offset length for given sIdx.
   *
   * @details
   *
   * @param      sIdx start index
   * @param      s size of the sequence
   * @param      eConf configuration
   *
   * @return     offset length for sIdx to perform crossover
   */
  int getRemainingLen(int sIdx, int s, executionConfig& eConf);
  struct CrossoverStrategy {
    // Conditions:
    // - Selection pools needs to be filled
    // - Next Pool is expected to be already emptied
    virtual void operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf) = 0;
    virtual void operator()(FamilyPool& fPool, Genpool_shr& pool , executionConfig& eConf) = 0;
    void copyActions(PAs::iterator begin, PAs::iterator end, PAs &child, bool modify=false);
  };

  struct DualPointCrossover : CrossoverStrategy {
    virtual void operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf) override;
    virtual void operator()(FamilyPool& fPool, Genpool_shr& pool , executionConfig& eConf) override;
    virtual bool mating(genome &par1, genome &par2, Genpool& newPopulation, executionConfig& eConf);
    virtual genome getChild(PAs par1, PAs par2, int sIdx[2], int len[2], bool move);
  };

  struct SameStartDualPointCrossover : DualPointCrossover {
    virtual bool mating(genome &par1, genome &par2, Genpool& newPopulation, executionConfig& eConf)override;

  };
}

#endif /* CROSSOVER_H */
