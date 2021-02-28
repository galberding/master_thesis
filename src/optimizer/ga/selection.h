#ifndef SELECTION_H
#define SELECTION_H

#include "../../tools/configuration.h"
#include "../../tools/debug.h"

namespace sel {
  using namespace conf;

  /////////////////////////////////////////////////////////////////////////////
  //                            SelectionStrategy                             //
  /////////////////////////////////////////////////////////////////////////////
  struct SelectionStrategy {
    // Conditions:
    // - current Pool will be cleared here
    // - Selection Pool is supposed to be cleared before it is filled again here
    virtual void operator()(Genpool& currentPool, SelectionPool& selPool, executionConfig& eConf);
    // virtual void operator()(Genpool& currentPool, FamilyPool& selPool, executionConfig& eConf);
    virtual genome selection(Genpool &currentPopulation, executionConfig& eConf);
    // Shuffle pool and generate pairs of two which are contained in a vector and placed in the family pool
    void uniformSelectionWithoutReplacement(Genpool &pool, FamilyPool &fPool, executionConfig &eConf);
    // Select two best of four in the family
    void elitistSelection(FamilyPool& fPool, Genpool& pool);
    genome tournamentSelection(Genpool &pool, executionConfig &eConf);
    // void tournamentSelection(Genpool &pool, FamilyPool &fPool, executionConfig &eConf);
  };

  struct RWS : SelectionStrategy{
    virtual genome selection(Genpool &currentPopulation, executionConfig& eConf) override;
  };

  struct RankedRWS : SelectionStrategy{
    virtual genome selection(Genpool &currentPopulation, executionConfig& eConf) override;
  };

  struct TournamentSelection : SelectionStrategy {
    virtual genome selection(Genpool &currentPopulation, executionConfig& eConf) override;
  };

}

#endif /* SELECTION_H */
