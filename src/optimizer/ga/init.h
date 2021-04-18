#ifndef INIT_H
#define INIT_H
#include "../../tools/configuration.h"
#include "../../tools/debug.h"

namespace init {
  using namespace conf;

  /////////////////////////////////////////////////////////////////////////////
  //                               InitStrategy                              //
  /////////////////////////////////////////////////////////////////////////////
  bool applyAction(float proba, executionConfig& eConf);

  struct InitStrategy {
    virtual void operator()(Genpool& pool, executionConfig& eConf);
    virtual void operator()(genome &gen, int len, executionConfig& eConf);
    void replaceZeroGensWithRandom(Genpool& pool);
    void boustrophedon(genome &gen, executionConfig& eConf);
    void spiral(genome &gen, executionConfig& eConf);
  };
}

#endif /* INIT_H */
