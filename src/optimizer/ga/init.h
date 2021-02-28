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
  };
}

#endif /* INIT_H */
