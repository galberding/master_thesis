#ifndef __OPTI_ADAPTER__
#define __OPTI_ADAPTER__

#include "ga_path_generator.h"

using namespace ga;

namespace op {
  // namespace to organize the optimizer strategies
  using SelectionPool = list<pair<genome, genome>>;

  /////////////////////////////////////////////////////////////////////////////
  //                               InitStrategy                              //
  /////////////////////////////////////////////////////////////////////////////
  struct InitStrategy {
    virtual void operator()(Genpool& pool, executionConfig& eConf);
    virtual void operator()(genome &gen, int len, executionConfig& eConf);
  };
  /////////////////////////////////////////////////////////////////////////////
  //                            SelectionStrategy                             //
  /////////////////////////////////////////////////////////////////////////////
  struct SelectionStrategy {
    // Conditions:
    // - current Pool will be cleared here
    // - Selection Pool is supposed to be cleared before it is filled again here
    virtual void operator()(Genpool& currentPool, SelectionPool& selPool, executionConfig& eConf);
  };

  struct RouletteWheelSelection : SelectionStrategy{
    virtual void operator()(Genpool& currentPool, SelectionPool& selPool, executionConfig& eConf) override;
  };

  /////////////////////////////////////////////////////////////////////////////
  //                            CrossoverStrategy                             //
  /////////////////////////////////////////////////////////////////////////////
  struct CrossoverStrategy {
    // Conditions:
    // - Selection pools needs to be filled
    // - Next Pool is expected to be already emptied
    virtual void operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf);
  };

  /////////////////////////////////////////////////////////////////////////////
  //                             MutationStrategy                            //
  /////////////////////////////////////////////////////////////////////////////
  struct MutationStrategy {
    virtual void operator()(Genpool& currentPool, executionConfig& eConf);
    void addOrthogonalAngleOffset(genome& gen, executionConfig& eConf);
    void addRandomAngleOffset(genome& gen, executionConfig& eConf);
    void addPositiveDistanceOffset(genome& gen, executionConfig& eConf);
    void addNegativeDistanceOffset(genome& gen, executionConfig& eConf);
    bool applyMutation(float proba, executionConfig eConf){
      uniform_real_distribution<float> probaDist(0,1);
      return proba > probaDist(eConf.generator);
    }
  };


  /////////////////////////////////////////////////////////////////////////////
  //                                Optimizer                                 //
  /////////////////////////////////////////////////////////////////////////////
  struct Optimizer {
    executionConfig eConf;
    Optimizer(
	      shared_ptr<InitStrategy> init,
	      shared_ptr<SelectionStrategy> select,
	      shared_ptr<CrossoverStrategy> cross,
	      shared_ptr<MutationStrategy> mutate,
	      executionConfig eConf
	      ):eConf(eConf){}
  };
}


#endif //__OPTI_ADAPTER__
