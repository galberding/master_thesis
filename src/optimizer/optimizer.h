#ifndef __OPTI_ADAPTER__
#define __OPTI_ADAPTER__

#include "ga_path_generator.h"

using namespace ga;

namespace op {
  // namespace to organize the optimizer strategies
  using SelectionPool = list<pair<genome, genome>>;


  bool applyAction(float proba, executionConfig eConf);

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
    virtual genome selection(ga::Genpool &currentPopulation, executionConfig& eConf) = 0;

  };

  struct RouletteWheelSelection : SelectionStrategy{
    virtual genome selection(ga::Genpool &currentPopulation, executionConfig& eConf) override;
  };

  /////////////////////////////////////////////////////////////////////////////
  //                            CrossoverStrategy                             //
  /////////////////////////////////////////////////////////////////////////////
  struct CrossoverStrategy {
    // Conditions:
    // - Selection pools needs to be filled
    // - Next Pool is expected to be already emptied
    virtual void operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf) = 0;
    void copyActions(PAs::iterator begin, PAs::iterator end, PAs &child, bool modify=false);
  };

  struct DualPointCrossover : CrossoverStrategy {
    virtual void operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf) override;
    void mating(genome &par1, genome &par2, Genpool& newPopulation, executionConfig eConf);
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
  };


  /////////////////////////////////////////////////////////////////////////////
  //                             FitnessStrategy                             //
  /////////////////////////////////////////////////////////////////////////////

  struct FitnessStrategy {
    virtual void operator()(Genpool &currentPool, path::Robot &rob, executionConfig eConf);
    virtual float calculation(float cdist,
			      float dist,
			      int crossed,
			      float cSpeed_m_s,
			      float speed_m_s,
			      int freeSpace,
			      executionConfig eConf);
  };

  /////////////////////////////////////////////////////////////////////////////
  //                                Optimizer                                 //
  /////////////////////////////////////////////////////////////////////////////
  struct Optimizer {
    executionConfig eConf;
    shared_ptr<FitnessStrategy> calFitness;
    shared_ptr<MutationStrategy> mutate;
    shared_ptr<CrossoverStrategy> cross;
    shared_ptr<SelectionStrategy> select;
    shared_ptr<InitStrategy> init;
    Genpool pool;
    SelectionPool sPool;

    Optimizer(
	      shared_ptr<InitStrategy> init,
	      shared_ptr<SelectionStrategy> select,
	      shared_ptr<CrossoverStrategy> cross,
	      shared_ptr<MutationStrategy> mutate,
	      shared_ptr<FitnessStrategy> calFitness,
	      executionConfig eConf
	      ):calFitness(calFitness),mutate(mutate),cross(cross),select(select),init(init),eConf(eConf){}
  };
  /**
     Continously optimize the path.
     If the current population is initialized -> size > 0,
     the initialization step will be omitted.
   */
  void optimizePath();
  void restorePopulationFromSnapshot(string path);
  void snapshotPopulation(string path);
  
}


#endif //__OPTI_ADAPTER__
