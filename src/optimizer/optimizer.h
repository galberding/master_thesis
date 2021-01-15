#ifndef __OPTI_ADAPTER__
#define __OPTI_ADAPTER__

// #include "ga_path_generator.h"
#include "../tools/pa_serializer.h"
#include "../tools/configuration.h"
#include "../tools/debug.h"
#include <algorithm>
// using namespace ga;
using namespace conf;

namespace op {
  // namespace to organize the optimizer strategies

  bool applyAction(float proba, executionConfig& eConf);

  /////////////////////////////////////////////////////////////////////////////
  //                               InitStrategy                              //
  /////////////////////////////////////////////////////////////////////////////
  struct InitStrategy {
    virtual void operator()(Genpool& pool, executionConfig& eConf);
    virtual void operator()(genome &gen, int len, executionConfig& eConf);
    void replaceZeroGensWithRandom(Genpool& pool);
  };
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

  };

  struct RouletteWheelSelection : SelectionStrategy{
    virtual genome selection(Genpool &currentPopulation, executionConfig& eConf) override;
  };

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
    void mating(genome &par1, genome &par2, T& newPopulation, executionConfig& eConf);
  };

  /////////////////////////////////////////////////////////////////////////////
  //                             MutationStrategy                            //
  /////////////////////////////////////////////////////////////////////////////
  struct MutationStrategy {
    virtual void operator()(Genpool& currentPool, executionConfig& eConf);
    virtual void operator()(FamilyPool& fPool, executionConfig& eConf);
    void mutateGen(genome &gen, executionConfig &eConf);
    void addOrthogonalAngleOffset(genome& gen, executionConfig& eConf);
    void addRandomAngleOffset(genome& gen, executionConfig& eConf);
    void addPositiveDistanceOffset(genome& gen, executionConfig& eConf);
    void addNegativeDistanceOffset(genome& gen, executionConfig& eConf);
    void randomScaleDistance(genome& gen, executionConfig& eConf);
  };


  /////////////////////////////////////////////////////////////////////////////
  //                             FitnessStrategy                             //
  /////////////////////////////////////////////////////////////////////////////

  struct FitnessStrategy {
    virtual void operator()(Genpool &currentPool, path::Robot &rob, executionConfig& eConf);
    void estimateChildren(FamilyPool& fPool, path::Robot &rob, executionConfig& eConf);
    virtual float calculation(genome& gen, int freeSpace, executionConfig &eConf);
    virtual float calculation(float cdist,
			      float dist,
			      int crossed,
			      float cSpeed_m_s,
			      float speed_m_s,
			      int freeSpace,
			      executionConfig& eConf);
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
    FamilyPool fPool;
    shared_ptr<Robot> rob;

    Optimizer(
	      shared_ptr<InitStrategy> init,
	      shared_ptr<SelectionStrategy> select,
	      shared_ptr<CrossoverStrategy> cross,
	      shared_ptr<MutationStrategy> mutate,
	      shared_ptr<FitnessStrategy> calFitness,
	      executionConfig eConf
	      ):init(init),
		select(select),
		cross(cross),
		mutate(mutate),
		calFitness(calFitness),
		eConf(eConf){
      rob = make_shared<Robot>(Robot(eConf.rob_conf,
				     eConf.gmap,
				     eConf.obstacleName));
    }
    /**
       Continously optimize the path.
     If the current population is initialized -> size > 0,
     the initialization step will be omitted.
      */
    // private:
    //   Optimizer()
    void printRunInformation(executionConfig& eConf, float zeroPercent, bool display);
    void optimizePath(bool display = false);
    void restorePopulationFromSnapshot(const string path);
    void snapshotPopulation(const string path);
    void snapshotPopulation(executionConfig& eConf);
    void logAndSnapshotPool(executionConfig& eConf, float zeros);
  };
}


#endif //__OPTI_ADAPTER__
