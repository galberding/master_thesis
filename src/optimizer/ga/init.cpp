#include "init.h"
#include <memory>

bool init::applyAction(float proba, executionConfig& eConf){
  uniform_real_distribution<float> probaDist(0,1);
  return proba > probaDist(eConf.generator);
}

/////////////////////////////////////////////////////////////////////////////
//                               InitStrategy                              //
/////////////////////////////////////////////////////////////////////////////
void init::InitStrategy::operator()(Genpool& pool, executionConfig& eConf){
  for(int i=0; i<eConf.initIndividuals;i++){
    genome gen;
    (*this)(gen, eConf.initActions, eConf);
    pool.push_back(gen);
  }
}

void init::InitStrategy::operator()(Genpool_shr& pool, executionConfig& eConf){
  for(int i=0; i<eConf.initIndividuals;i++){
    genome gen;
    (*this)(gen, eConf.initActions, eConf);
    pool.push_back(make_shared<genome>(gen));
  }
}

void init::InitStrategy::operator()(genome_shr &gen, int len, executionConfig& eConf){
  (*this)(*gen, len, eConf);
}

void init::InitStrategy::operator()(genome &gen, int len, executionConfig& eConf){
  // PAs actions;
  gen.actions.clear();
  uniform_int_distribution<> angleDist(0,360);
  uniform_real_distribution<float> distanceDist(0,50);
  gen.actions.push_back(make_shared<StartAction>(StartAction(eConf.start)));
    for(int i=0; i<len; i++){
      PA_config config{{PAP::Angle,angleDist(eConf.generator)}, {PAP::Distance, distanceDist(eConf.generator)}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
    }
    gen.actions.push_back(make_shared<EndAction>(EndAction(eConf.ends)));
    // gen.actions = actions;
}

void init::InitStrategy::replaceZeroGensWithRandom(Genpool& pool){
  debug("Replace phase: ");


}
