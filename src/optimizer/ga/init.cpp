#include "init.h"

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

void init::InitStrategy::spiral(genome &gen, executionConfig &eConf){
  gen.actions.clear();
  int wallsize = 3;
  gen.actions.push_back(make_shared<StartAction>(StartAction(eConf.start)));
  int dir = 0;
  float x_travel = 8.4;
  float y_travel = 8.4;
  // right
  PA_config config1{{PAP::Angle, 270}, {PAP::Distance, x_travel}};
  gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config1)));
  // Down
  PA_config config2{{PAP::Angle, 180}, {PAP::Distance, y_travel}};
  gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config2)));
  // left
  PA_config config3{{PAP::Angle, 90}, {PAP::Distance, y_travel}};
  gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config3)));
  dir = 3;
  // y_travel -= 0.6;

  for(int i = 0; i<29; i++){
    switch (dir) {
    case 0: { // Right -> down
      x_travel -= 0.6;
      PA_config config{{PAP::Angle, 270}, {PAP::Distance, x_travel}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
      dir = 1;
      break;
    }
    case 1: { // Down -> left
      y_travel -= 0.6;
      PA_config config{{PAP::Angle, 180}, {PAP::Distance, y_travel}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
      dir = 2;
      break;
    }
    case 2: { // Left -> Up
      x_travel -= 0.6;
      PA_config config{{PAP::Angle, 90}, {PAP::Distance, x_travel}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
      dir = 3;
      break;
    }
    case 3: { // Up -> right
      y_travel -= 0.6;
      PA_config config{{PAP::Angle, 0}, {PAP::Distance, y_travel}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
      dir = 0;
      break;
    }
    default:
      debug("Nothing");
      break;
    }
  }
  // PA_config config{{PAP::Angle, 180}, {PAP::Distance, 0.6}};
  // gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
  gen.actions.push_back(make_shared<EndAction>(EndAction(eConf.ends)));
}

void init::InitStrategy::boustrophedon(genome &gen, executionConfig &eConf){
  gen.actions.clear();
  int wallsize = 3;
  gen.actions.push_back(make_shared<StartAction>(StartAction(eConf.start)));
  int dir = 270;
  for(int i = 0; i<29; i++){
    switch (dir) {
    case 270: { // Right
      PA_config config{{PAP::Angle, 270}, {PAP::Distance, 8.4}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
      dir = 180;
      break;
    }
    case 180: { // Down
      PA_config config{{PAP::Angle, 180}, {PAP::Distance, 0.6}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
      dir = 90;
      break;
    }
    case 90: { // Left
      PA_config config{{PAP::Angle, 90}, {PAP::Distance, 8.4}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
      dir = 0;
      break;
    }
    case 0: { // Down
      PA_config config{{PAP::Angle, 180}, {PAP::Distance, 0.6}};
      gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
      dir = 270;
      break;
    }
    default:
      debug("Nothing");
      break;
    }
  }
  // PA_config config{{PAP::Angle, 180}, {PAP::Distance, 0.6}};
  // gen.actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
  gen.actions.push_back(make_shared<EndAction>(EndAction(eConf.ends)));
}
