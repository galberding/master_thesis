#include "path_tools.h"

using namespace path;

///////////////////////////////////////////////////////////////////////////////
//                                Path Helper                                //
///////////////////////////////////////////////////////////////////////////////

template<typename K, typename V>
void path::updateConfig(map<K,V> config, map<K,V> &update){
  for(auto it = config.begin(); it != config.end(); it++){
    if(update.find(it->first) != update.end()){
      it->second = update[it->first];
    }
  }
}

direction path::radAngleToDir(double angle){
  return direction(cos(angle), sin(angle));
}

direction path::angleToDir(double angle){
  return radAngleToDir(angle / (180/M_PI));
}

grid_map::Index vecToIdx(direction vec){
  return grid_map::Index(round(vec[0]), round(vec[1]));
}

///////////////////////////////////////////////////////////////////////////////
//                                   Action                                  //
///////////////////////////////////////////////////////////////////////////////

WPs path::PathAction::generateWPs(grid_map::Index start) {

  return wps;
}



grid_map::Index path::PathAction::vecToIdx(direction vec){

}

///////////////////////////////////////////////////////////////////////////////
//                                AheadAction                                //
///////////////////////////////////////////////////////////////////////////////

path::AheadAction::AheadAction(path::PAT type, PA_config conf):PathAction(type){
  mod_config.insert({{PAP::Angle, 0}, {PAP::Distance, 0}});
  updateConfig(mod_config, conf);
}

WPs path::AheadAction::generateWPs(grid_map::Index start){
  if(modified){
    wps.clear();
    direction dir = angleToDir(mod_config[PAP::Angle]);
    wps.push_back(make_shared<grid_map::Index>(start));
    dir[0] = start.x() + dir[0] * mod_config[PAP::Distance];
    dir[0] = start.y() + dir[1] * mod_config[PAP::Distance];
    wps.push_back(make_shared<grid_map::Index>(vecToIdx(dir)));
  }
  return wps;
}

///////////////////////////////////////////////////////////////////////////////
//                                   Robot                                   //
///////////////////////////////////////////////////////////////////////////////

bool path::Robot::execute(PathAction &action, grid_map::GridMap &map) {
  if(action.get_type() == PathActionType::Start){
    resetCounter();
  }
  return true;
}

bool path::Robot::fixOrDismiss(PathAction &action) {
  return true;
}

void path::Robot::resetCounter() {
  typeCount = {{PathActionType::Ahead, 0},
	       {PathActionType::CAhead, 0}};
}
