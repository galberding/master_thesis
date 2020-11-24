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

cv::Point2f path::radAngleToDir(double angle){
  return cv::Point2f(cos(angle), sin(angle));
}

cv::Point2f path::angleToDir(double angle){
  return radAngleToDir(angle / (180/M_PI));
}

///////////////////////////////////////////////////////////////////////////////
//                                   Action                                  //
///////////////////////////////////////////////////////////////////////////////

waypoints& path::Action::getWaypoints(cv::Point start) {

  if (modified){
    switch(type){
    case ActionType::Start:
      break;
    case ActionType::End:
      break;
    case ActionType::Ahead: case ActionType::CAhead:
      break;
    }
  }
  waypoints b;

  return b;
}


///////////////////////////////////////////////////////////////////////////////
//                                   Robot                                   //
///////////////////////////////////////////////////////////////////////////////


bool path::Robot::execute(Action &action, grid_map::GridMap &map) {
  if(action.get_type() == ActionType::Start){
    resetCounter();
  }
  return true;
}

bool path::Robot::fixOrDismiss(Action &action) {
  return true;
}

void path::Robot::resetCounter() {
  typeCount = {{ActionType::Ahead, 0},
		  {ActionType::CAhead, 0}};
}
