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

waypoints path::Action::getWaypoints(cv::Point start, cv::Point2f direction) {

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

waypoints path::Action::calEndpoint(cv::Point &start, cv::Point2f &direction){

  int timeout = 3;
  for (auto it = 0; it != timeout; ++it) {
    int x = round(start.x + direction.x * mod_config[ModificationParameter::Distance]);
    int y = round(start.y + direction.y * mod_config[ModificationParameter::Distance]);
    grid_map::Index idx(x,y);
    // Check if point is inside
    obstacle_map.at("env", idx);
  }

  return wp;
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
