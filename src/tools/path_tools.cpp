#include "path_tools.h"

// #ifdef __DEBUG__
// #undef __DEBUG__
// #define __DEBUG__ false
// #endif

using namespace path;


// grid_map::GridMap path::global_map = grid_map::GridMap();

///////////////////////////////////////////////////////////////////////////////
//                                Path Helper                                //
///////////////////////////////////////////////////////////////////////////////

template<typename K, typename V>
void path::updateConfig(map<K,V> &config, map<K,V> &update){
    for (auto &[key, value] : config){
      if(update.find(key) != update.end()){
	value = update[key];
       }
    }
}

template<typename K, typename V>
void resetConfParameter(map<K, V> &config, K key){
  // set Parameter if not exists
  auto ret = config.insert(pair(key, 0.0));
  if(ret.second == false){
    config[key] = 0;
  }
}

template<typename K, typename V>
void incConfParameter(map<K, V> &config,const K key, V value){
  // set Parameter if not exists
  auto ret = config.insert(pair(key, value));
  if(ret.second == false){
    config[key]++;
  }
}


direction path::radAngleToDir(float angle_rad){
  return direction(cos(angle_rad), sin(angle_rad));
}

direction path::angleToDir(float angle){
  return radAngleToDir(angle / (180/M_PI));
}

float path::dirToAngle(direction pos){
  return atan2(pos[1], pos[0]) * (180 / M_PI);
}


///////////////////////////////////////////////////////////////////////////////
//                                   Action                                  //
///////////////////////////////////////////////////////////////////////////////

WPs path::PathAction::generateWPs(Position start) {
  return wps;
}

bool path::PathAction::updateConf(PAP param, float val) {
  mod_config[param] = val;
  return true;
}


bool path::PathAction::mend(PathAction &pa){
  if(wps.size() != 2) return false;
  Position start = pa.get_wps().back();
  Position end = wps.back();
  Position V = end - start;
  float dist = V.norm();
  float angle = dirToAngle(V/dist);

  // Update Parameter
  mod_config[PAP::Distance] = dist;
  mod_config[PAP::Angle] = angle;

  // set new start Position
  *wps.begin() = start;
  return true;
}

bool path::PathAction::applyMods(){
 // Regenerate waypoints based on the already existing start Point
  // if waypoints are empty return false
  if(wps.size() == 0) return false;

  generateWPs(*wps.begin());
  return true;
}

///////////////////////////////////////////////////////////////////////////////
//                                AheadAction                                //
///////////////////////////////////////////////////////////////////////////////

path::AheadAction::AheadAction(path::PAT type, PA_config conf):PathAction(type){
  mod_config.insert({{PAP::Angle, 0}, {PAP::Distance, 300}});
  updateConfig(mod_config, conf);
}

WPs path::AheadAction::generateWPs(Position start) {
  if(modified){
    wps.clear();
    direction dir = angleToDir(mod_config[PAP::Angle]);
    // cout << "Direction x: " << dir[0] << " Direction y: " << dir[1] << endl;
    wps.push_back(start);
    wps.push_back(start + dir*(mod_config[PAP::Distance] / 100));
  }
  return wps;
}

///////////////////////////////////////////////////////////////////////////////
//                                 EndAction                                 //
///////////////////////////////////////////////////////////////////////////////

WPs path::EndAction::generateWPs(Position start) {
  WPs b;
  b.push_back(start);
  if(wps.size()){
    b.push_back(wps[0]);
  }else{
    b.push_back(start);
  }

  return b;
}

///////////////////////////////////////////////////////////////////////////////
//                                   Robot                                   //
///////////////////////////////////////////////////////////////////////////////


path::Robot::Robot(float initAngle, rob_config conf, GridMap &gMap):lastAngle(initAngle), cMap(gMap){
     defaultConfig = {
       {RobotProperty::Width_cm, 1},
       {RobotProperty::Height_cm, 1},
       {RobotProperty::Drive_speed_cm_s, 50},
       {RobotProperty::Clean_speed_cm_s, 20}};

     updateConfig(defaultConfig, conf);
     resetCounter();
    }



bool path::Robot::execute(shared_ptr<PathAction> action, grid_map::GridMap &map) {

  int steps = 0;
  bool res = false;
  switch(action->get_type()){
  case PAT::Start:{
    resetCounter();
    // map.clear("map");
    map.add("map", 0.0);
    // get start position for all following actions
    currentPos = action->generateWPs(currentPos)[0];
    res = true;
    break;
  }
  case PAT::Ahead:{
    res = mapMove(map, action, steps, currentPos, traveledPath, false);
    break;
  }
  case PAT::CAhead:{
    res = mapMove(map, action, steps, currentPos, traveledPath, true);
    break;
  }
  case PAT::End:{
    WPs wps = action->generateWPs(currentPos);
    // res = mapMove(action, steps, currentPos);
    wps = findShortestEndpointPath(wps);
    res = true;
    break;
  }
  default:{
    // TODO: What in case of error?
    warn("Unknown Action detected!");
    break;
  }
  }

  // std::cout << "Moved " << steps << " steps" << "\n";
  // debug("Moved ", steps, " steps");
  // if(!res && steps == 0){
  //   warn("Action from type ", static_cast<int>(action->get_type()), " failed");
  //   // Action failed, destroy action from genome
  //   return false;
  // }else
    if (!res){
      // TODO: Is the distance calculation sufficient??
      // TODO: Use all metrics in M
    // Action was at least partially executable
    // Case of Ahead action ...
    Position start =  action->get_wps().front();
    float dist = (start-currentPos).norm() * 100;
    // debug("New distance: ", dist);
    action->updateConf(PAP::Distance, dist);
    // return true;
  }

  return res;
}

bool path::Robot::evaluateActions(PAs &pas){

  bool success = false;

  for(PAs::iterator it = begin(pas); it != end(pas); it++){
    success = execute(*it, cMap);
    if(success){
      incConfParameter(typeCount, (*it)->get_type(), 1);
    } else {
      // TODO: Adapt consecutive action if current action could only be executed partially
      next(it, 1)->get()->mend(*it->get());
      // it = pas.erase(it);
    }
  }

  if(pas.size() < 3){
    warn("Only start and end action remain in list");
    return false;
  }
  return true;
}


void path::Robot::resetCounter() {
  typeCount = {{PathActionType::Ahead, 0},
	       {PathActionType::CAhead, 0}
  };
  resetConfParameter(typeCount, PAT::Ahead);
  resetConfParameter(typeCount, PAT::CAhead);
  resetConfParameter(typeCount, PAT::Start);
  resetConfParameter(typeCount, PAT::End);
  traveledPath.clear();
  traveledDist = 0;
}

WPs path::Robot::findShortestEndpointPath(WPs endpoints) {
  WPs b;

  return b;
}

bool path::Robot::mapMove(GridMap &cmap, shared_ptr<PathAction> action, int &steps, Position &currentPos, WPs &path, bool clean) {

  // TODO: Ensure start and endpoint are given in waypoint -> use this assumption exclusively
  // AS LONG as we do not have any other actions available
  WPs waypoints = action->generateWPs(currentPos);
  // Check if wp generation was successful
  steps = 0;
  bool res = true;
  if(waypoints.size() < 2){
    warn("Map move failed, out of bounds!");
    return false;
  }

  Position start =  waypoints.front();
  Position lastPos = waypoints.back();
  // debug("MapMove from: ", start[0], "|", start[1], " to ", lastPos[0] , "|", lastPos[1]);

  // Check if points are in range
  // if(!cmap.isInside(start) || !cmap.isInside(lastPos)){
  //   // debug("Waypoints not in map range");
  //   return false;
  // }

  Index lastIdx;
  for(grid_map::LineIterator lit(cmap, start, lastPos) ; !lit.isPastEnd(); ++lit){

    // Check if start or endpoint collidates with obstacle
    float obstacle = cmap.at("obstacle", *lit);
    // std::cout << "Cell" << "\n";

    if(obstacle > 0){
      // The current index collides with an object
      // debug("Collision detected!");
      res = false;
      break;
    }else{
      // Update last position
      lastIdx = *lit;
      if (clean){
	float mapVal = cmap.at("map", *lit);
	if (mapVal){
	  // TODO: happens almost always at the beginning when marking a new action
	  // debug("Crossing Path detected!");
	  incConfParameter(action->getConfig(), PAP::CrossCount, static_cast<float>(mapVal));
	}
	cmap.at("map", *lit)++;
      }
      // increment steps (to determain if action was successful)
      // Could be omitted by checking the last index
      steps++;
    }
  }
  // Update last valid robot position (before possible collision)
  cmap.getPosition(lastIdx, currentPos);
  // Update Waypoints
  path.push_back(start);
  path.push_back(currentPos);
  return res;
}


cv::Mat path::Robot::gridToImg(string layer){
  cv::Mat img;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(cMap, layer, CV_8U, 0.0, 2, img);
  return img;
}

int path::Robot::getFreeArea(){
  return cMap.getSize().x()*cMap.getSize().y() - (cMap.get("obstacle").sum());

}
