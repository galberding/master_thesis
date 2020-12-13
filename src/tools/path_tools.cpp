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
  // if(abs(angle) > 360) throw __LINE__;
  return radAngleToDir(angle / (180/M_PI));
}

float path::dirToAngle(direction pos){
  float angle = atan2(pos[1], pos[0]) * (180 / M_PI);
  if(isnan(angle)) warn("Broken angle!!");
  return angle;
}


///////////////////////////////////////////////////////////////////////////////
//                                   Action                                  //
///////////////////////////////////////////////////////////////////////////////

WPs path::PathAction::generateWPs(Position start) {
  debug("Haha geflaxt!!");
  modified = false;
  return wps;
}

bool path::PathAction::updateConf(PAP param, float val) {
  mod_config[param] = val;
  return true;
}


bool path::PathAction::mend(PathAction &pa){
  // debug("Mending!");
  if(wps.size() == 0 && modified){
    // During initialization phase
    generateWPs(pa.wps.back());
    return true;
  }else if(wps.size() != 2) {
    warn("Waypoints not generated yet! Should never happen!");
    return false;
  }
  Position start = pa.get_wps().back();
  Position end = wps.back();
  Position V = end - start;
  float dist = V.norm();
  if(dist == 0){
    warn("Action has no distrance");
    return false;
  }
  float angle = dirToAngle(V/dist);

  debug("Mending -- Dist: ", dist, " Angle: ", angle);
  if(isnan(angle)) {
    warn("Error!");
    throw __LINE__;}
  // Update Parameter
  mod_config[PAP::Distance] = dist;
  mod_config[PAP::Angle] = angle;

  // set new start Position
  *wps.begin() = start;
  // Override all changes
  pa.modified = false;
  return true;
}

bool path::PathAction::applyMods(){
  // debug("Mods!");
 // Regenerate waypoints based on the already existing start Point
  // if waypoints are empty return false
  if(wps.size() == 0) return false;
  modified = true;
  debug("Apply Changes");
  generateWPs(wps.front());
  return true;
}

///////////////////////////////////////////////////////////////////////////////
//                                AheadAction                                //
///////////////////////////////////////////////////////////////////////////////

path::AheadAction::AheadAction(path::PAT type, PA_config conf):PathAction(type){
  mod_config.insert({{PAP::Angle, 0}, {PAP::Distance, 300}});
  updateConfig(mod_config, conf);
  modified = true;
}

WPs path::AheadAction::generateWPs(Position start) {
  if(modified || (start != wps.front())){
    wps.clear();
    direction dir = angleToDir(mod_config[PAP::Angle]);
    // cout << "Direction x: " << dir[0] << " Direction y: " << dir[1] << endl;
    wps.push_back(start);
    wps.push_back(start + dir*(mod_config[PAP::Distance]));
    // debug("Generate WPS ...");
    modified = false;
  }
  if(wps.size() != 2){
    warn("Generation not completed!");
    throw __LINE__;
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
  modified = false;
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
  Position pos(currentPos);
  switch(action->get_type()){
  case PAT::Start:{
    resetCounter();
    // map.clear("map");
    map.add("map", 0.0);
    // get start position for all following actions
    currentPos = action->generateWPs(currentPos)[0];
    debug("Startpoint: ", currentPos);
    res = true;
    break;
  }
  case PAT::Ahead:{
    res = mapMove(map, action, steps, currentPos, traveledPath, true);
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

  if(!res){
    Position start =  action->get_wps().front();
    float dist = (currentPos-start).norm();
    action->mod_config[PAP::Distance] = dist;
    action->applyMods();
    if(!map.isInside(action->wps.back())){
      warn("Generated Point outside!!");
    }
  }

  // std::cout << "Moved " << steps << " steps" << "\n";
  // debug("Moved ", steps, " steps");
  // if(!res && steps == 0){
  //   warn("Action from type ", static_cast<int>(action->get_type()), " failed");
  //   // Action failed, destroy action from genome
  //   return false;
  // }else
 // if (!res){
 //      // warn("Robot alters action");
 //      // TODO: Is the distance calculation sufficient??
 //      // TODO: Use all metrics in M
 //    // Action was at least partially executable
 //    // Case of Ahead action ...
 //     if(steps == 0) {
 //       for(int i=0; i<4; i++){
 // 	 action->modified = true;
 // 	 action->mod_config[PAP::Angle] += 90;
 // 	 action->applyMods();
 // 	 currentPos = action->wps.front();
 // 	 bool res2 = mapMove(map, action, steps, currentPos, traveledPath, true);
 // 	 if(res2  || steps > 0){
 // 	   info("---> Success <----");
 // 	   break;
 // 	 }else {
 // 	   warn("Adaptation ",i, " failed!");
 // 	 }
 //       }
 //     } else if(!res) {
 //      Position start =  action->get_wps().front();
 //      debug("Actual start: ", pos);
 //      float dist = (currentPos-start).norm();
 //      debug("old Dist: ",action->getConfig()[PAP::Distance],  " New distance: ", dist);
 //      action->updateConf(PAP::Distance, dist);
 //      action->modified = true;
 //      debug("Old end pos: ", action->get_wps().back());
 //      action->applyMods();
 //      debug("Current Pos: ", currentPos);
 //      debug("New end pos: ", action->get_wps().back());
 //      // return true;
 //     }

 //  }

  return res;
}

bool path::Robot::evaluateActions(PAs &pas){

  bool success = false;
  for(PAs::iterator it = begin(pas); it != end(pas); it++){
    debug("--------------------");
    success = execute(*it, cMap);
    debug("Success: ", success);
    if(success){
      incConfParameter(typeCount, (*it)->get_type(), 1);
    } else {

      // Check if the next gen is ready to be mended
      auto it_next = next(it, 1);
      while(!it_next->get()->mend(*(it->get())) && it_next != pas.end()){
	it_next = pas.erase(it_next);
	debug("WPs: ", it_next->get()->wps.size(), " Modified: ", it_next->get()->modified);
	warn("Remove Action while execution!");
	break;
      }
      // next(it, 1)->get()->mend(*it->get());
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
    warn("Map move failed, out of bounds, type: ", int(action->type));
    return false;
  }

  Position start =  waypoints.front();
  if(cmap.getClosestPositionInMap(start) != currentPos) {
    warn("Positions do not match!!");
    cout << cmap.getClosestPositionInMap(start) << endl;
    cout << currentPos << endl;
  }
  Position lastPos = waypoints.back();
  // debug("MapMove from: ", start[0], "|", start[1], " to ", lastPos[0] , "|", lastPos[1]);

  // Check if points are in range
  if(!cmap.isInside(start)){

    warn("Startpoint not in map range: ", start);
    debug(cmap.getClosestPositionInMap(start));

    throw __LINE__;
    return false;
  }

  Index lastIdx;
  int count = 0;
  for(grid_map::LineIterator lit(cmap, start, lastPos) ; !lit.isPastEnd(); ++lit){

    // Check if start or endpoint collidates with obstacle
    float obstacle = cmap.at("obstacle", *lit);
    // std::cout << "Cell" << "\n";

    if(obstacle > 0){
      // The current index collides with an object
      debug("Collision detected!");
      if(steps == 0){
	warn("Fail in first round!!");
	return false;
      }else{
	res = false;
      }
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
	debug("Mark map");
      }
      // increment steps (to determain if action was successful)
      // Could be omitted by checking the last index
      steps++;
    }
  }

  // Update last valid robot position (before possible collision)
  if (!cmap.getPosition(lastIdx, currentPos)) warn("Point Transition faulty!");
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
