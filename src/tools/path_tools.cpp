#include "path_tools.h"

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
void incConfParameter(map<K, V> &config, K key, V value){
  // set Parameter if not exists
  auto ret = config.insert(pair(key, value));
  if(ret.second == false){
    config[key] += value;
  }
}


direction path::radAngleToDir(double angle_rad){
  return direction(cos(angle_rad), sin(angle_rad));
}

direction path::angleToDir(double angle){
  return radAngleToDir(angle / (180/M_PI));
}

///////////////////////////////////////////////////////////////////////////////
//                                   Action                                  //
///////////////////////////////////////////////////////////////////////////////

WPs path::PathAction::generateWPs(Position start) {

  return wps;
}


///////////////////////////////////////////////////////////////////////////////
//                                AheadAction                                //
///////////////////////////////////////////////////////////////////////////////

path::AheadAction::AheadAction(path::PAT type, PA_config conf):PathAction(type){
  mod_config.insert({{PAP::Angle, 0}, {PAP::Distance, 0}});
  updateConfig(mod_config, conf);
}

WPs path::AheadAction::generateWPs(Position start){
  if(modified){
    wps.clear();
    direction dir = angleToDir(mod_config[PAP::Angle]);
    cout << "Direction x: " << dir[0] << " Direction y: " << dir[1] << endl;
    wps.push_back(start);
    cout << "Direction x: " << dir[0] << " Direction y: " << dir[1] << endl;
    wps.push_back(start + dir*(mod_config[PAP::Distance] / 100));
  }
  return wps;
}

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


path::Robot::Robot(double initAngle, rob_config conf, GridMap &gMap):lastAngle(initAngle), cMap(gMap){
     defaultConfig = {
       {RobotProperty::Width_cm, 1},
       {RobotProperty::Height_cm, 1},
       {RobotProperty::Drive_speed_cm_s, 50},
       {RobotProperty::Clean_speed_cm_s, 20}};

     updateConfig(defaultConfig, conf);
     resetCounter();
    }



bool path::Robot::execute(PathAction &action, grid_map::GridMap &map) {

  int steps = 0;
  bool res = false;
  switch(action.get_type()){
  case PAT::Start:{
    resetCounter();
    // map.clear("map");
    map.add("map", 0.0);
    // get start position for all following actions
    currentPos = action.generateWPs(currentPos)[0];
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
    WPs wps = action.generateWPs(currentPos);
    // res = mapMove(action, steps, currentPos);
    wps = findShortestEndpointPath(wps);
    break;
  }
  default:{
    // TODO: What in case of error?
    throw __LINE__;
    break;
  }
  }

  std::cout << "Moved " << steps << " steps" << "\n";

  if(!res && steps == 0){
      // Action failed, destroy action from genome
      return false;
    }else if (!res){
      // Action was at least partially executable
      // TODO: Adapt action parameter (How??)
    //Add generated waypoints to the complete path
    }
  return res;
}

bool path::Robot::evaluateActions(PAs &pas){

  // TODO: Only add endpoints to path
  bool success;
  for (auto it = pas.begin(); it != pas.end();){

    success = execute(**it, cMap);

    if(success){
      it++;
      incConfParameter(typeCount, (*it)->get_type(), 1);
    } else {
      // remove Action from sequence
      it = pas.erase(it);
    }
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

bool path::Robot::mapMove(GridMap &cmap, PathAction& action, int &steps, Position &currentPos, WPs &path, bool clean) {

  // TODO: Ensure start and endpoint are given in waypoint -> use this assumption exclusively
  // as long as we do not have any other actions available
  WPs waypoints = action.generateWPs(currentPos);
  // Check if wp generation was successful
  steps = 0;
  bool res = true;
  if(waypoints.size() < 2){
    std::cout << "Map Move Failed!" << "\n";


    return false;
  }else{
    std::cout << "Start Map move" << "\n";

  }

  // for (WPs::iterator it = begin(waypoints); it != end(waypoints);){
    Position start =  waypoints.front();
    Position lastPos = waypoints.back();
    std::cout << "Start and last waypoint:" << "\n";
    std::cout << start << "\n";
    std::cout << lastPos << "\n";

    // Check if points are in range
    if(!cmap.isInside(start) || !cmap.isInside(lastPos)){
      cout << "Not inside!" << endl;
      return false;
    }

    Index lastIdx;
    for(grid_map::LineIterator lit(cmap, start, lastPos) ; !lit.isPastEnd(); ++lit){

      // Check if start or endpoint collidates with obstacle
      float obstacle = cmap.at("obstacle", *lit);
      std::cout << "Cell" << "\n";

      if(obstacle > 0){
	// The current index collides with an object
	// TODO: store the last valid position to the waypoints
	// return false;
	res = false;
	break;
      }else{
	// Update last position
	lastIdx = *lit;
	if (clean){
	  float mapVal = cmap.at("map", *lit);
	  if (mapVal)
	    incConfParameter(action.getConfig(), PAP::CrossCount, static_cast<double>(mapVal));
	  cmap.at("map", *lit)++;
	}
	// get Position and put it to the visited Path

	// if(cMap.getPosition(*lit, last))
	//   path.push_back(last);
	steps++;
      }
    }

    cmap.getPosition(lastIdx, currentPos);
  // }

    // cout << "Inner Sum: " << cmap.get("map").sum() << endl;
  // Append all collected waypoints

  return res;
}


cv::Mat path::Robot::gridToImg(string layer){
  cv::Mat img;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(cMap, "obstacle", CV_8U, 0.0, 1, img);
  return img;
}
