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
  float angle = atan2(pos[1], pos[0]) * (180 / M_PI);
  // if(isnan(angle)) warn("Broken angle!!");
  return angle;
}

bool path::dirToAngle(direction pos, float &angle) {
  angle = dirToAngle(pos);
  return !isnan(angle);
}

void path::actionToPath(PAs& pas, vector<Position>& path) {
  for (auto it = pas.begin(); it != pas.end(); ++it) {
    assert((*it)->wps.size() > 0);
    path.push_back((*it)->wps.back());
  }
}

bool path::compareF(float f1, float f2, float epsilon){
  float diff = fabs(f1-f2);
  // debug(f1, "-", f2, "=", diff);
    if(diff < epsilon) return true;
    return false;
  }

///////////////////////////////////////////////////////////////////////////////
//                                   PathAction                                  //
///////////////////////////////////////////////////////////////////////////////

uint32_t path::PathAction::id = 0;

WPs path::PathAction::generateWPs(Position start) {
  // debug("Haha geflaxt!!");
  // debug("MyType: ", static_cast<int>(type));
  assertm(!(PAT::CAhead == type || PAT::Ahead == type), "C/Ahead called generic Implementation!!");
  modified = false;
  return wps;
}

bool path::PathAction::updateConf(PAP param, float val) {
  mod_config[param] = val;
  return true;
}

void path::PathAction::setConfigByWaypoints(Position start, Position end){
    // Start the mending process by
  float angle;
  Position V = end - start;
  float dist = V.norm();
  // ensure that the calculation does not divide by 0
  if(dist > 0 && dirToAngle(V/dist, angle)){
    // Only adapt the angle if distance is greater 0
    // otherwise we just want to ignore it
    mod_config[PAP::Angle] = angle;
  }

  // Update Parameter
  mod_config[PAP::Distance] = dist;

  if(wps.size() == 0){
    wps.push_back(start);
    wps.push_back(end);
  }
}


bool path::PathAction::mendConfig(shared_ptr<PathAction> pa, bool overrideChanges){

  // Choose one of the stategies with overrideChanges
  // [x] override change of current mutation --> always return true
  // [ ] apply changes and propagate to next action --> return false

  assertm(!(wps.size() == 0 && !modified), "Mending during initialization!");
  assertm(!(wps.size() == 0 && modified), "Modified action (probably added action while modification?) with no waypoints. \nWe do not allow this anymore!");
  // assertm(!(type == PAT::End), "End action was called for mending!");
  assertm(wps.size() >= 2, "Waypoint generation failure while mending!");


  if(modified && overrideChanges){
    // debug("Propagate changes");
    generateWPs(pa->wps.back());
    return false;
  }

  //TODO: Replace by setConfigByWaypoints
  // Start the mending process by
  float angle;
  Position start = pa->get_wps().back();
  Position end = wps.back();

  Position V = end - start;
  float dist = V.norm();
  // ensure that the calculation does not divide by 0
  if(dist > 0 && dirToAngle(V/dist, angle)){
    // Only adapt the angle if distance is greater 0
    // otherwise we just want to ignore it
    mod_config[PAP::Angle] = angle;
  }

  // Update Parameter
  mod_config[PAP::Distance] = dist;

  // set new start Position
  *wps.begin() = start;
  // Override all changes
  pa->modified = false;
  return true;
}

bool path::PathAction::applyModifications(){

  assertm(!(wps.size() == 0), "Cannot apply a changed config without waypoints");
  modified = true;
  generateWPs(wps.front());
  return true;
}

///////////////////////////////////////////////////////////////////////////////
//                                AheadAction                                //
///////////////////////////////////////////////////////////////////////////////

path::AheadAction::AheadAction(path::PAT type, PA_config conf):PathAction(type){
  mod_config.insert({{PAP::Angle, 0}, {PAP::Distance, 10}, {PAP::CrossCount, 0}, {PAP::StepCount, 0}});
  updateConfig(mod_config, conf);
  // modified = false;
}
// path::AheadAction::AheadAction(AheadAction &aa):AheadAction(aa.type, ){
//   wps = aa.wps;

// }

WPs path::AheadAction::generateWPs(Position start) {

  if(modified || (wps.size() == 0)  || (start != wps.front())){
    wps.clear();
    direction dir = angleToDir(mod_config[PAP::Angle]);
    // cout << "Direction x: " << dir[0] << " Direction y: " << dir[1] << endl;
    wps.push_back(start);
    wps.push_back(start + dir*(mod_config[PAP::Distance]));
    // debug("Generate WPS ...");
    modified = false;
  }
  assertm(wps.size() >= 2, "Waypoint generation failed with too few points!");
  // if(wps.size() != 2){
  //   warn("Generation not completed!");

  // }
  // assertm(mod_config[PAP::Distance], "");
  return wps;
}

///////////////////////////////////////////////////////////////////////////////
//                                 EndAction                                 //
///////////////////////////////////////////////////////////////////////////////

WPs path::EndAction::generateWPs(Position start){
  // debug("End ACtion WP generation");
  WPs b;
  b.push_back(start);
  if(wps.size()){
    b.push_back(wps[0]);
  }else{
    b.push_back(start);
  }
  // modified = false;
  return b;
}

///////////////////////////////////////////////////////////////////////////////
//                                   Robot                                   //
///////////////////////////////////////////////////////////////////////////////


// path::Robot::Robot(float initAngle, rob_config conf, GridMap &gMap):lastAngle(initAngle), cMap(gMap), pmap(make_shared<GridMap>(cMap)), opName("map"){
//      defaultConfig = {
//        {RobotProperty::Width_cm, 1},
//        {RobotProperty::Height_cm, 1},
//        {RobotProperty::Drive_speed_cm_s, 50},
//        {RobotProperty::Clean_speed_cm_s, 20}};

//      updateConfig(defaultConfig, conf);
//      resetCounter();
//     }

path::Robot::Robot(rob_config conf, shared_ptr<GridMap> gmap, string mapOperationName)
  :cMap(*gmap),
   pmap(gmap),
   opName(mapOperationName){
  defaultConfig = {
       {RobotProperty::Width_cm, 1},
       {RobotProperty::Height_cm, 1},
       {RobotProperty::Drive_speed_cm_s, 50},
       {RobotProperty::Clean_speed_cm_s, 20}};

  updateConfig(defaultConfig, conf);
  pmap->add(opName, 0);
  resetCounter();
}


bool path::Robot::execute(shared_ptr<PathAction> action, shared_ptr<grid_map::GridMap> map) {

  int steps = 0;
  bool res = false;

  Position pos(currentPos);

  switch(action->type){
  case PAT::Start:{
    resetCounter();
    // map.clear("map");
    map->add(opName, 0.0);
    // get start position for all following actions
    currentPos = action->generateWPs(currentPos)[0];
    // debug("Startpoint: ", currentPos);
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
    action->setConfigByWaypoints(currentPos, action->endPoint);
    res = mapMove(map, action, steps, currentPos, traveledPath, true);
    break;
  }
  default:{
    assertm(false, ("Unknown Action to process while execution: " + to_string(static_cast<int>(action->type))));
    break;
  }
  }

  // Post processing of action
  if(!res){
    // Ignore actions that have no distance
    if(action->c_config[Counter::StepCount] == 0){
      // debug("Action has distance 0");
      return true;
    }

    // Recalculate action parameter
    Position start =  action->get_wps().front();
    float dist = (currentPos-start).norm();
    action->mod_config[PAP::Distance] = dist;
    // action->modified = true;
    // action->generateWPs(action->wps.front());
    action->applyModifications();
    assertm(map->isInside(action->wps.back()), "Generated point outside the map");
  }

  // When initializing we do not want to modify the next action
  return res;
}

bool path::Robot::evaluateActions(PAs &pas){

  bool success = false;
  bool overrideChanges = true;

  for(PAs::iterator it = begin(pas); it != end(pas); it++){
    // debug("--------------------");

    bool init = ((*it)->wps.size() == 0 ) && !(*it)->modified;
    // assertm()
    assertm(!((*it)->type == PAT::Start && init), "Init should be skipped if start action appeared!");
    if(init){
      // initialize the action by generating the waypoints
      // this enables evaluateActions to handle completely uninitialized action sequences
      (*it)->modified = true;
      (*it)->generateWPs((*prev(it, 1))->wps.back());
    }

    success = execute(*it, pmap);
    // debug("Success: ", success);
    // do not propagate changes when actions are initialized
    if(!(success || init)){
      auto it_next = next(it, 1);
      auto it_prev = it;
      while(it_next != pas.end() && !((*it_next)->mendConfig(*it_prev, overrideChanges))){
	// debug("Propagate change ", static_cast<int>((*it_next)->type));
	it_prev = it_next;
	it_next++;
      }
    }
    incConfParameter(typeCount, (*it)->type, 1);
  }
  // assertm(pas.size() >= 3, "TOO few action remain in sequence");
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
  b.push_back(endpoints.front());

  return b;
}

bool path::Robot::mapMove(shared_ptr<GridMap> cmap, shared_ptr<PathAction> action, int &steps, Position &currentPos, WPs &path, bool clean) {


  WPs waypoints = action->generateWPs(currentPos);
  // Reset counter values

  resetConfParameter(action->c_config, Counter::CrossCount);
  resetConfParameter(action->c_config, Counter::StepCount);
  // Check if wp generation was successful
  steps = 0;
  bool res = true;

  // Check if start and endpoint are contained
  assertm(waypoints.size() >= 2, "Map move failed, not enough points given by action");
  Position start =  waypoints.front();
  if(cmap->getClosestPositionInMap(start) != currentPos){
    warn("Start: ", start, " Closest to start: ", cmap->getClosestPositionInMap(start), " Current: ", currentPos);
    assertm(cmap->getClosestPositionInMap(start) == currentPos, "Positions do not match!");
  }
  Position lastPos = waypoints.back();
  // debug("MapMove from: ", start[0], "|", start[1], " to ", lastPos[0] , "|", lastPos[1]);

  // Check if points are in range
  assertm(cmap->isInside(start), "Start-point is not in map range!");
  assertm(cmap->atPosition("obstacle", start) == 0, "Start Position is inside obstacle!!");
  if(action->mod_config[PAP::Distance] == 0){
    // Action is not doing anything, delete it!!
    // warn("Delete action ", action->pa_id, ", distance between start and endpoint == ", (waypoints.back()-waypoints.front()).norm());

    return false;
  }

  Index lastIdx;
  // TODO: setting values directly in the matrix is faster!
  // Matrix& moveMap = (*cmap)[opName];
  // Matrix& obstacleMap = (*cmap)["obstacle"];
  for(grid_map::LineIterator lit(*cmap, start, lastPos) ; !lit.isPastEnd(); ++lit){

    // Check if start or endpoint collidates with obstacle
    float obstacle = cmap->at("obstacle", *lit);
    // std::cout << "Cell" << "\n";

    if(obstacle > 0){
      // The current index collides with an object
      // debug("Collision detected!");
      if(action->c_config[Counter::StepCount] == 0){
	warn("Fail in first round!!");
	// TODO: This is a problem because the position seems to be outside
	//  so some control mechanism allows for point creation in obstacles!
	return false;
      }else{
	res = false;
      }
      break;
    }else{
      // Update last position
      lastIdx = *lit;
      if (clean){
	// TODO: Not prepared for pure Ahead action
	float mapVal = cmap->at(opName, *lit);
	// debug("Map Val: ", mapVal);
	if (mapVal > 0){
	  action->c_config[Counter::CrossCount] +=  cmap->at(opName, *lit);
	}
	action->c_config[Counter::StepCount] +=  1;
	cmap->at(opName, *lit) += 1;
	// debug("Mark map");
      }
      steps++;
    }
  }

  // Update last valid robot position (before possible collision)
  // debug("Track Current Pos: ", currentPos[0], "|", currentPos[1]);
  if (!cmap->getPosition(lastIdx, currentPos)) {
    debug("Track Current Pos: ", currentPos[0], "|", currentPos[1]);
    debug("Last index: ", lastIdx);
    warn("Point Transition faulty!");
    res = false;
    // assertm(false, "Cannot convert index to Position!");
  }
  // Update Waypoints
  path.push_back(start);
  path.push_back(currentPos);
  return res;
}


cv::Mat path::Robot::gridToImg(string layer){
  cv::Mat img = mapgen::gmapToImg(mapgen::changeMapRes(pmap, 0.2), layer);
  // grid_map::GridMapCvConverter::toImage<unsigned char, 1>(*pmap, layer, CV_8U, 0.0, 6, img);
  return img;
}

int path::Robot::getFreeArea(){
  if (!(freeArea > 0))
    freeArea =  pmap->getSize().x()*pmap->getSize().y() - (pmap->get("obstacle").sum());
  return freeArea;
}
