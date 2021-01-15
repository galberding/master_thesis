#ifndef __PATH_TOOLS__
#define __PATH_TOOLS__

#include <memory>
#include <opencv2/opencv.hpp>
#include <experimental/random>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <stack>
#include <thread>
#include <future>
#include <exception>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <cstring>
#include "mapGen.h"

#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define CYAN    "\033[36m"

// #define __DEBUG__ true
// #define __DEBUG_DEBUG__ true
// #define __DEBUG_INFO__ true
// #define __DEBUG_WARN__ true

#include "debug.h"

// #ifdef __DEBUG__
#undef __DEBUG__
#define __DEBUG__ false
// #endif
namespace path {

  /*
    Size parameter should be given in cm
    Speed parameter should be given in cm/s
   */
  enum class RobotProperty{

    Width_cm = 0,
    Height_cm = 1,
    Drive_speed_cm_s = 2,
    Clean_speed_cm_s = 3,
    // Rotation_speed = 4,
  };

  enum class PathActionParameter {
    AngleOffset = 0,
    Angle = 1,
    Distance = 2,
    DistanceOffset = 3,
    CrossCount = 4, // How often did the action cross other actions
    StepCount = 5
  };

  enum class PathActionType
  {
        Ahead = 0,
	CAhead = 1,
	Start = 2,
	End = 3

   };

  enum class Counter{
    CrossCount = 0,
    StepCount = 1
  };
  struct PathAction;

  using RP = RobotProperty;
  using PAP = PathActionParameter;
  using PAT = PathActionType;
  using namespace std;
  using time_sec = uint32_t;
  using distance_cm = uint32_t;
  using rob_config = map<RobotProperty, float>;
  using PA_config = map<PAP, float>;
  using WPs = vector<grid_map::Position>;
  //Direction needs to be a normed vector
  using direction = grid_map::Position;
  using PAs = deque<shared_ptr<PathAction>>;


  // static grid_map::GridMap global_map;
  using namespace grid_map;

  template<typename K, typename V>
  void updateConfig(map<K,V> &config, map<K,V> &update);


  //TODO: adapt to direction vector
  direction radAngleToDir(float angle);

  direction angleToDir(float angle);

  // Convert direction vector to angle
  float dirToAngle(direction pos);

  // return true if the conversion to angle was successful
  bool dirToAngle(direction pos, float &angle);

  void actionToPath(PAs& pas, vector<Position> &path);

  bool compareF(float f1, float f2, float epsilon = 0.001);
  /////////////////////////////////////////////////////////////////////////////
  //                                PathAction                               //
  /////////////////////////////////////////////////////////////////////////////

  struct PathAction{
    // An action shold be initialized by an action type
    // or better we need an action factory that will generate an action based on the
    static uint32_t id;

    int pa_id;
    bool modified = false;
    PAT type;
    time_sec estimatedDuration;
    WPs wps;
    PA_config mod_config;
    map<Counter, int> c_config;

    PathAction(PAT type):
      pa_id(id),
      modified(false),
      type(type),
      c_config{
	{Counter::StepCount, 0},
	{Counter::CrossCount, 0}}{
      id++;
    };

    WPs get_wps() { return wps; }

    void set_wps(WPs wps) { this->wps = wps; }



    // ~PathAction() = default;

    virtual WPs generateWPs(Position start);
    // virtual waypoints calEndpoint(grid_map::Index &start);
    // grid_map::Index vecToIdx(direction vec);
    PA_config& getConfig(){return mod_config;}
    void setConfigByWaypoints(Position start, Position end);

    /**
       This will adapt the parameter actions to drive from given start point to last one
       Will alter the start point.
       Return false if no waypoints have been generated yet
     */
    virtual bool mendConfig(shared_ptr<PathAction> pa, bool overrideChanges=true);
    virtual bool applyModifications();

    bool updateConf(PAP param, float val);
  };





  /////////////////////////////////////////////////////////////////////////////
  //                              AheadAction                                 //
  /////////////////////////////////////////////////////////////////////////////

  struct AheadAction : public  PathAction{
    AheadAction(path::PAT type, PA_config conf);
    // AheadAction(AheadAction &aa);
    virtual WPs generateWPs(Position start);

  };

  /////////////////////////////////////////////////////////////////////////////
  //                                EndAction                                //
  /////////////////////////////////////////////////////////////////////////////

  struct EndAction : public PathAction{
    EndAction(WPs endPoints):PathAction(PAT::End){
      modified = false;
      wps.insert(wps.begin(), endPoints.begin(), endPoints.end());
    };
    /*
      If no endpoints are given to the end action, the resulting waypoints will contain only the start point which will be listed twice, such that the execution can process it.
      TODO: mechanism to check if endpoints are coccupied
For now we will just return the start point because the robot object should find the shortest path to the possible endpoints
     */
    virtual WPs generateWPs(Position start) override;
    virtual bool mendConfig(shared_ptr<PathAction> pa, bool overrideChanges) override {return true;}
    virtual bool applyModifications() override {return true;}
  };

  /////////////////////////////////////////////////////////////////////////////
  //                               StartAction                               //
  /////////////////////////////////////////////////////////////////////////////
  struct StartAction : public PathAction{
    StartAction(Position startPoint):PathAction(PAT::Start) {
      // wps.insert(wps.begin(), startPoint);
      mod_config.insert({{PAP::Angle, 0}, {PAP::Distance, 0}, {PAP::CrossCount, 0}, {PAP::StepCount, 0}});
      // updateConfig(mod_config, {{PAP::Distance, 0}});
      wps.push_back(startPoint);
    };
    bool mendConfig(shared_ptr<PathAction> pa, bool overrideChanges){return true;};
    bool applyModifications(){return true;}
  };

  /////////////////////////////////////////////////////////////////////////////
  //                                  Robot                                  //
  /////////////////////////////////////////////////////////////////////////////


  class Robot{

  public:
    const map<PathActionType, int> get_typeCount() const { return typeCount; }

    const WPs get_traveledPath() const { return traveledPath; }

    Position& get_currentPos() { return currentPos; }

    // Robot(float initAngle, rob_config conf, GridMap &gMap);
    Robot(rob_config conf, shared_ptr<GridMap> gmap, string mapOperationName);

    /*
      Execute an action on the given grid map.
      Return false if execution was not successful (object was in the way)
    */
    bool execute(shared_ptr<PathAction> action, shared_ptr<GridMap> map);

    bool evaluateActions(PAs &pas);

    void resetCounter();

    /*
      Given a list of endpoints we want to find the endpoint to which the path is shortest.
      TODO: use A* or similar to get the shortest path
     */
    WPs findShortestEndpointPath(WPs endpoints);

    /*
      Mark points on the map and cal
     */
    bool mapMove(shared_ptr<GridMap> cmap, shared_ptr<PathAction> action, int &steps, Position &currentPos, WPs &path, bool clean=true);

    cv::Mat gridToImg(string layer);


    rob_config getConfig(){return defaultConfig;};

    int getFreeArea();
  private:
    grid_map::GridMap cMap;
    shared_ptr<GridMap> pmap;
    string opName;
    map<PathActionType, int> typeCount;
    rob_config defaultConfig;
    distance_cm traveledDist = 0;
    WPs traveledPath;
    float lastAngle;
    direction lastDirection;
    Position currentPos;
  };
}


#endif // __PATH_TOOLS__
