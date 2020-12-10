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
  };

  enum class PathActionType
  {
        Ahead = 0,
	CAhead = 1,
	Start = 2,
	End = 3

   };

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


  // static grid_map::GridMap global_map;
  using namespace grid_map;

  template<typename K, typename V>
  void updateConfig(map<K,V> &config, map<K,V> &update);


  //TODO: adapt to direction vector
  direction radAngleToDir(float angle);

  direction angleToDir(float angle);
  float dirToAngle(direction pos);



  /////////////////////////////////////////////////////////////////////////////
  //                                PathAction                               //
  /////////////////////////////////////////////////////////////////////////////

  class PathAction{
    // An action shold be initialized by an action type
    // or better we need an action factory that will generate an action based on the
  public:
    const WPs get_wps() const { return wps; }

    void set_wps(const WPs wps) { this->wps = wps; }

    //TODO: substitute public inheritance with getter and setter
    // static grid_map::GridMap obstacle_map;
    const PAT get_type() const { return type; }

    PathAction(const PAT type):type(type){};
    // ~PathAction() = default;
    /*
      Update the current config parameters.
      Parameter:
      config the action specific configuration

      return:
      false if the configuration was faulty. The previous parameters will be used
     */
    // virtual bool updateConfig(robot_standard_config config);
    // virtual void calWaypoints(cv::Point start);
    // const robot_standard_config& getConfig(){return config;}
    // time_sec getEstimatedDuration(){return estimatedDuration;};
    virtual WPs generateWPs(Position start);
    // virtual waypoints calEndpoint(grid_map::Index &start);
    // grid_map::Index vecToIdx(direction vec);
    PA_config& getConfig(){return mod_config;}

    /**
       This will adapt the parameter actions to drive from given start point to last one
       Will alter the start point.
       Return false if no waypoints have been generated yet
     */
    bool mend(Position start);

    bool updateConf(PAP param, float val);
  // private:
  // protected:
    bool modified = true;
    const PAT type;
    time_sec estimatedDuration;
    WPs wps;
    PA_config mod_config;
  };

  using PAs = deque<shared_ptr<PathAction>>;

  /////////////////////////////////////////////////////////////////////////////
  //                              AheadAction                                 //
  /////////////////////////////////////////////////////////////////////////////

  class AheadAction : public  PathAction{
  public:
    AheadAction(path::PAT type, PA_config conf);
    virtual WPs generateWPs(Position start);
  };

  /////////////////////////////////////////////////////////////////////////////
  //                                EndAction                                //
  /////////////////////////////////////////////////////////////////////////////

  class EndAction : public PathAction{
  public:
    EndAction(WPs endPoints):PathAction(PAT::End){
      wps.insert(wps.begin(), endPoints.begin(), endPoints.end());
    };
    /*
      If no endpoints are given to the end action, the resulting waypoints will contain only the start point which will be listed twice, such that the execution can process it.
      TODO: mechanism to check if endpoints are coccupied
For now we will just return the start point because the robot object should find the shortest path to the possible endpoints
     */
    virtual WPs generateWPs(Position start);
    bool mend(Position start){return true;};
  };

  /////////////////////////////////////////////////////////////////////////////
  //                               StartAction                               //
  /////////////////////////////////////////////////////////////////////////////
  class StartAction : public PathAction{
  public:
    StartAction(Position startPoint):PathAction(PAT::Start) {
      wps.insert(wps.begin(), startPoint);
    };
  };

  /////////////////////////////////////////////////////////////////////////////
  //                                  Robot                                  //
  /////////////////////////////////////////////////////////////////////////////


  class Robot{

  public:
    const map<PathActionType, int> get_typeCount() const { return typeCount; }

    const WPs get_traveledPath() const { return traveledPath; }

    Position& get_currentPos() { return currentPos; }

    Robot(float initAngle, rob_config conf, GridMap &gMap);

    /*
      Execute an action on the given grid map.
      Return false if execution was not successful (object was in the way)
    */
    bool execute(shared_ptr<PathAction> action, grid_map::GridMap &map);

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
    bool mapMove(GridMap &cmap, shared_ptr<PathAction> action, int &steps, Position &currentPos, WPs &path, bool clean=true);

    cv::Mat gridToImg(string layer);


    rob_config getConfig(){return defaultConfig;};

    int getFreeArea();
  private:
    grid_map::GridMap cMap;
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
