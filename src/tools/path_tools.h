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
  };
  using PAP = PathActionParameter;


  using namespace std;
  using time_sec = uint32_t;
  using distance_cm = uint32_t;
  using rob_config = map<RobotProperty, double>;
  using PA_config = map<PAP, double>;
  using WPs = vector<shared_ptr<grid_map::Index>>;
  using direction = Eigen::Vector2f;



  template<typename K, typename V>
  void updateConfig(map<K,V> &config, map<K,V> &update);


  //TODO: adapt to direction vector
  direction radAngleToDir(double angle);

  direction angleToDir(double angle);

  grid_map::Index vecToIdx(direction vec);


  enum class PathActionType
  {
        Ahead = 0,
	CAhead = 1,
	Start = 2,
	End = 3
   };
  using PAT = PathActionType;

  class PathAction{
    // An action shold be initialized by an action type
    // or better we need an action factory that will generate an action based on the
  public:
    static grid_map::GridMap obstacle_map;
    const PAT get_type() const { return type; }

    PathAction(PAT type):type(type){};
    ~PathAction(){};
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
    virtual WPs generateWPs(grid_map::Index start);
    // virtual waypoints calEndpoint(grid_map::Index &start);
    // grid_map::Index vecToIdx(direction vec);
  // private:
  protected:
    bool modified = true;
    const PAT type;
    time_sec estimatedDuration;
    WPs wps;
    PA_config mod_config;
  };

  class AheadAction : protected  PathAction{
  public:
    AheadAction(path::PAT type, PA_config conf);
    WPs generateWPs(grid_map::Index start) override;


  };

  class EndAction : protected PathAction{
    EndAction(WPs endPoints):PathAction(PAT::End){
      wps.insert(wps.begin(), endPoints.begin(), endPoints.end());
    };
  };

  class StartAction : protected PathAction{
    StartAction(grid_map::Index startPoint, direction dir):PathAction(PAT::Start) {
      wps.insert(wps.begin(), make_shared<grid_map::Index>(startPoint));
    };
  };


  class Robot{

  public:
    Robot(double initAngle, rob_config conf):lastAngle(initAngle){
     defaultConfig = {
       {RobotProperty::Width_cm, 1},
       {RobotProperty::Height_cm, 1},
       {RobotProperty::Drive_speed_cm_s, 50},
       {RobotProperty::Clean_speed_cm_s, 20}};

     updateConfig(defaultConfig, conf);
     resetCounter();
    }

    /*
      Execute an action on the given grid map.
      Return false if execution was not successful (object was in the way)
    */
    bool execute(PathAction &action, grid_map::GridMap &map);

    /*
      Try to fix given action.
      If timeout criteria is met the return is false
     */
    bool fixOrDismiss(PathAction &action);

    void resetCounter();
  private:
    map<PathActionType, int> typeCount;
    rob_config defaultConfig;
    distance_cm traveledDist = 0;
    double lastAngle;
    direction lastDirection;
  };

  namespace path_mapping{

    // namespace to define all action to gridmap operations
  }
}


#endif // __PATH_TOOLS__
