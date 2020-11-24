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

  enum class ModificationParameter {
    AngleOffset = 0,
    Angle = 1,
    Distance = 2,
    DistanceOffset = 3,
  };


  using namespace std;
  using time_sec = uint32_t;
  using distance_cm = uint32_t;
  using robot_standard_config = map<RobotProperty, double>;
  using modufication_config = map<ModificationParameter, double>;
  using waypoints = vector<shared_ptr<cv::Point>>;



  template<typename K, typename V>
  void updateConfig(map<K,V> config, map<K,V> &update);


  cv::Point2f radAngleToDir(double angle);

  cv::Point2f angleToDir(double angle);


  enum class ActionType
  {
        Ahead = 0,
	CAhead = 1,
	Start = 2,
	End = 3
   };
  class Action{
    // An action shold be initialized by an action type
    // or better we need an action factory that will generate an action based on the
  public:
    const ActionType get_type() const { return type; }

    Action(ActionType type);
    ~Action(){};
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
    waypoints& getWaypoints(cv::Point start);

  private:
    bool modified = true;
    const ActionType type;
    time_sec estimatedDuration;
    waypoints wp;
    robot_standard_config config;
    modufication_config mod_config;
    double angle;
    cv::Point2f direction;

  };



  class Robot{

  public:
    Robot(double initAngle, robot_standard_config conf):lastAngle(initAngle){
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
    bool execute(Action &action, grid_map::GridMap &map);

    /*
      Try to fix given action.
      If timeout criteria is met the return is false
     */
    bool fixOrDismiss(Action &action);

    void resetCounter();
  private:
    map<ActionType, int> typeCount;
    robot_standard_config defaultConfig;
    distance_cm traveledDist = 0;
    double lastAngle;
    cv::Point2f lastDirection;


  };

  namespace path_mapping{

    // namespace to define all action to gridmap operations
  }
}


#endif // __PATH_TOOLS__
