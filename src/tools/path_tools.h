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
    End
  };


  using namespace std;
  using time_sec = uint32_t;
  using robot_standard_config = map<RobotProperty, double>;
  using waypoints = vector<shared_ptr<cv::Point>>;

  template<typename K, typename V>
  void updateConfig(map<K,V> config, map<K,V> &update){
    for(auto it = config.begin(); it != config.end(); it++){
	if(update.find(it->first) != update.end()){
      it->second = update[it->first];
    }
  }
  }

  enum class ActionType
  {
        Ahead = 0,
	CAhead = 1,
	Rotate = 2
   };
  class Action{
    // An action shold be initialized by an action type
    // or better we need an action factory that will generate an action based on the
  public:
    static int actionCount;
    static map<ActionType, int> typeCount;

    Action(ActionType type, robot_standard_config config):type(type),config(config){typeCount[type]++;}
    ~Action(){typeCount[type]--;};
    /*
      Update the current config parameters.
      Parameter:
      config the action specific configuration

      return:
      false if the configuration was faulty. The previous parameters will be used
     */
    virtual bool updateConfig(robot_standard_config config);
    virtual void calWaypoints(cv::Point start);
    const robot_standard_config& getConfig(){return config;}
    time_sec getEstimatedDuration(){return estimatedDuration;};

  private:
    const ActionType type;
    time_sec estimatedDuration;
    waypoints wp;
    robot_standard_config config;

  };

  class AheadAction : public Action{

  public:
    // Possible are Ahead and CAhead -> both will do the same but differently interpreted
    AheadAction(ActionType type, robot_standard_config config):Action(type, config){}

  };

  class RotateAction : public Action {
  public:
    RotateAction(robot_standard_config config);
  };


  class ActionFactory{
  public:
    static robot_standard_config defaultConfig;
    ActionFactory(){}
    static Action createAction(ActionType type, robot_standard_config robotProperties=defaultConfig);
  };

  namespace path_mapping{

    // namespace to define all action to gridmap operations
  }
}


#endif // __PATH_TOOLS__
