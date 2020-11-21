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
  using namespace std;
  using time_sec = uint32_t;
  using standard_config = map<string, double>;
  using waypoints = vector<shared_ptr<cv::Point>>;

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

    Action(ActionType type, standard_config config):type(type),config(config){}


    /*
      Update the current config parameters.
      Parameter:
      config the action specific configuration

      return:
      false if the configuration was faulty. The previous parameters will be used
     */
    virtual bool updateConfig(standard_config config);
    virtual void calWaypoints(cv::Point start);
    const standard_config& getConfig(){return config;}
    time_sec getEstimatedDuration();
  private:
    const ActionType type;
    time_sec estimatedDuration;
    waypoints wp;
    standard_config config;

  };

  class AheadAction : public Action{

  public:
    // Possible are Ahead and CAhead -> both will do the same but differently interpreted
    AheadAction(ActionType type, standard_config config):Action(type, config){}
  };


  // class ActionFactory{
  // public:
  //   ActionFactory(ActionType type){
  //     switch(type){
  //     case ActionType::Ahead:

  // 	break;
  //     }
  //   }
  // }

  namespace path_mapping{

    // namespace to define all action to gridmap operations
  }
}


#endif // __PATH_TOOLS__
