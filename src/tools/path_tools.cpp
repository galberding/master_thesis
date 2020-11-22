#include "path_tools.h"

using namespace path;

// Initialize the counter
map<ActionType, int> Action::typeCount = {{ActionType::Ahead, 0}, {ActionType::CAhead, 0}, {ActionType::Rotate, 0}};

// Prepare the standard config
robot_standard_config ActionFactory::defaultConfig = {
    {RobotProperty::Width_cm, 1},
    {RobotProperty::Height_cm, 1},
    {RobotProperty::Drive_speed_cm_s, 50},
    {RobotProperty::Clean_speed_cm_s, 20}};



Action ActionFactory::createAction(ActionType type, robot_standard_config robotProperties){
  // Update the default config


  switch(type){
  case ActionType::Ahead:
    return AheadAction(ActionType::Ahead, defaultConfig);
    break;
  case ActionType::CAhead:
    return AheadAction(ActionType::CAhead, defaultConfig);
    break;
  case ActionType::Rotate:
    return RotateAction(defaultConfig);
    break;
  default:
    cout << "Unknown Action insert exception here!!" << endl;
    throw __LINE__;

  }
}
