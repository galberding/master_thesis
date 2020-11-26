#include <gtest/gtest.h>
// #include "ros/ros.h"
// #include "robot.hpp"
// #include "../src/environment/robot.hpp"
#include "../src/tools/path_tools.h"

using namespace path;
using namespace grid_map;


TEST(HelperFunctions, vecToIdx){
  Index idx = vecToIdx(direction(3.5,3.1));

  cout << idx.x() << endl;
  ASSERT_EQ(idx.x(), 4);
  ASSERT_EQ(idx.y(), 3);
}


TEST(HelperFunctions, updateConfig){
  PA_config conf1 = {{PAP::Angle, 40}};
  PA_config conf2 = {{PAP::Angle, 42}};

  updateConfig(conf1, conf2);
  ASSERT_EQ(conf1[PAP::Angle], 42);
}


TEST(HelperFunctions, angleToDir){
  cout << angleToDir(45) << endl;
}

TEST(AheadActionTest, genWaypointsTest){
  // Robot rob;

  AheadAction aa(PAT::CAhead, {{PAP::Distance, 2.0}, {PAP::Angle, 45}});

  WPs wps = aa.generateWPs(grid_map::Index(1,1));



  for(auto &wp : wps){
    cout << wp->x() << " " << wp->y() << endl;
  }
  ASSERT_EQ(wps.size(), 2);
  ASSERT_EQ(wps[1]->x(), 2);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
