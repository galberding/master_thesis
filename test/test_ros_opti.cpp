#include <gtest/gtest.h>
// #include "ros/ros.h"
#include "robot.hpp"

TEST(MathExpressions, badInput){
  Robot rob();
  ASSERT_EQ(1, rob.hello());
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
