#include <gtest/gtest.h>
// #include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"
#include <opencv2/opencv.hpp>
// #include "../src/optimizer/grid_search.h"
#include "../src/optimizer/optimizer.h"
#include <yaml-cpp/yaml.h>


// using namespace ga;
using namespace path;
namespace fs = std::filesystem;
// using namespace gsearch;


TEST(Optimizer, standardConfig){

  executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
  // executionConfig eConf("/homes/galberding/catkin_ws/src/ros_optimizer/test/config.yml");
  debug("Start opti");
  op::Optimizer opti(
		     make_shared<op::InitStrategy>(op::InitStrategy()),
		     make_shared<op::SelectionStrategy>(op::SelectionStrategy()),
		     make_shared<op::DualPointCrossover>(op::DualPointCrossover()),
		     make_shared<op::MutationStrategy>(op::MutationStrategy()),
		     make_shared<op::FitnessStrategy>(op::FitnessStrategy()),
		     eConf);
  debug("Init");

  if(eConf.scenario == 0)  // elitist selection
    opti.optimizePath(eConf.printInfo);
  else if(eConf.scenario == 1) // tournament selection
    opti.optimizePath_s_tourn_c_dp(eConf.printInfo);
  else if(eConf.scenario == 2) // roulette selection
    opti.optimizePath_s_roulette_c_dp(eConf.printInfo);
  else
    warn("No valid scenario selected!");


  // opti.optimizePath(true);


}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
