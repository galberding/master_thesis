#include <gtest/gtest.h>
// #include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"
// #include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
// #include "../src/optimizer/grid_search.h"
#include "../src/optimizer/optimizer.h"
#include <yaml-cpp/yaml.h>


using namespace op;
using namespace path;
namespace fs = std::filesystem;
// using namespace gsearch;


// TEST(Crossover, idxCalculation){
//   executionConfig eConf("../../../src/ros_optimizer/test/config.yml");

//   eConf.crossLength = 0.0;
//   debug("Min gen len: ", eConf.getMinGenLen());
//   int s1 = 60;
//   int s2 = 60;
//   for (int j=0; j<9; j++){
//     eConf.crossLength += 0.1;
//     for(int i=0; i<10; i++){
//       int sIdx = getsIdx(s1, s2, eConf);
//       // debug(sIdx);
//       int len1 = getRemainingLen(sIdx, s1, eConf);
//       int len2 = getRemainingLen(sIdx, s2, eConf);
//       debug("SIdx: ", sIdx, " len1: ", len1, " len2: ", len2," -- " ,eConf.crossLength);
//     }
//   }

// }

TEST(Optimizer, standardConfig){

  executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
  // executionConfig eConf("conf_test.yml");
  // executionConfig eConf("/homes/galberding/catkin_ws/src/ros_optimizer/test/config.yml");
  debug("Start opti");
  op::Optimizer opti(
		     make_shared<op::InitStrategy>(op::InitStrategy()),
		     make_shared<op::SelectionStrategy>(op::SelectionStrategy()),
		     make_shared<op::DualPointCrossover>(op::DualPointCrossover()),
		     make_shared<op::MutationStrategy>(op::MutationStrategy()),
		     make_shared<op::FitnessStrategy>(op::FitnessStrategy()),
		     eConf);

  if(eConf.scenario == 0)  // elitist selection
    opti.optimizePath(eConf.printInfo);
  else
    opti.optimizePath_Turn_RWS(eConf.printInfo);



  if(eConf.retrain){
    mapgen::emulateCoveredMapSegment(opti.eConf.gmap, eConf.start);
    opti.eConf.maxIterations = eConf.retrain;
    if(eConf.scenario == 0)  // elitist selection
      opti.optimizePath(eConf.printInfo);
    else
      opti.optimizePath_Turn_RWS(eConf.printInfo);

  }

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
