#include <gtest/gtest.h>
#include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"
#include <opencv2/opencv.hpp>
#include "../src/optimizer/grid_search.h"
#include <yaml-cpp/yaml.h>


using namespace ga;
using namespace path;
namespace fs = std::filesystem;
// using namespace gsearch;

void displayGridmap(shared_ptr<GridMap> cmap){
    cv::Mat obstacle, rob_map;

    GridMapCvConverter::toImage<unsigned char, 1>(*cmap, "obstacle", CV_8U, 0.0, 255, obstacle);
    // GridMapCvConverter::toImage<unsigned char, 1>(*cmap, "map", CV_8U, 0.0, 1, rob_map);

    // cv::imwrite("testio.jpg", obstacle);
    cv::imshow("Grid Map Obstacle", obstacle);
    // cv::imshow("Grid Map Map", rob_map);
    // cv::moveWindow("Grid Map Map", 1000, 0);
    cv::waitKey();
  }

TEST(Map, generation){
  gsearch::Searcher se;
  Position start;
  auto mapptr = se.generateMapType(100, 50, 0.2, 1, start);

  // displayGridmap(mapptr);
}

TEST(Try, argsToString){
  debug("Test string generation: ");
  cout << argsToCsv(1, 3, 4.6);
}

// TEST(GridSearch, createConfigs){

//   gsearch::Searcher se;
//   auto confs = se.generateConfigs();
//   for(auto conf : confs){
//     cout << conf.config_to_string() <<endl;
//   }
//   debug("Totally generated: ", confs.size(), " configs.");

// }


// TEST(GridSearch, search){
//   gsearch::Searcher se;
//   auto confs = se.generateConfigs("blub2");
//   // se.tSearchV2();
//   auto co = *next(confs.begin(), 4);

//   // co.initActions = 20;
//   GA_V2 ga(42, co);
//   ga.optimizePath(true);
// }

// TEST(GridSearch, search){
//   gsearch::Searcher se;
//   // se.tSearchV2("v2_mutation_selectBest_timeWeightedByCoverage");
//   se.search("test1");
// }

void loadYaml(string path, executionConfig &config){

  YAML::Node conf = YAML::LoadFile(path);
  if(conf["weights"]){
    cout << conf["weights"] << endl;
  }
}

TEST(YML, loadCong){
  gsearch::Searcher se;
  auto confs = se.generateConfigs("io_test");
  auto co = confs.front();
  loadYaml("../../../src/ros_optimizer/test/config.yml", co);
}

TEST(Optimizer, standardConfig){
  gsearch::Searcher se;
  auto confs = se.generateConfigs("io_test");

  auto co = confs.front();
  co.maxIterations = 10000;
  // Time, occ, coverage
  co.fitnessWeights = {0.05, 0.35, 0.6};
  co.selectKeepBest = 10;
  co.selectIndividuals = 500;

  co.mutaRandAngleProba = 0;

  GA_V2 ga(42, co);
  ga.optimizePath(true);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
