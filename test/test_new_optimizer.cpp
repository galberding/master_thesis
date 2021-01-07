#include <gtest/gtest.h>
#include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"
#include <opencv2/opencv.hpp>
#include "../src/optimizer/grid_search.h"
#include "../src/optimizer/optimizer.h"
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


void loadConfigFromYaml(string path, executionConfig &config){

  YAML::Node conf = YAML::LoadFile(path);
  if(conf["weights"]){
    cout << conf["weights"] << endl;
  }
  config.maxIterations = conf["maxIterations"].as<int>();
  // Time, occ, configverage

  config.logName = conf["logName"].as<string>();
  config.logDir = conf["logDir"].as<string>();
  config.fitnessWeights[0] = conf["weights"]["time"].as<float>();
  config.fitnessWeights[1] = conf["weights"]["occ"].as<float>();
  config.fitnessWeights[2] = conf["weights"]["coverage"].as<float>();
  config.initActions = conf["initActions"].as<int>();
  config.initIndividuals = conf["initIndividuals"].as<float>();
  config.selectKeepBest = conf["keep"].as<int>();
  config.selectIndividuals = conf["select"].as<int>();
  config.crossoverProba = conf["crossoverProba"].as<float>();
  config.crossLength = conf["crossLength"].as<float>();
  config.mutaRandAngleProba = conf["mutaRandAngleProba"].as<float>();
  config.mutaOrtoAngleProba = conf["mutaOrtoAngleProba"].as<float>();
  config.mutaPosDistProba = conf["mutaPosDistProba"].as<float>();
  config.mutaNegDistProba = conf["mutaNegDistProba"].as<float>();
  config.mutaPosDistMax = conf["mutaPosDistMax"].as<float>();
}

TEST(YML, loadCong){
  gsearch::Searcher se;
  auto confs = se.generateConfigs("io_test");
  auto co = confs.front();
  loadConfigFromYaml("../../../src/ros_optimizer/test/config.yml", co);
}

TEST(Optimizer, standardConfig){
  gsearch::Searcher se;
  auto confs = se.generateConfigs("io_test");

  auto co = confs.front();
  loadConfigFromYaml("../../../src/ros_optimizer/test/config.yml", co);
  // co.maxIterations = 50;
  // // Time, occ, coverage
  // co.fitnessWeights = {0.05, 0.35, 0.6};
  // co.selectKeepBest = 100;
  // co.selectIndividuals = 1000;

  // co.mutaRandAngleProba = 0;


  GA_V2 ga(42, co);
  ga.optimizePath(true);

}

void printState(std::mt19937 &generator){
  uniform_real_distribution<float> dist(0.0,1);
  cout << "State: " << dist(generator) << endl;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
