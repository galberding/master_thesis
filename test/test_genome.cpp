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


TEST(Crossover, idxCalculation){
  executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
  ostringstream msg;

  Genpool pool;
  InitStrategy init;
  eConf.initIndividuals = 1;
  eConf.initActions = 1;
  init(pool, eConf);
  Robot rob(eConf.rob_conf,
	    eConf.gmap,
	    eConf.obstacleName);

  genome gen = pool.front();
  int startAngle = 180;
  gen.actions[1]->mod_config[PAP::Angle] = startAngle;

  gen.actions.erase(next(gen.actions.begin(), 2));
  float factor = 0.5;
  for (int a=0; a <=90; a++){
    msg << startAngle + a*factor << ",";
    for(float i=0; i<50; i+=0.2){
      gen.actions[1]->mod_config[PAP::Distance] = i;
      gen.actions[1]->mod_config[PAP::Angle] = startAngle + a*factor;
      gen.actions[1]->modified = true;
      // gen.actions[1]->applyModifications();
      rob.evaluateActions(gen.actions);
      gen.updateGenParameter();
      // debug("Start: ", gen.actions[1]->wps[0][0],"|",gen.actions[1]->wps[0][1]);
      // debug("End: ", gen.actions[1]->wps[1][0],"|",gen.actions[1]->wps[1][1]);
      // debug("Dist: ", gen.traveledDist, " Cross: ", gen.cross, " Pdist: ", gen.traveledDist * 0.3, " Actual: ", i, " error: ", i - gen.traveledDist * 0.3);
      msg << i - gen.traveledDist * 0.3 << ",";
      // cv::Mat img = rob.gridToImg("map");
      // cv::imshow("Test", img);
      // cv::waitKey(0.5);
    }
    msg << endl;
  }

  logging::Logger(msg.str(),"hello/", "angletest");
  // cv::Mat img = rob.gridToImg("map");
  // cv::imshow("Test", img);
  // cv::waitKey();
}



int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
