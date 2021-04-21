#include <gtest/gtest.h>
// #include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"
// #include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
// #include "../src/optimizer/grid_search.h"
#include "../src/optimizer/optimizer.h"
#include <sstream>
#include <yaml-cpp/yaml.h>


using namespace op;
using namespace path;
namespace fs = std::filesystem;
// using namespace gsearch;


class RobTest : public ::testing::Test{
protected:
  void SetUp() override {
    executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
    ostringstream msg;

    // Genpool pool;
    InitStrategy init;
    eConf.initIndividuals = 1;
    eConf.initActions = 4;
    init(pool, eConf);
    Robot rob(eConf.rob_conf,
	      eConf.gmap,
	      eConf.obstacleName);
    this->rob = make_shared<Robot>(rob);
    genome gen = pool.front();

    // rob.evaluateActions(gen.actions);

  }

  shared_ptr<Robot> rob;
  Genpool pool;
};

TEST_F(RobTest, initReset){

}

// TEST_F(RobTest, consecutiveReset){
//       for (int i=0; i<100; i++)
// 	rob->resetPAidx();
// }



// TEST(Crossover, idxCalculation){
//   executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
//   ostringstream msg;

//   Genpool pool;
//   InitStrategy init;
//   eConf.initIndividuals = 1;
//   eConf.initActions = 1;
//   init(pool, eConf);
//   Robot rob(eConf.rob_conf,
// 	    eConf.gmap,
// 	    eConf.obstacleName);

//   genome gen = pool.front();
//   int startAngle = 180;
//   gen.actions[1]->mod_config[PAP::Angle] = startAngle;

//   gen.actions.erase(next(gen.actions.begin(), 2));
//   float factor = 0.5;
//   for (int a=0; a <=90; a++){
//     msg << startAngle + a*factor << ",";
//     for(float i=0; i<50; i+=0.2){
//       gen.actions[1]->mod_config[PAP::Distance] = i;
//       gen.actions[1]->mod_config[PAP::Angle] = startAngle + a*factor;
//       gen.actions[1]->modified = true;
//       // gen.actions[1]->applyModifications();
//       rob.evaluateActions(gen.actions);
//       gen.updateGenParameter();
//       // debug("Start: ", gen.actions[1]->wps[0][0],"|",gen.actions[1]->wps[0][1]);
//       // debug("End: ", gen.actions[1]->wps[1][0],"|",gen.actions[1]->wps[1][1]);
//       // debug("Dist: ", gen.traveledDist, " Cross: ", gen.cross, " Pdist: ", gen.traveledDist * 0.3, " Actual: ", i, " error: ", i - gen.traveledDist * 0.3);
//       msg << i - gen.traveledDist * 0.3 << ",";
//       // cv::Mat img = rob.gridToImg("map");
//       // cv::imshow("Test", img);
//       // cv::waitKey(0.5);
//     }
//     msg << endl;
//   }

//   logging::Logger(msg.str(),"hello/", "angletest");
//   // cv::Mat img = rob.gridToImg("map");
//   // cv::imshow("Test", img);
//   // cv::waitKey();
// }


// TEST(Robot, intersectionTest){
//   executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
//   // grid_map::Polygon polygon;
//   // polygon.addVertex(Position(-1,-1));
//   // polygon.addVertex(Position(1,1));
//   Polygon polygon = Polygon::convexHullOfTwoCircles(Position(-1,-1), Position(1,1), eConf.Rob_width/2);


//   // polygon.thickenLine(0.5);

//   eConf.gmap->add("map", 0);
//   GridMap &map_ = *eConf.gmap;
//   for (grid_map::PolygonIterator iterator(*eConf.gmap, polygon);
//       !iterator.isPastEnd(); ++iterator) {
//     // debug(*iterator);
//     map_.at("map", *iterator) = 255.0;
//   }
//  // Robot rob(eConf.rob_conf,
//  // 	    eConf.gmap,
//  // 	    eConf.obstacleName);
// //  cv::Mat img = mapgen::gmapToImg(eConf.gmap, "map");
// //   cv::imshow("Test", img);
// //   cv::waitKey();
// }

void printPath(genome gen){
  ostringstream os;
  os << "\\def\\vvs{";
  Position first = gen.actions.front()->wps.front();
  auto it_next = next(gen.actions.begin(), 1);
  for (auto it = gen.actions.begin(); it != gen.actions.end() ; ++it) {
    os << (*it)->wps.front()[0] << "/" << (*it)->wps.front()[1] << "/" << (*it)->wps.back()[0] << "/" << (*it)->wps.back()[1] << "," << endl;
  }
  os << "}";
  cout << os.str() << endl;
}


void printTable(genome &gen, executionConfig &eConf, PolyRobot &rob, FitnessPoly & fit){
  rob.evaluateActions(gen.actions);
  fit.calculation(gen, rob.getFreeArea(), eConf);
  rob.evaluateActions(gen.actions);
  fit.calculation(gen, rob.getFreeArea(), eConf);
  cout << gen.p_obj << " & " << eConf.funSelect << " & " << eConf.Rob_angleSpeed << " & " << gen.fitness << " & " << gen.finalCoverage << " & " << gen.finalTime << " \\\\" << endl;
}


TEST(Initialization, boustrophedon) {
  executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
  // ostringstream msg;

  // Genpool pool;
  InitStrategy init;
  // eConf.initIndividuals = 1;
  // eConf.initActions = 1;
  // init(pool, eConf);
  genome gen;
  FitnessPoly fit;
  init.boustrophedon(gen, eConf);
  PolyRobot rob(eConf.rob_conf, eConf.gmap, eConf.obstacleName);
  validateGen(gen);
  rob.evaluateActions(gen.actions);
  // debug("Total: ", rob.getFreeArea());
  // eConf.funSelect = 0;
  // fit.calculation(gen, rob.getFreeArea(), eConf);
  // debug("Fit: ", eConf.funSelect," PathLen: ", gen.pathLengh, " cross ", gen.cross, " fit: ", gen.fitness, " time: ", gen.finalTime, " cov: ", gen.finalCoverage, " obj: ", gen.p_obj);
  // eConf.funSelect = 1;
  // fit.calculation(gen, rob.getFreeArea(), eConf);
  // debug("Fit: ", eConf.funSelect," PathLen: ", gen.pathLengh, " cross ", gen.cross, " fit: ", gen.fitness, " time: ", gen.finalTime, " cov: ", gen.finalCoverage, " obj: ", gen.p_obj);

  // debug("With obstacle");
  // mapgen::add_obstacle(eConf.gmap);
  // rob.evaluateActions(gen.actions);
  // debug("Total: ", rob.getFreeArea());
  // eConf.funSelect = 0;
  // fit.calculation(gen, rob.getFreeArea(), eConf);
  // debug("Fit: ", eConf.funSelect," PathLen: ", gen.pathLengh, " cross ", gen.cross, " fit: ", gen.fitness, " time: ", gen.finalTime, " cov: ", gen.finalCoverage, " obj: ", gen.p_obj);
  // eConf.funSelect = 1;
  // fit.calculation(gen, rob.getFreeArea(), eConf);
  // debug("Fit: ", eConf.funSelect," PathLen: ", gen.pathLengh, " cross ", gen.cross, " fit: ", gen.fitness, " time: ", gen.finalTime, " cov: ", gen.finalCoverage, " obj: ", gen.p_obj);


  float a_speed = eConf.Rob_angleSpeed;


  eConf.funSelect = 0;
  eConf.Rob_angleSpeed = 0;
  printTable(gen, eConf, rob, fit);

  eConf.funSelect = 0;
  eConf.Rob_angleSpeed = a_speed;
  printTable(gen, eConf, rob, fit);


  eConf.funSelect = 1;
  eConf.Rob_angleSpeed = 0;
  printTable(gen, eConf, rob, fit);

  eConf.funSelect = 1;
  eConf.Rob_angleSpeed = a_speed;
  printTable(gen, eConf, rob, fit);

  // debug("With obstacle");
  mapgen::add_obstacle(eConf.gmap);


  eConf.funSelect = 0;
  eConf.Rob_angleSpeed = 0;
  printTable(gen, eConf, rob, fit);

  eConf.funSelect = 0;
  eConf.Rob_angleSpeed = a_speed;
  printTable(gen, eConf, rob, fit);


  eConf.funSelect = 1;
  eConf.Rob_angleSpeed = 0;
  printTable(gen, eConf, rob, fit);

  eConf.funSelect = 1;
  eConf.Rob_angleSpeed = a_speed;
  printTable(gen, eConf, rob, fit);



  cv::Mat img = mapgen::gmapToImg(eConf.gmap, "map");
  img += mapgen::gmapToImg(eConf.gmap, "obstacle");
  cv::imshow("Test", img);
  cv::waitKey();

  printPath(gen);
}


TEST(Initialization, spiral) {
  executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
  // ostringstream msg;

  // Genpool pool;
  InitStrategy init;
  // eConf.initIndividuals = 1;
  // eConf.initActions = 1;
  // init(pool, eConf);
  genome gen;
  FitnessPoly fit;
  init.spiral(gen, eConf);
  PolyRobot rob(eConf.rob_conf, eConf.gmap, eConf.obstacleName);
  validateGen(gen);
  float a_speed = eConf.Rob_angleSpeed;


  eConf.funSelect = 0;
  eConf.Rob_angleSpeed = 0;
  printTable(gen, eConf, rob, fit);

  eConf.funSelect = 0;
  eConf.Rob_angleSpeed = a_speed;
  printTable(gen, eConf, rob, fit);


  eConf.funSelect = 1;
  eConf.Rob_angleSpeed = 0;
  printTable(gen, eConf, rob, fit);

  eConf.funSelect = 1;
  eConf.Rob_angleSpeed = a_speed;
  printTable(gen, eConf, rob, fit);

  // debug("With obstacle");
  mapgen::add_obstacle(eConf.gmap);


  eConf.funSelect = 0;
  eConf.Rob_angleSpeed = 0;
  printTable(gen, eConf, rob, fit);

  eConf.funSelect = 0;
  eConf.Rob_angleSpeed = a_speed;
  printTable(gen, eConf, rob, fit);


  eConf.funSelect = 1;
  eConf.Rob_angleSpeed = 0;
  printTable(gen, eConf, rob, fit);

  eConf.funSelect = 1;
  eConf.Rob_angleSpeed = a_speed;
  printTable(gen, eConf, rob, fit);



  cv::Mat img = mapgen::gmapToImg(eConf.gmap, "map");
  img += mapgen::gmapToImg(eConf.gmap, "obstacle");
  cv::imshow("Test", img);
  cv::waitKey();

  printPath(gen);
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
