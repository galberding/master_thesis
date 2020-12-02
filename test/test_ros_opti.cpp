#include <gtest/gtest.h>
// #include "ros/ros.h"
// #include "robot.hpp"
// #include "../src/environment/robot.hpp"
#include "../src/tools/path_tools.h"
#include <grid_map_cv/GridMapCvConverter.hpp>

using namespace path;
using namespace grid_map;


TEST(HelperFunctions, updateConfig){
  PA_config conf1 = {{PAP::Angle, 40}};
  PA_config conf2 = {{PAP::Angle, 42}};

  updateConfig(conf1, conf2);
  ASSERT_EQ(conf1[PAP::Angle], 42);
}



TEST(ActionTests, aheadActionWPgeneration){
  AheadAction aa(PAT::CAhead, {{PAP::Angle, 45}, {PAP::Distance, 10}});

  WPs wps = aa.generateWPs(Position(1,1));
  for(auto wp : wps){
    std::cout << "Waypont: " << wp << "\n";

  }
}

class RobotTest : public ::testing::Test{
protected:
  void SetUp() override {
    grid_map::GridMap map({"obstacle", "map"});
    map.setGeometry(Length(100,100), 0.30);
    map.add("obstacle", 255.0);
    map.add("map", 0.0);

    for (SubmapIterator sm(map, Index(10, 10), Size(map.getSize().x() - 10, map.getSize().y() - 20)); !sm.isPastEnd();
	 ++sm) {
    // if (!map.isInside(sm.getSubmapIndex())){
      // cout << "outside the map: " << sm.getSubmapIndex().x() << " " << sm.getSubmapIndex().y() << endl;
      // continue;
    // }
      map.at("obstacle", *sm) = 0.0;
  }


    cmap = make_shared<GridMap>(map);
    rob = make_shared<Robot>(Robot(42, {}, map));
    StartAction sa(Position(10,10));
    EndAction ea(WPs({Position(800, 800)}));
    AheadAction aa(PAT::CAhead, {{PAP::Angle, 45}, {PAP::Distance, 10}});

    actions.push_back(make_shared<PathAction>(sa));
    actions.push_back(make_shared<PathAction>(aa));
    actions.push_back(make_shared<PathAction>(ea));

  }

  void displayGridmap(){
    cv::Mat obstacle, rob_map;

    GridMapCvConverter::toImage<unsigned char, 1>(*cmap, "obstacle", CV_8U, 0.0, 255, obstacle);
    GridMapCvConverter::toImage<unsigned char, 1>(*cmap, "map", CV_8U, 0.0, 1, rob_map);

    cv::imshow("Grid Map Obstacle", obstacle);
    cv::imshow("Grid Map Map", rob_map);
    cv::moveWindow("Grid Map Map", 1000, 0);
    cv::waitKey();
  }

  void displayImage(cv::Mat img){
    cv::imshow("Grid Map Map", img);
    cv::waitKey();
  }

  PAs actions;
  shared_ptr<GridMap> cmap;
  shared_ptr<Robot> rob;
};


// TEST_F(RobotTest, startActionTest){
//   /*
//     - Check currentPos
//     - clear Map
//     - reset counter
//    */
//   Position pos;
//   cmap->getPosition(Index(11,11), pos);
//   StartAction sa(pos);
//   bool res = rob->execute(sa, *cmap);

//   ASSERT_TRUE(res);
//   ASSERT_EQ(rob->get_currentPos(), pos);
//   auto conf = rob->get_typeCount();
//   for (auto &[k ,v] : conf){
//     ASSERT_EQ(v, 0);
//   }


//   // displayGridmap();
// }


TEST_F(RobotTest, mapMoveBasicTest){
  Position pos0, pos1;
  cmap->getPosition(Index(11,11),pos0);
  cmap->getPosition(Index(33,33),pos1);
  PathAction pa(PAT::CAhead);
  pa.set_wps(vector<Position>({pos0, pos1}));
  int steps;
  WPs path;
  bool res = rob->mapMove(*cmap, pa, steps, rob->get_currentPos(), path, true);


  cout << "Current Sum " << cmap->get("map").sum() << endl;
  cout << steps << endl;
  ASSERT_TRUE(res);

  // displayGridmap();
}


TEST_F(RobotTest, executionTest){
  Position pos0, pos1;
  ASSERT_TRUE(cmap->getPosition(Index(100,100),pos0));
  cmap->getPosition(Index(33,33),pos1);
  PathAction pa(PAT::CAhead);
  StartAction sa(pos0);
  AheadAction aa1(PAT::CAhead, PA_config({{PAP::Angle ,0}, {PAP::Distance, 500}}));
  AheadAction aa2(PAT::CAhead, PA_config({{PAP::Angle ,90}, {PAP::Distance, 1000}}));
  AheadAction aa3(PAT::CAhead, PA_config({{PAP::Angle ,180}, {PAP::Distance, 1000}}));
  AheadAction aa4(PAT::CAhead, PA_config({{PAP::Angle ,270}, {PAP::Distance, 1000}}));

  bool res = rob->execute(sa, *cmap);
  res &= rob->execute(aa1, *cmap);
  res &= rob->execute(aa2, *cmap);
  res &= rob->execute(aa3, *cmap);
  res &= rob->execute(aa4, *cmap);

  displayGridmap();
  ASSERT_TRUE(res);
}


// TEST_F(RobotTest, aheadActionTest){
//   Position pos;
//   cmap->getPosition(Index(11,11),pos);
//   StartAction sa(pos);
//   // EndAction ea(WPs({Position(800, 800)}));
//   AheadAction aa(PAT::CAhead, {{PAP::Angle, 90}, {PAP::Distance, 100}});

//   bool res = rob->execute(sa, *cmap);
//   res &= rob->execute(aa, *cmap);

//   cout << "Map sum " << cmap->get("map").sum() << endl;
//   ASSERT_TRUE(res);


//   // displayGridmap();
// }

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
