#include <gtest/gtest.h>
// #include "ros/ros.h"
// #include "robot.hpp"
// #include "../src/environment/robot.hpp"
#include "../src/tools/path_tools.h"
#include <grid_map_cv/GridMapCvConverter.hpp>

using namespace path;
using namespace grid_map;


// TEST(HelperFunctions, vecToIdx){
//   Index idx = vecToIdx(direction(3.5,3.1));

//   cout << idx.x() << endl;
//   ASSERT_EQ(idx.x(), 4);
//   ASSERT_EQ(idx.y(), 3);
// }


TEST(HelperFunctions, updateConfig){
  PA_config conf1 = {{PAP::Angle, 40}};
  PA_config conf2 = {{PAP::Angle, 42}};

  updateConfig(conf1, conf2);
  ASSERT_EQ(conf1[PAP::Angle], 42);
}


// TEST(HelperFunctions, angleToDir){
//   cout << angleToDir(45) << endl;
//  }

// TEST(AheadActionTest, genWaypointsTest){
//   // Robot rob;

//   AheadAction aa(PAT::CAhead, {{PAP::Distance, 2.0}, {PAP::Angle, 45}});

//   WPsPtr wps = aa.generateWPs(grid_map::Index(1,1));



//   for(auto &wp : wps){
//     cout << wp->x() << " " << wp->y() << endl;
//   }
//   ASSERT_EQ(wps.size(), 2);
//   ASSERT_EQ(wps[1]->x(), 2);
// }

// TEST(GridMapTester, createGridMap){
//   grid_map::GridMap map({"obstacle", "map"});
//   map.setGeometry(Length(1,1), 0.01);


//   for(auto &lay : map.getLayers()){
//     cout << lay << endl;
//     cout << map.get(lay).rows() << map.get(lay).rows() << endl;
//   }
// }



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

  PAs actions;
  shared_ptr<GridMap> cmap;
  shared_ptr<Robot> rob;
};


TEST_F(RobotTest, startActionTest){
  /*
    - Check currentPos
    - clear Map
    - reset counter
   */
  Position pos;
  cmap->getPosition(Index(11,11), pos);
  StartAction sa(pos);
  bool res = rob->execute(sa, *cmap);

  ASSERT_TRUE(res);
  ASSERT_EQ(rob->get_currentPos(), pos);
  auto conf = rob->get_typeCount();
  for (auto &[k ,v] : conf){
    ASSERT_EQ(v, 0);
  }

  // displayGridmap();
}

TEST_F(RobotTest, aheadActionTest){
  Position pos;
  cmap->getPosition(Index(11,11),pos);
  StartAction sa(pos);
  // EndAction ea(WPs({Position(800, 800)}));
  AheadAction aa(PAT::CAhead, {{PAP::Angle, 90}, {PAP::Distance, 100}});

  bool res = rob->execute(sa, *cmap);
  res &= rob->execute(aa, *cmap);

  cout << "Map sum " << cmap->get("map").sum() << endl;
  ASSERT_TRUE(res);

  // for(LineIterator it(*cmap, Position(-1,1), Position(2,2)); !it.isPastEnd(); ++it){
  //   Position pos;
  //   cmap->getPosition(*it, pos);
  //   std::cout << "Pos: " << pos[0] << " " << pos[1] << "\n";
  //   std::cout << "Index: " << *it <<"\n";
  //   cmap->at("map", *it) = 1;

  // }
  // Position pos;
  // cmap->getPosition(Index(0,0), pos);
  // cout << "first: " << pos << endl;
  // cmap->getPosition(Index(cmap->getSize().x()-1,cmap->getSize().y()-1), pos);
  // cout << "last: " << pos << endl;
  // std::cout << "Size: " << cmap->getSize() << "\n";


  displayGridmap();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
