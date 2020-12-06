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


TEST(HelperFunctions, compyAction){
  StartAction sa(Position(42,42));
  StartAction copy(sa);

  AheadAction aa(PAT::CAhead, {{PAP::Distance, 42}});
  AheadAction aa_copy(aa);

  ASSERT_EQ(aa.getConfig()[PAP::Distance], aa_copy.getConfig()[PAP::Distance]);

  ASSERT_EQ(sa.generateWPs(Position(0,0)), copy.generateWPs(Position(0,0)));
}


TEST(ActionTests, aheadActionWPgeneration){
  AheadAction aa(PAT::CAhead, {{PAP::Angle, 45}, {PAP::Distance, 10}});

  ASSERT_EQ(aa.generateWPs(Position(42,42)).size(), 2);
}

class RobotTest : public ::testing::Test{
protected:
  void SetUp() override {
    grid_map::GridMap map({"obstacle", "map"});
    map.setGeometry(Length(100,100), 0.30);
    map.add("obstacle", 255.0);
    map.add("map", 0.0);

    for (SubmapIterator sm(map, Index(10, 10), Size(map.getSize().x() - 10, map.getSize().y() - 30)); !sm.isPastEnd();
	 ++sm) {

      map.at("obstacle", *sm) = 0.0;
  }


    cmap = make_shared<GridMap>(map);
    rob = make_shared<Robot>(Robot(42, {}, map));
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

  // PAs actions;
  shared_ptr<GridMap> cmap;
  shared_ptr<Robot> rob;
};


TEST_F(RobotTest, executionStartActionTest){
  /*
    - Check currentPos
    - clear Map
    - reset counter
   */
  Position pos;
  cmap->getPosition(Index(11,11), pos);
  StartAction sa(pos);
  bool res = rob->execute(make_shared<StartAction>(sa), *cmap);
  ASSERT_TRUE(res);
  ASSERT_EQ(rob->get_currentPos(), pos);
  auto conf = rob->get_typeCount();
  for (auto &[k ,v] : conf){
    ASSERT_EQ(v, 0);
  }


//   // displayGridmap();
}


TEST_F(RobotTest, mapMoveBasicTest){
  Position pos0, pos1;
  cmap->getPosition(Index(11,11),pos0);
  cmap->getPosition(Index(33,33),pos1);
  PathAction pa(PAT::CAhead);
  pa.set_wps(vector<Position>({pos0, pos1}));
  int steps;
  WPs path;
  bool res = rob->mapMove(*cmap, make_shared<PathAction>(pa), steps, rob->get_currentPos(), path, true);

  ASSERT_GT(steps, 0);
  ASSERT_GT(cmap->get("map").sum(), 0);
  ASSERT_TRUE(res);

  // displayGridmap();
}


TEST_F(RobotTest, executionAheadActionTest){
  Position pos;
  cmap->getPosition(Index(11,11),pos);
  StartAction sa(pos);
  // EndAction ea(WPs({Position(800, 800)}));
  AheadAction aa(PAT::CAhead, {{PAP::Angle, 180}, {PAP::Distance, 100}});

  bool res = rob->execute(make_shared<StartAction>(sa), *cmap);
  res &= rob->execute(make_shared<AheadAction>(aa), *cmap);

  ASSERT_GT(cmap->get("map").sum(), 0);
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

  auto sap = make_shared<StartAction>(sa);
  auto aa1p = make_shared<AheadAction>(aa1);
  auto aa2p = make_shared<AheadAction>(aa2);
  auto aa3p = make_shared<AheadAction>(aa3);
  auto aa4p = make_shared<AheadAction>(aa4);

  bool res = rob->execute(sap, *cmap);
  res &= rob->execute(aa1p, *cmap);
  res &= rob->execute(aa2p, *cmap);
  res &= rob->execute(aa3p, *cmap);
  res &= rob->execute(aa4p, *cmap);

  // displayGridmap();
  ASSERT_TRUE(res);
  ASSERT_GT(cmap->get("map").sum(), 0);
}

TEST_F(RobotTest, evaluateActionTest){
  Position pos0, pos1;
  ASSERT_TRUE(cmap->getPosition(Index(100,100),pos0));
  cmap->getPosition(Index(33,33),pos1);
  PathAction pa(PAT::CAhead);
  StartAction sa(pos0);
  AheadAction aa1(PAT::CAhead, PA_config({{PAP::Angle ,0}, {PAP::Distance, 500}}));
  AheadAction aa2(PAT::CAhead, PA_config({{PAP::Angle ,90}, {PAP::Distance, 1000}}));
  AheadAction aa3(PAT::CAhead, PA_config({{PAP::Angle ,180}, {PAP::Distance, 1000}}));
  AheadAction aa4(PAT::CAhead, PA_config({{PAP::Angle ,270}, {PAP::Distance, 5000}}));

  PAs actions2;
  actions2.push_back(make_shared<StartAction>(sa));
  actions2.push_back(make_shared<AheadAction>(aa1));
  actions2.push_back(make_shared<AheadAction>(aa2));
  actions2.push_back(make_shared<AheadAction>(aa3));
  actions2.push_back(make_shared<AheadAction>(aa4));

  bool res = rob->evaluateActions(actions2);
  ASSERT_TRUE(res);

  auto conf = rob->get_typeCount();
  ASSERT_EQ(conf[PAT::Start], 1);
  ASSERT_EQ(conf[PAT::CAhead], 4);

  // displayImage(rob->gridToImg("map"));

}


TEST_F(RobotTest, evaluateActionTestCollision){
  // Adapt action to not hit the obstacle
  Position pos0, pos1;
  ASSERT_TRUE(cmap->getPosition(Index(100,100),pos0));
  cmap->getPosition(Index(33,33),pos1);
  PathAction pa(PAT::CAhead);
  StartAction sa(pos0);
  AheadAction aa1(PAT::CAhead, PA_config({{PAP::Angle ,0}, {PAP::Distance, 500}}));
  AheadAction aa2(PAT::CAhead, PA_config({{PAP::Angle ,90}, {PAP::Distance, 1000}}));
  AheadAction aa3(PAT::CAhead, PA_config({{PAP::Angle ,180}, {PAP::Distance, 1000}}));
  AheadAction aa4(PAT::CAhead, PA_config({{PAP::Angle ,270}, {PAP::Distance, 7500}}));

  PAs actions2;
  actions2.push_back(make_shared<StartAction>(sa));
  actions2.push_back(make_shared<AheadAction>(aa1));
  actions2.push_back(make_shared<AheadAction>(aa2));
  actions2.push_back(make_shared<AheadAction>(aa3));
  actions2.push_back(make_shared<AheadAction>(aa4));


  bool res = rob->evaluateActions(actions2);

  ASSERT_TRUE(res);
  ASSERT_EQ(actions2.size(), 5);
  ASSERT_EQ(actions2.back()->getConfig()[PAP::Distance], 7350);

  // displayImage(rob->gridToImg("map"));

}

TEST_F(RobotTest, evaluateActionTestPointOutOfMapBounds){
  // Remove failed action from action list
  Position pos0, pos1;
  ASSERT_TRUE(cmap->getPosition(Index(100,100),pos0));
  cmap->getPosition(Index(33,33),pos1);
  PathAction pa(PAT::CAhead);
  StartAction sa(pos0);
  AheadAction aa1(PAT::CAhead, PA_config({{PAP::Angle ,0}, {PAP::Distance, 500}}));
  AheadAction aa2(PAT::CAhead, PA_config({{PAP::Angle ,90}, {PAP::Distance, 1000}}));
  AheadAction aa3(PAT::CAhead, PA_config({{PAP::Angle ,180}, {PAP::Distance, 1000}}));
  AheadAction aa4(PAT::CAhead, PA_config({{PAP::Angle ,270}, {PAP::Distance, 100500}}));

  PAs actions2;
  actions2.push_back(make_shared<StartAction>(sa));
  actions2.push_back(make_shared<AheadAction>(aa1));
  actions2.push_back(make_shared<AheadAction>(aa2));
  actions2.push_back(make_shared<AheadAction>(aa3));
  actions2.push_back(make_shared<AheadAction>(aa4));

  bool res = rob->evaluateActions(actions2);

  ASSERT_TRUE(res);
  ASSERT_EQ(actions2.size(), 4);
  // Last PA will be aa3
  // ASSERT_EQ(*actions2.back(), aa3);

  // displayImage(rob->gridToImg("map"));

}


TEST_F(RobotTest, evaluateActionTestTooFewActions){
  // When all action except start and end remain in action list, evaluation action
  // is supposed to return false to ultimately delete the action sequence / genome
  Position pos0, pos1;
  ASSERT_TRUE(cmap->getPosition(Index(100,100),pos0));
  cmap->getPosition(Index(33,33),pos1);
  PathAction pa(PAT::CAhead);
  StartAction sa(pos0);
  EndAction ea(WPs({Position(42,42)}));
  // AheadAction aa1(PAT::CAhead, PA_config({{PAP::Angle ,0}, {PAP::Distance, 500}}));
  // AheadAction aa2(PAT::CAhead, PA_config({{PAP::Angle ,90}, {PAP::Distance, 1000}}));
  // AheadAction aa3(PAT::CAhead, PA_config({{PAP::Angle ,180}, {PAP::Distance, 1000}}));
  // AheadAction aa4(PAT::CAhead, PA_config({{PAP::Angle ,270}, {PAP::Distance, 100500}}));

  PAs actions2;
  actions2.push_back(make_shared<StartAction>(sa));
  actions2.push_back(make_shared<EndAction>(ea));
  // actions2.push_back(make_shared<AheadAction>(aa1));
  // actions2.push_back(make_shared<AheadAction>(aa2));
  // actions2.push_back(make_shared<AheadAction>(aa3));
  // actions2.push_back(make_shared<AheadAction>(aa4));

  bool res = rob->evaluateActions(actions2);

  ASSERT_TRUE(!res);
  ASSERT_EQ(actions2.size(), 2);
  // Last PA will be aa3
  // ASSERT_EQ(*actions2.back(), aa3);

  // displayImage(rob->gridToImg("map"));

}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
