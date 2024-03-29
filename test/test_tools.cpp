#include <gtest/gtest.h>
// #include "ros/ros.h"
// #include "robot.hpp"
// #include "../src/environment/robot.hpp"
#include "../src/tools/path_tools.h"
#include "../src/tools/mapGen.h"
#include <grid_map_cv/GridMapCvConverter.hpp>
#include "../src/tools/pa_serializer.h"
#include "../src/tools/genome_tools.h"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/iterators/LineIterator.hpp"


using namespace path;
using namespace grid_map;


// TEST(HelperFunctions, updateConfig){
//   PA_config conf1 = {{PAP::Angle, 40}};
//   PA_config conf2 = {{PAP::Angle, 42}};

//   updateConfig(conf1, conf2);
//   EXPECT_EQ(conf1[PAP::Angle], 42);
// }


TEST(HelperFunctions, copyAction){
  StartAction sa(Position(42,42));
  StartAction copy(sa);

  AheadAction aa(PAT::CAhead, {{PAP::Distance, 42}});
  AheadAction aa_copy(aa);

  EXPECT_EQ(aa.getConfig()[PAP::Distance], aa_copy.getConfig()[PAP::Distance]);

  EXPECT_EQ(sa.generateWPs(Position(0,0)), copy.generateWPs(Position(0,0)));
}


TEST(ActionTests, aheadActionWPgeneration){
  AheadAction aa(PAT::CAhead, {{PAP::Angle, 45}, {PAP::Distance, 10}});

  EXPECT_EQ(aa.generateWPs(Position(42,42)).size(), 2);
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
    rob = make_shared<Robot>(Robot({}, cmap, "map"));
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
  shared_ptr<StartAction> sa = make_shared<StartAction>(StartAction(pos));
  bool res = rob->execute(sa, cmap);
  EXPECT_TRUE(res);
  EXPECT_EQ(rob->get_currentPos(), pos);
  auto conf = rob->get_typeCount();
  for (auto &[k ,v] : conf){
    EXPECT_EQ(v, 0);
  }
// displayGridmap();
}


TEST_F(RobotTest, mapMoveBasicTest){
  Position pos0, pos1;
  cmap->getPosition(Index(11,11),pos0);
  cmap->getPosition(Index(33,33),pos1);
  shared_ptr<AheadAction> pa = make_shared<AheadAction>(AheadAction(PAT::CAhead, {{PAP::Angle, 0}, {PAP::Distance, 10}}));
  pa->set_wps(vector<Position>({pos0, pos1}));
  int steps;
  WPs path;
  bool res = rob->mapMove(cmap, pa, steps, rob->get_currentPos(), path, true);

  EXPECT_GT(steps, 0);
  EXPECT_GT(cmap->get("map").sum(), 0);
  EXPECT_TRUE(res);

  // displayGridmap();
}


TEST_F(RobotTest, executionAheadActionTest){
  Position pos;
  cmap->getPosition(Index(11,11),pos);
  shared_ptr<StartAction> sa = make_shared<StartAction>(StartAction(pos));
  // EndAction ea(WPs({Position(800, 800)}));
  shared_ptr<AheadAction> aa = make_shared<AheadAction>(AheadAction(PAT::CAhead, {{PAP::Angle, 180}, {PAP::Distance, 100}}));
  bool res = rob->execute(sa, cmap);
  res &= rob->execute(aa, cmap);

  EXPECT_GT(cmap->get("map").sum(), 0);
  EXPECT_TRUE(res);


  // displayGridmap();
}

TEST_F(RobotTest, executionTest){
  Position pos0, pos1;
  EXPECT_TRUE(cmap->getPosition(Index(100,100),pos0));
  cmap->getPosition(Index(33,33),pos1);
  // PathAction pa(PAT::CAhead);
  StartAction sa(pos0);
  AheadAction aa1(PAT::CAhead, PA_config({{PAP::Angle ,0}, {PAP::Distance, 10}}));
  AheadAction aa2(PAT::CAhead, PA_config({{PAP::Angle ,90}, {PAP::Distance, 10}}));
  AheadAction aa3(PAT::CAhead, PA_config({{PAP::Angle ,180}, {PAP::Distance, 10}}));
  AheadAction aa4(PAT::CAhead, PA_config({{PAP::Angle ,270}, {PAP::Distance, 10}}));

  auto sap = make_shared<StartAction>(sa);
  auto aa1p = make_shared<AheadAction>(aa1);
  auto aa2p = make_shared<AheadAction>(aa2);
  auto aa3p = make_shared<AheadAction>(aa3);
  auto aa4p = make_shared<AheadAction>(aa4);

  bool res = rob->execute(sap, cmap);
  EXPECT_TRUE(res);
  res &= rob->execute(aa1p, cmap);
  EXPECT_TRUE(res);
  res &= rob->execute(aa2p, cmap);
  EXPECT_TRUE(res);
  res &= rob->execute(aa3p, cmap);
  EXPECT_TRUE(res);
  res &= rob->execute(aa4p, cmap);

  // displayGridmap();
  EXPECT_TRUE(res);
  EXPECT_GT(cmap->get("map").sum(), 0);
}

TEST_F(RobotTest, evaluateActionTest){
  Position pos0, pos1;
  EXPECT_TRUE(cmap->getPosition(Index(100,100),pos0));
  cmap->getPosition(Index(33,33),pos1);
  PathAction pa(PAT::CAhead);
  StartAction sa(pos0);
  AheadAction aa1(PAT::CAhead, PA_config({{PAP::Angle ,0}, {PAP::Distance, 5}}));
  AheadAction aa2(PAT::CAhead, PA_config({{PAP::Angle ,90}, {PAP::Distance, 10}}));
  AheadAction aa3(PAT::CAhead, PA_config({{PAP::Angle ,180}, {PAP::Distance, 10}}));
  AheadAction aa4(PAT::CAhead, PA_config({{PAP::Angle ,270}, {PAP::Distance, 50}}));

  PAs actions2;
  actions2.push_back(make_shared<StartAction>(sa));
  actions2.push_back(make_shared<AheadAction>(aa1));
  actions2.push_back(make_shared<AheadAction>(aa2));
  actions2.push_back(make_shared<AheadAction>(aa3));
  actions2.push_back(make_shared<AheadAction>(aa4));

  bool res = rob->evaluateActions(actions2);
  EXPECT_TRUE(res);

  auto conf = rob->get_typeCount();
  EXPECT_EQ(conf[PAT::Start], 1);
  EXPECT_EQ(conf[PAT::CAhead], 4);

  // displayImage(rob->gridToImg("map"));

}


TEST_F(RobotTest, evaluateActionTestCollision){
  // Adapt action to not hit the obstacle
  Position pos0, pos1;
  EXPECT_TRUE(cmap->getPosition(Index(100,100),pos0));
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

  EXPECT_TRUE(res);
  EXPECT_EQ(actions2.size(), 5);
  // EXPECT_EQ(actions2.back()->mod_config[PAP::Distance], 7350);

  // displayImage(rob->gridToImg("map"));

}

TEST_F(RobotTest, evaluateActionTestPointOutOfMapBounds){
  // Remove failed action from action list
  Position pos0, pos1;
  EXPECT_TRUE(cmap->getPosition(Index(100,100),pos0));
  cmap->getPosition(Index(33,33),pos1);
  PathAction pa(PAT::CAhead);
  StartAction sa(pos0);
  EndAction ea({pos0});
  AheadAction aa1(PAT::CAhead, PA_config({{PAP::Angle ,0}, {PAP::Distance, 500}}));
  AheadAction aa2(PAT::CAhead, PA_config({{PAP::Angle ,90}, {PAP::Distance, 1000}}));
  AheadAction aa3(PAT::CAhead, PA_config({{PAP::Angle ,180}, {PAP::Distance, 1000}}));
  AheadAction aa4(PAT::CAhead, PA_config({{PAP::Angle ,270}, {PAP::Distance, 100500}}));
  AheadAction aa5(PAT::CAhead, PA_config({{PAP::Angle ,270}, {PAP::Distance, 100500}}));

  PAs actions2;
  actions2.push_back(make_shared<StartAction>(sa));
  actions2.push_back(make_shared<AheadAction>(aa1));
  actions2.push_back(make_shared<AheadAction>(aa2));
  actions2.push_back(make_shared<AheadAction>(aa3));
  actions2.push_back(make_shared<AheadAction>(aa4));
  actions2.push_back(make_shared<AheadAction>(aa5));
  actions2.push_back(make_shared<EndAction>(ea));


  bool res = rob->evaluateActions(actions2);
  // EXPECT_DEATH(rob->evaluateActions(actions>)

  EXPECT_TRUE(res);

  EXPECT_EQ(actions2.size(), 7);
  // Last PA will be aa3
  // EXPECT_EQ(*actions2.back(), aa3);

  // displayImage(rob->gridToImg("map"));

}


TEST_F(RobotTest, evaluateActionTestTooFewActions){
  // When all action except start and end remain in action list, evaluation action
  // is supposed to return false to ultimately delete the action sequence / genome
  Position pos0, pos1;
  EXPECT_TRUE(cmap->getPosition(Index(100,100),pos0));
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

  ASSERT_TRUE(rob->evaluateActions(actions2));
  // bool res = rob->evaluateActions(actions2);

  // EXPECT_TRUE(!res);
  // EXPECT_EQ(actions2.size(), 2);
  // Last PA will be aa3
  // EXPECT_EQ(*actions2.back(), aa3);

  // displayImage(rob->gridToImg("map"));

}

TEST_F(RobotTest, mapBoundsTest){
  Position pos;
  // debug(cmap->getSize().x());
  // debug(cmap->getPosition(Index(cmap->getSize().x()-1, cmap->getSize().y()-1), pos));
  // debug(pos);
  // debug(cmap->getPosition(Index(0, 0), pos));
  // debug(pos);

}


TEST(ActionTest, EndActionMendingTest){
  Position start(42,42), end(42,42);

  // debug("Start: ", start);
  // debug("End: ", end);
  // debug("Distance: ", start - end);

  StartAction sa(start);
  EndAction ea({end});
  AheadAction aa(PAT::CAhead, {{PAP::Distance, 2000}, {PAP::Angle, 270}});
  AheadAction aa1(PAT::CAhead, {{PAP::Distance, 30}, {PAP::Angle, 130}});
  AheadAction aa2(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 90}});
  AheadAction aa3(PAT::CAhead, {{PAP::Distance, 10}, {PAP::Angle, 0}});


  PAs act = {
      make_shared<StartAction>(sa),
      make_shared<AheadAction>(aa),
      make_shared<AheadAction>(aa1),
      make_shared<AheadAction>(aa2),
      make_shared<AheadAction>(aa3),
      make_shared<EndAction>(ea)
  };

  // debug("Execute Mending on end action");
  act.back()->mendConfig(act.front());

  auto sta = act.begin();
  auto nxt = next(sta, 1);
  (*nxt)->modified = true;
  (*nxt)->generateWPs((*sta)->wps.front());
  while(nxt != act.end()){
    // debug("Mend: ", int((*nxt)->type));
    (*nxt)->mendConfig(*sta);
    sta = nxt;
    nxt++;
    if(nxt == act.end()) break;
    (*nxt)->generateWPs((*sta)->wps.front());
    // debug("Done!");
  }
}


TEST(ActionCopy, copytest){
  AheadAction aa(PAT::CAhead, {{PAP::Distance, 42}});
  aa.generateWPs(Position(42,42));
  EXPECT_EQ(aa.wps.size(), 2);
  AheadAction aa_copy(aa);

  EXPECT_EQ(aa_copy.wps.size(), 2);
  EXPECT_EQ(aa_copy.wps.front(), aa.wps.front());
  EXPECT_TRUE(!aa_copy.modified);

}


TEST(Serializer, writePoolToFile){
  Position start(42,42), end(42,42);

  StartAction sa(start);
  EndAction ea({end});
  AheadAction aa(PAT::CAhead, {{PAP::Distance, 2000}, {PAP::Angle, 270}});
  AheadAction aa1(PAT::CAhead, {{PAP::Distance, 30}, {PAP::Angle, 130}});
  AheadAction aa2(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 90}});
  AheadAction aa3(PAT::CAhead, {{PAP::Distance, 10}, {PAP::Angle, 0}});


  PAs act = {
      make_shared<StartAction>(sa),
      make_shared<AheadAction>(aa),
      make_shared<AheadAction>(aa1),
      make_shared<AheadAction>(aa2),
      make_shared<AheadAction>(aa3),
      make_shared<EndAction>(ea)
  };

  for (auto it = act.begin(); it != act.end(); ++it) {
    if((*it)->type == PAT::Start){
      (*it)->generateWPs(start);
    } else {
      (*it)->generateWPs((*prev(it,1))->wps.back());
    }
  }

  vector<PAs> pps = {act};

  EXPECT_TRUE(pa_serializer::writeActionsToFile(pps, "testSerialize"));

}

TEST(Serializer, readPoolFromFile){
  vector<PAs> pps;

  pa_serializer::readActrionsFromFile(pps, "testSerialize");
  pa_serializer::writeActionsToFile(pps, "testReSer");
}

TEST(MapGen, changeResolution){
  Position start;
  shared_ptr<GridMap> map = mapgen::generateMapType(10, 10, 0.1, 0.1, 1, start);
  EXPECT_FLOAT_EQ(mapgen::changeMapRes(map, 0.3)->getResolution(), 0.3);
}

TEST(MapGen, drawPath){
  Position start, pos0, pos1, pos2, pos3;
  cv::Mat rob_map, old;
  shared_ptr<GridMap> map = mapgen::generateMapType(10, 10, 0.4, 0.4, 1, start);


  map->getPosition(Index(1,1), pos0);
  map->getPosition(Index(5,5), pos1);
  map->getPosition(Index(10,10), pos2);
  vector<Position> pp ={
    Position(0,0),
    Position(2,3),
    Position(5,5)
  };
  map->add("map", 0);
  // GridMap gg = *map;

  mapgen::drawPathOnMap(map, pp, true);


  shared_ptr<GridMap> newMap = mapgen::changeMapRes(map, 0.01);
  GridMapCvConverter::toImage<unsigned char, 1>(*map, "map", CV_8U, 0.0, 1, old);
  // cv::imshow("Path", old);
  // cv::waitKey();

  mapgen::drawPathOnMap(newMap, pp, false);

  GridMapCvConverter::toImage<unsigned char, 1>(*newMap, "map", CV_8U, 0.0, 1, rob_map);

  // cv::imshow("Path", rob_map);
  // cv::waitKey();


}


TEST(MapGen, inspectMapType2){
  // TODO: Check if start position is correct
  // Vis results
  Position start, pos0, pos1, pos2, pos3;
  shared_ptr<GridMap> map = mapgen::generateMapType(10, 10, 0.1, 0.3, 2, start);


  // Visualize results
  cv::Mat bb = mapgen::gmapToImg(map, "obstacle");
  cv::imshow("Path", bb);
  // cv::waitKey();

}

TEST(MapGen, inspectMapType1){
  // TODO: Check if start position is correct
  // Vis results
  Position start, pos0, pos1, pos2, pos3;
  shared_ptr<GridMap> map = mapgen::generateMapType(11, 11, 0.1, 0.3, 1, start);


  // Visualize results
  cv::Mat bb = mapgen::gmapToImg(map, "obstacle");
  cv::imshow("Path", bb);
  // cv::waitKey();

}


void genIdx(shared_ptr<GridMap> map, Position start, Position end){


  for(LineIterator li(*map, start, end); !li.isPastEnd(); ++li){
    Position pos;
    // debug((*li).x()," ", (*li).y());
    map->getPosition(*li, pos);
    // debug("Pos: ", pos[0], " ", pos[1]);
    cout << pos[0] << "/" << pos[1] << endl;
  }
  cout << endl;
}


void printWall(shared_ptr<GridMap> map){

  for(GridMapIterator it(*map); !it.isPastEnd(); ++it){
    if(map->at("obstacle",*it)==1){
      Position pos;
      map->getPosition(*it, pos);
      cout << pos(0) << "/" << pos(1) << ",";
    }
  }
  cout << endl;
}

void genIdxPoly(shared_ptr<GridMap> map, Position start, Position end){
  Polygon poly;
  poly.addVertex(start);
  poly.addVertex(end);
  poly.thickenLine(0.3);
  for (grid_map::PolygonIterator it(*map, poly);
       !it.isPastEnd(); ++it) {
    Position pos;
    // debug((*li).x()," ", (*li).y());
    map->getPosition(*it, pos);
    cout << pos[0] << "/" << pos[1] << ",";
  }
}

TEST(MapGen, generatIdx){
  Position start, pos0, pos1, pos2, pos3;
  shared_ptr<GridMap> map = mapgen::generateMapType(10, 10, 0.5, 0.3, 0, start);
  // map.setPosition(Position())
  // genIdx(map, Position(4.75, 4.74), Position(1.25, 3.25));
  // genIdx(map, Position(1.25, 3.25), Position(1.25, 1.25));

  genIdx(map, Position(0.5, 4.5), Position(0.5, 0.5));
  genIdx(map, Position(0.5, 0.5), Position(4.25, 2.25));
  genIdx(map, Position(4.25, 2.25), Position(4.25, 4.25));
  genIdx(map, Position(4.25, 4.25), Position(1.25, 3.25));
  genIdx(map, Position(1.25, 3.25), Position(1.25, 3.2));

  // Visualize results


  // cv::Mat bb = mapgen::gmapToImg(map, "obstacle");
  // cv::imshow("Path", bb);
  // cv::waitKey();
}

TEST(MapGen, generatIdxPoly){
  Position start, pos0, pos1, pos2, pos3;
  shared_ptr<GridMap> map = mapgen::generateMapType(10, 10, 0.25, 0.3, 0, start);
  // map.setPosition(Position())
  // genIdx(map, Position(4.75, 4.74), Position(1.25, 3.25));
  // genIdx(map, Position(1.25, 3.25), Position(1.25, 1.25));
  debug("Poly Index");
  cout << "\\def\\points{";
  genIdxPoly(map, Position(0.5, 4.5), Position(0.5, 0.5));
  genIdxPoly(map, Position(0.5, 0.5), Position(4.25, 2.25));
  genIdxPoly(map, Position(4.25, 2.25), Position(4.25, 4.25));
  genIdxPoly(map, Position(4.25, 4.25), Position(1.25, 3.25));
  genIdxPoly(map, Position(1.25, 3.25), Position(1.25, 3.2));

  // Visualize results


  // cv::Mat bb = mapgen::gmapToImg(map, "obstacle");
  // cv::imshow("Path", bb);
  // cv::waitKey();

}

TEST(MapGen, generatWallIdx){
  Position start, pos0, pos1, pos2, pos3;
  shared_ptr<GridMap> map = mapgen::generateMapType(5, 5, 0.25, 0.1, 1, start);
  cout << "\\def\\wall{";
  printWall(map);
  // map.setPosition(Position())
  // genIdx(map, Position(4.75, 4.74), Position(1.25, 3.25));
  // genIdx(map, Position(1.25, 3.25), Position(1.25, 1.25));
  // debug("Poly Index");
  // cout << "\\def\\points{";
  // genIdxPoly(map, Position(0.5, 4.5), Position(0.5, 0.5));
  // genIdxPoly(map, Position(0.5, 0.5), Position(4.25, 2.25));
  // genIdxPoly(map, Position(4.25, 2.25), Position(4.25, 4.25));
  // genIdxPoly(map, Position(4.25, 4.25), Position(1.25, 3.25));
  // genIdxPoly(map, Position(1.25, 3.25), Position(1.25, 3.2));
}


TEST(MapGen, testMapCoverMtype1){
  Position start, pos0, pos1, pos2, pos3;
  shared_ptr<GridMap> map = mapgen::generateMapType(11, 11, 0.1, 0.3, 2, start);
  mapgen::emulateCoveredMapSegment(map, start);
  cv::Mat bb = mapgen::gmapToImg(map, "covered");
  bb += mapgen::gmapToImg(map, "obstacle");
  cv::imshow("Path", bb);
  cv::waitKey();
  // EXPECT_TRUE(mapgen::emulateCoveredMapSegment(map, start));
}

TEST(MapGen, testMapCoverMtype2){
  Position start, pos0, pos1, pos2, pos3;
  shared_ptr<GridMap> map = mapgen::generateMapType(11, 11, 0.1, 0.3, 1, start);
  mapgen::emulateCoveredMapSegment(map, start);

  cv::Mat bb = mapgen::gmapToImg(map, "covered");
  bb += mapgen::gmapToImg(map, "obstacle");
  cv::imshow("Path", bb);
  cv::waitKey();
  // EXPECT_TRUE(mapgen::emulateCoveredMapSegment(map, start));

}

TEST(GenTools, testErase){
  Position start(42,42), end(42,42);

  StartAction sa(start);
  EndAction ea({end});
  AheadAction aa(PAT::CAhead, {{PAP::Distance, 2000}, {PAP::Angle, 270}});
  AheadAction aa1(PAT::CAhead, {{PAP::Distance, 0}, {PAP::Angle, 130}});
  AheadAction aa2(PAT::CAhead, {{PAP::Distance, 0}, {PAP::Angle, 90}});
  AheadAction aa3(PAT::CAhead, {{PAP::Distance, 0}, {PAP::Angle, 0}});
  AheadAction aa4(PAT::CAhead, {{PAP::Distance, 0}, {PAP::Angle, 0}});
  AheadAction aa5(PAT::CAhead, {{PAP::Distance, 0}, {PAP::Angle, 0}});
  AheadAction aa6(PAT::CAhead, {{PAP::Distance, 10}, {PAP::Angle, 0}});
  AheadAction aa7(PAT::CAhead, {{PAP::Distance, 10}, {PAP::Angle, 0}});

  PAs act = {make_shared<StartAction>(sa)};

  vector<float> dists = {0,0,0,0,0,0,0,1,0};
  for(auto f : dists){
    AheadAction aa(PAT::CAhead, {{PAP::Distance, f}, {PAP::Angle, 2}});
    act.push_back(make_shared<AheadAction>(aa));
  }
  act.push_back(make_shared<EndAction>(ea));

  // PAs act = {
  //     make_shared<StartAction>(sa),
  //     make_shared<AheadAction>(aa),
  //     make_shared<AheadAction>(aa1),
  //     make_shared<AheadAction>(aa2),
  //     make_shared<AheadAction>(aa3),
  //     make_shared<AheadAction>(aa4),
  //     make_shared<AheadAction>(aa5),
  //     make_shared<AheadAction>(aa6),
  //     make_shared<AheadAction>(aa7),
  //     make_shared<EndAction>(ea)
  // };

  genome_tools::genome gen(act);
  // debug("Zero Actions: ", genome_tools::calZeroActionPercent(gen));
  genome_tools::removeZeroPAs(gen, 0);
  // genome_tools::genome gen(act);
  // debug("Zero Actions: ", genome_tools::calZeroActionPercent(gen));
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
