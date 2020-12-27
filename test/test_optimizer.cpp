#include <gtest/gtest.h>
#include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"
#include <opencv2/opencv.hpp>

using namespace ga;
using namespace path;

class GATest : public ::testing::Test{
protected:
  void SetUp() override {

    Mutation_conf muta = {
      {"addAction", make_pair(addAction, 10)},
      {"removeAction", make_pair(removeAction, 10)},
      {"addAngleOffset", make_pair(addAngleOffset, 90)},
      {"addDistanceOffset", make_pair(addDistanceOffset, 50)},
      // {"swapRandomAction", make_pair(swapRandomAction, 10)},
    };
    ga = make_shared<GA>(GA(42, 4, 0.5, 0, 50, muta));
    grid_map::GridMap map({"obstacle", "map"});
    map.setGeometry(Length(100,100), 0.30);
    map.add("obstacle", 1.0);
    map.add("map", 0.0);

    for (SubmapIterator sm(map, Index(10, 10), Size(map.getSize().x() - 10, map.getSize().y() - 30)); !sm.isPastEnd();
	 ++sm) {
      map.at("obstacle", *sm) = 0.0;
    }


    cmap = make_shared<GridMap>(map);
    rob = make_shared<Robot>(Robot({}, cmap, "map"));
  }


  void displayImage(cv::Mat img){
    cv::imshow("Grid Map Map", img);
    cv::waitKey();
  }

  void displayGridmap(){
    cv::Mat obstacle, rob_map;

    GridMapCvConverter::toImage<unsigned char, 1>(*cmap, "obstacle", CV_8U, 0.0, 255, obstacle);
    GridMapCvConverter::toImage<unsigned char, 1>(*cmap, "map", CV_8U, 0.0, 1, rob_map);

    // cv::imshow("Grid Map Obstacle", obstacle);
    cv::imshow("Grid Map Map", rob_map);
    // cv::moveWindow("Grid Map Map", 1000, 0);
    cv::waitKey();
  }

  // PAs actions;
  shared_ptr<GridMap> cmap;
  shared_ptr<Robot> rob;
  shared_ptr<GA> ga;
};

TEST(HelperTest, dirToAngleConversionTest){
  direction dir(20,20);
  auto dist = dir.norm();
  debug("Distance: ", dist);
  debug("Distance: ", (dir/dist).norm());

  debug("Angle: ", dirToAngle(dir / dist));

}

TEST(HelperTest, dirToAngleConversionDirToSmallTest){
  direction dir(0.25,0.25);
  auto dist = dir.norm();
  debug("Distance: ", dist);
  debug("Distance: ", (dir/dist).norm());

  debug("Angle: ", dirToAngle(dir / dist));
}

TEST(HelperTest, dirToAngleConversionZeroTest){
  direction dir(0,0);
  auto dist = dir.norm();
  debug("Distance: ", dist);
  debug("Distance: ", (dir/dist).norm());

  float angle = dirToAngle(dir / dist);
  debug("Angle: ", angle);
  EXPECT_TRUE(isnan(angle));
}


TEST_F(GATest, initPopulationTest){
  Genpool pool;
  Genpool selection_pool;
  Position pos, end1, end2;

  warn(cmap->getPosition(Index(100,110),pos));
  debug("Generated start pos: ", pos);
  cmap->getPosition(Index(30,22),end1);
  cmap->getPosition(Index(40,22),end2);
  ga->populatePool(pool, pos, WPs{end1, end2}, 100, 300);

  // EXPECT_EQ(pool.size(), 12);
  // EXPECT_EQ(pool.front().actions.size(), 14);
  EXPECT_TRUE(rob->evaluateActions(pool.front().actions));
}


TEST_F(GATest, evalFitnessTest){
  Genpool pool;
  Genpool selection_pool;
  Position pos, end1, end2;

  warn(cmap->getPosition(Index(100,110),pos));
  debug("Generated start pos: ", pos);
  cmap->getPosition(Index(30,22),end1);
  cmap->getPosition(Index(40,22),end2);
  ga->populatePool(pool, pos, WPs{end1, end2}, 10, 100);

  // EXPECT_EQ(pool.size(), 12);
  // EXPECT_EQ(pool.front().actions.size(), 14);
  EXPECT_TRUE(rob->evaluateActions(pool.front().actions));
  ga->evalFitness(pool, *rob);
  for(auto gen : pool){
    EXPECT_GT(gen.fitness, 0);
    EXPECT_GE(gen.actions.size(), 100);
  }
}

TEST_F(GATest, crossoverMatingTest){
  // Create two gens that are obviously different
  // mate them
  // show the and their children
  Genpool pool, newPop;
  Genpool selection_pool;
  Position pos, end1, end2, start(11,11);

  debug("Map Size x: ", cmap->getSize()(0), " y: ", cmap->getSize()(1));
  ASSERT_TRUE(cmap->getPosition(Index(100,110),pos));
  // cmap->getPosition(Index(100,110),end1);

  debug("Generated start pos: ", pos);
  cmap->getPosition(Index(30,22),end1);
  cmap->getPosition(Index(40,22),end2);
  int childs = 2;
  ga->populatePool(pool, pos, WPs{end1, end2}, childs, 10);

  rob->evaluateActions(pool.front().actions);
  cv::Mat gen1 = rob->gridToImg("map");
  rob->evaluateActions(pool.back().actions);
  cv::Mat gen2 = rob->gridToImg("map");

  EXPECT_EQ(pool.size(), childs);
  // EXPECT_NE(&pool.front(), &pool.back());

  ga->mating(pool.front(), pool.back(), newPop);
  EXPECT_EQ(newPop.size(), 2);

  rob->evaluateActions(newPop.front().actions);
  cv::Mat child1 = rob->gridToImg("map");
  rob->evaluateActions(newPop.back().actions);
  cv::Mat child2 = rob->gridToImg("map");
  // ga->mating(pool.front(), pool.back(), newPop);

  // cv::imshow("gen1", gen1);
  // cv::imshow("gen2", gen2);
  // cv::imshow("child1", child1);
  // cv::imshow("child2", child2);
  // cv::moveWindow("gen1", 0, 0);
  // cv::moveWindow("gen2", 500, 0);
  // cv::moveWindow("child2", 500, 500);
  // cv::moveWindow("child1", 0, 500);

  // cv::waitKey();

}


TEST_F(GATest, actionModifivationTest){
   Position start, end;

  cmap->getPosition(Index(11,11), start);
  cmap->getPosition(Index(22,22), end);
  debug("Start: ", start);
  debug("End: ", end);
  debug("Distance: ", start - end);

  StartAction sa(start);
  EndAction ea({end});
  AheadAction aa(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 290}});
  AheadAction aa1(PAT::CAhead, {{PAP::Distance, 30}, {PAP::Angle, 180}});
  AheadAction aa2(PAT::CAhead, {{PAP::Distance, 2000}, {PAP::Angle, 270}});
  AheadAction aa3(PAT::CAhead, {{PAP::Distance, 2000}, {PAP::Angle, 270}});
  AheadAction aa4(PAT::CAhead, {{PAP::Distance, 10}, {PAP::Angle, 0}});
  AheadAction aa5(PAT::CAhead, {{PAP::Distance, 2000}, {PAP::Angle, 270}});


  PAs act;
  act.push_back(make_shared<StartAction>(sa));
  act.push_back(make_shared<AheadAction>(aa));
  act.push_back(make_shared<AheadAction>(aa1));
  act.push_back(make_shared<AheadAction>(aa2));
  act.push_back(make_shared<AheadAction>(aa3));
  act.push_back(make_shared<AheadAction>(aa4));
  act.push_back(make_shared<AheadAction>(aa5));
  act.push_back(make_shared<EndAction>(ea));

  genome gen(act);
  validateGen(gen);

  rob->evaluateActions(gen.actions);

  // displayImage(rob->gridToImg("map"));

  next(gen.actions.begin(), 3)->get()->mod_config[PAP::Angle] += 90;
  next(gen.actions.begin(), 3)->get()->mod_config[PAP::Angle] += 90;
  next(gen.actions.begin(), 3)->get()->mod_config[PAP::Distance] += 1000;
  // next(gen.actions.begin(), 3)->get()->modified = true;
  next(gen.actions.begin(), 3)->get()->applyModifications();
  next(gen.actions.begin(), 4)->get()->mendConfig(*(next(gen.actions.begin(), 3)));

  rob->evaluateActions(gen.actions);

  // displayImage(rob->gridToImg("map"));


}

TEST_F(GATest, sortingTest){

  Genpool pool;
  Genpool selection_pool;
  Position pos, end1, end2;

  debug("Generated start pos: ", pos);
  cmap->getPosition(Index(30,22),end1);
  cmap->getPosition(Index(40,22),end2);

  ga->populatePool(pool, pos, WPs{end1, end2}, 100, 300);
  ga->evalFitness(pool, *rob);
  sort(pool.begin(), pool.end(), compareFitness);
  for (auto gen : pool){
    EXPECT_GE(pool.back().fitness, gen.fitness);
  }

}

TEST_F(GATest, selectionTest){

  Genpool pool;
  Genpool selection_pool;
  Position pos, end1, end2;

  debug("Generated start pos: ", pos);
  cmap->getPosition(Index(30,22),pos);
  cmap->getPosition(Index(40,22),end2);

  ga->populatePool(pool, pos, WPs{end1, end2}, 100, 300);
  ga->evalFitness(pool, *rob);
  ga->selection(pool, selection_pool, 3);

  debug(selection_pool.front().fitness);
  debug(selection_pool.back().fitness);
  EXPECT_EQ(selection_pool.size(), 3);
}

TEST_F(GATest, calculateFreeArea){
  // debug("Size: ", (cmap->get("obstacle").sum()));
  // debug("Size: ", cmap->getSize().x()*cmap->getSize().y() - (cmap->get("obstacle").sum()));
  EXPECT_EQ(cmap->getSize().x()*cmap->getSize().y() - (cmap->get("obstacle").sum()), rob->getFreeArea());
}

TEST_F(GATest, randTest){

  list<int> l1, l2, c1, c2;

  l1.push_back(1);
  l1.push_back(2);
  l1.push_back(3);

  l2.push_back(4);
  l2.push_back(5);
  l2.push_back(6);

  c1.insert(c1.begin(), l1.begin(), std::next(l1.begin(), 2));
  c1.insert(c1.end(),std::next(l2.begin(), 2), l2.end());

  for(auto i : c1){
    debug("Value: ", i);
  }


}


TEST_F(GATest, mutationConfigTest){
  Genpool pool;
  ga->populatePool(pool, Position(42,42), {Position(42,42)}, 2, 4);

  genome gen1 = pool.front();
  genome gen2 = pool.back();
  rob->evaluateActions(gen1.actions);
  rob->evaluateActions(gen2.actions);

  auto size = gen1.actions.size();
  addAction(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
  EXPECT_EQ(gen1.actions.size(), size + 1);

  removeAction(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
  EXPECT_EQ(gen1.actions.size(), size);

  auto action = *next(gen1.actions.begin(), 1);

  auto dOffset = action->mod_config[PAP::DistanceOffset];
  debug("DistOff: ", dOffset);
  addDistanceOffset(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
  EXPECT_NE(action->mod_config[PAP::DistanceOffset], dOffset);

  auto aOffset = action->mod_config[PAP::AngleOffset];
  debug("AngleOff: ", aOffset);
  addAngleOffset(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
  EXPECT_NE(action->mod_config[PAP::AngleOffset], aOffset);

}


TEST_F(GATest, adaptationTest){
  Position start, end;

  cmap->getPosition(Index(41,41), start);
  cmap->getPosition(Index(22,22), end);
  debug("Start: ", start);
  debug("End: ", end);
  debug("Distance: ", start - end);

  StartAction sa(start);
  EndAction ea({end});
  AheadAction aa(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 270}});
  AheadAction aa1(PAT::CAhead, {{PAP::Distance, 30}, {PAP::Angle, 180}});
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

  genome gen(act);
  // addAngleOffset(gen, ga->distanceDistr, ga->angleDistr, ga->generator);
  rob->evaluateActions(gen.actions);
  addAction(gen, ga->distanceDistr, ga->angleDistr, ga->generator);
  validateGen(gen);
  rob->evaluateActions(gen.actions);

  // displayImage(rob->gridToImg("map"));


}



TEST_F(GATest, boundaryMendingTest){
  Position start, end;

  cmap->getPosition(Index(41,41), start);
  cmap->getPosition(Index(22,22), end);
  debug("Start: ", start);
  debug("End: ", end);
  debug("Distance: ", start - end);

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

  genome gen(act);
  debug("1New size: ", gen.actions.size());
  // addAngleOffset(gen, ga->distanceDistr, ga->angleDistr, ga->generator);
  rob->evaluateActions(gen.actions);
  // addAction(gen, ga->distanceDistr, ga->angleDistr, ga->generator);
  // validatldeGen(gen);
  // displayImage(rob->gridToImg("map"));
  rob->evaluateActions(gen.actions);
  rob->evaluateActions(gen.actions);

  debug("2New size: ", gen.actions.size());

  debug("Dist ", next(gen.actions.begin(), 1)->get()->mod_config[PAP::Distance]);
  debug("Dist ", next(gen.actions.begin(), 2)->get()->mod_config[PAP::Distance]);

  // displayImage(rob->gridToImg("map"));


}


TEST_F(GATest, basicApplicationTest){
  Genpool pool, sel, newPop;
  Position start, end;
  Mutation_conf muta = {
      {"addAction", make_pair(addAction, 100)},
      // {"removeAction", make_pair(removeAction, 100)},
      {"addAngleOffset", make_pair(addAngleOffset, 100)},
      {"addDistanceOffset", make_pair(addDistanceOffset, 100)},
      // {"swapRandomAction", make_pair(swapRandomAction, 100)},
    };
  cmap->getPosition(Index(100,100), start);
  cmap->getPosition(Index(300,300), end);
  debug("Start");
  ga->populatePool(pool, start, {end}, 10, 300);
  EXPECT_EQ(pool.size(), 10);
  for(auto gen : pool){
    EXPECT_GT(gen.actions.size(), 30);
  }
  ga->evalFitness(pool, *rob);
  EXPECT_GT(pool.size(), 0);
  debug("Select: ", pool.size());
  ga->selection(pool, sel, 5);
  for(auto gen : sel){
    debug("Check");
    EXPECT_GT(gen.actions.size(), 30);
  }
  EXPECT_EQ(sel.size(), 5);
  debug("Cross");
  pool.clear();
  ga->crossover(sel, pool);
  EXPECT_GT(pool.size(), 6);
  for(auto gen : pool){
    EXPECT_GT(gen.actions.size(), 2);
  }

  debug("Mutate");
  ga->mutation(pool, muta);
  ga->evalFitness(pool, *rob);
  debug("Done");

  // displayImage(rob->gridToImg("map"));

}



class GAApplication : public ::testing::Test {
protected:
  void SetUp() override {

    Mutation_conf muta = {
      // {"addAction", make_pair(addAction, 60)},
      // {"removeAction", make_pair(removeAction, 10)},
      {"addAngleOffset", make_pair(addAngleOffset, 50)},
      {"addDistanceOffset", make_pair(addDistanceOffset, 50)},
      // {"swapRandomAction", make_pair(swapRandomAction, 10)},
    };
    ga = make_shared<GA>(GA(42, 4, 0.7, 0, 70, muta));
    grid_map::GridMap map({"obstacle", "map"});
    map.setGeometry(Length(100,100), 0.30);
    map.add("obstacle", 1.0);
    map.add("map", 0.0);

    for (SubmapIterator sm(map, Index(100, 100), Size(map.getSize().x() - 100, map.getSize().y() - 10)); !sm.isPastEnd();
	 ++sm) {
      map.at("obstacle", *sm) = 0.0;
    }

    cmap = make_shared<GridMap>(map);
    // rob = make_shared<Robot>(Robot(42, {}, map));
    rob = make_shared<Robot>(Robot({}, cmap, "map"));
    // displayImage(rob->gridToImg("obstacle"));
  }


  void displayImage(cv::Mat img){
    cv::imshow("Grid Map Map", img);
    cv::waitKey();
  }

  // PAs actions;
  shared_ptr<GridMap> cmap;
  shared_ptr<Robot> rob;
  shared_ptr<GA> ga;
};



// TEST_F(GAApplication, mutationTest){

//   Position start, end;

//   cmap->getPosition(Index(41,41), start);
//   cmap->getPosition(Index(22,22), end);
//   debug("Start: ", start);
//   debug("End: ", end);
//   debug("Distance: ", start - end);

//   StartAction sa(start);
//   EndAction ea({end});
//   AheadAction aa(PAT::CAhead, {{PAP::Distance, 2000}, {PAP::Angle, 270}});
//   AheadAction aa1(PAT::CAhead, {{PAP::Distance, 30}, {PAP::Angle, 130}});
//   AheadAction aa2(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 90}});
//   AheadAction aa3(PAT::CAhead, {{PAP::Distance, 10}, {PAP::Angle, 0}});


//   PAs act = {
//       make_shared<StartAction>(sa),
//       make_shared<AheadAction>(aa),
//       make_shared<AheadAction>(aa1),
//       make_shared<AheadAction>(aa2),
//       make_shared<AheadAction>(aa3),
//       make_shared<EndAction>(ea)
//   };

//   genome gen(act);

//   int size = act.size();
//   int iter = 4;

//   rob->evaluateActions(gen.actions);

//   for(int i=0; i<iter;i++){
//     addAction(gen,  ga->angleDistr,  ga->distanceDistr,  ga->generator);
//     validateGen(gen);
//     rob->evaluateActions(gen.actions);
//     // displayImage(rob->gridToImg("map"));
//   }

//   EXPECT_EQ(gen.actions.size(), size + iter);

//   for(int i=0; i<iter;i++){
//     removeAction(gen,  ga->angleDistr,  ga->distanceDistr,  ga->generator);
//     validateGen(gen);
//     rob->evaluateActions(gen.actions);
//     // displayImage(rob->gridToImg("map"));
//   }
//   EXPECT_EQ(gen.actions.size(), size );
// }




TEST(SortingTest, sortStuff){

  struct Action{
    Action(int blub):blub(blub){};
    int blub;
  };

  using PAs = vector<shared_ptr<Action>>;

  struct genomeS{
    genomeS(){};
    genomeS(float fitness):fitness(fitness){};
    genomeS(PAs actions):actions(actions){};
    genomeS(PAs actions, float fitness):actions(actions), fitness(fitness){};
    bool operator < (const genomeS& gen) const
    {
        return (fitness > gen.fitness);
    }

    int id = 0;
    PAs actions;
    WPs waypoints;
    float fitness = 0;
  };
  deque<genomeS> pool;

  for(int i=0; i<10; i++){
    PAs actions;
    for (int j=0; j < 10; j++) {

      actions.push_back(make_shared<Action>(Action(j)));
    }
    pool.push_back(genomeS(actions, static_cast<float>(0)));
  }

  for(auto gen : pool){
    debug("Current size: ", gen.actions.size());
    assert(("Gen has no actions -- before!", gen.actions.size() > 0));
  }
  sort(pool.begin(), pool.end());
  for(auto gen : pool){
    debug("Current size: ", gen.actions.size());
    assert(("Gen has no actions -- after!", gen.actions.size() > 0));
  }


}


TEST(SortingTest, sortStuffdeleteGenes){

  struct Action{
    Action(int blub):blub(blub){};
    int blub;
  };

  using PAs = vector<shared_ptr<Action>>;

  struct genomeS{
    genomeS(){};
    genomeS(float fitness):fitness(fitness){};
    genomeS(PAs actions):actions(actions){};
    genomeS(PAs actions, float fitness):actions(actions), fitness(fitness){};
    bool operator < (const genomeS& gen) const
    {
        return (fitness > gen.fitness);
    }

    int id = 0;
    PAs actions;
    WPs waypoints;
    float fitness = 0;
  };
  deque<genomeS> pool;

  for(int i=0; i<10; i++){
    PAs actions;
    for (int j=0; j < 10; j++) {

      actions.push_back(make_shared<Action>(Action(j)));
    }
    pool.push_back(genomeS(actions, static_cast<float>(0)));
  }

  deque<genomeS> keep;

  keep.insert(keep.begin(), pool.begin(), next(pool.begin(), 4));

  pool.clear();

  pool.insert(pool.begin(), keep.begin(), keep.end());

  for(auto &gen : pool){
    debug("Copied: ", gen.actions.size());
    ASSERT_GT(gen.actions.size(), 0);
  }
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
