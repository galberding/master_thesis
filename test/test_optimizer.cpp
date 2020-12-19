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
    rob = make_shared<Robot>(Robot(42, {}, map));
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
  ASSERT_TRUE(isnan(angle));
}


// TEST_F(GATest, initPopulationTest){
//   Genpool pool;
//   Genpool selection_pool;
//   Position pos, end1, end2;

//   warn(cmap->getPosition(Index(100,110),pos));
//   debug("Generated start pos: ", pos);
//   cmap->getPosition(Index(30,22),end1);
//   cmap->getPosition(Index(40,22),end2);
//   ga->populatePool(pool, pos, WPs{end1, end2}, 100, 300);

//   // ASSERT_EQ(pool.size(), 12);
//   // ASSERT_EQ(pool.front().actions.size(), 14);
//   ASSERT_TRUE(rob->evaluateActions(pool.front().actions));
// }


TEST_F(GATest, evalFitnessTest){
  Genpool pool;
  Genpool selection_pool;
  Position pos, end1, end2;

  warn(cmap->getPosition(Index(100,110),pos));
  debug("Generated start pos: ", pos);
  cmap->getPosition(Index(30,22),end1);
  cmap->getPosition(Index(40,22),end2);
  ga->populatePool(pool, pos, WPs{end1, end2}, 10, 100);

  // ASSERT_EQ(pool.size(), 12);
  // ASSERT_EQ(pool.front().actions.size(), 14);
  // ASSERT_TRUE(rob->evaluateActions(pool.front().actions));
  ga->evalFitness(pool, *rob);
  for(auto gen : pool){
    ASSERT_GT(gen.fitness, 0);
    ASSERT_GE(gen.actions.size(), 100);
  }
}

TEST_F(GATest, crossoverMatingTest){
  // Create two gens that are obviously different
  // mate them
  // show the and their children
  Genpool pool, newPop;
  Genpool selection_pool;
  Position pos, end1, end2, start;
  cmap->getPosition(Index(100,110),start);
  cmap->getPosition(Index(100,110),end1);
  // StartAction sa(start);
  // EndAction ea({end1});
  // AheadAction aa0(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 290}});
  // AheadAction aa01(PAT::CAhead, {{PAP::Distance, 30}, {PAP::Angle, 180}});
  // AheadAction aa02(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 270}});
  // AheadAction aa03(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 270}});
  // AheadAction aa04(PAT::CAhead, {{PAP::Distance, 10}, {PAP::Angle, 0}});
  // AheadAction aa05(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 270}});

  // AheadAction aa1(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 290}});
  // AheadAction aa11(PAT::CAhead, {{PAP::Distance, 30}, {PAP::Angle, 180}});
  // AheadAction aa12(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 270}});
  // AheadAction aa13(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 270}});
  // AheadAction aa14(PAT::CAhead, {{PAP::Distance, 10}, {PAP::Angle, 0}});
  // AheadAction aa15(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 270}});

  // PAs act1, act2;
  // act1.push_back(make_shared<StartAction>(sa));
  // act1.push_back(make_shared<AheadAction>(aa0));
  // act1.push_back(make_shared<AheadAction>(aa01));
  // act1.push_back(make_shared<AheadAction>(aa02));
  // act1.push_back(make_shared<AheadAction>(aa03));
  // act1.push_back(make_shared<AheadAction>(aa04));
  // act1.push_back(make_shared<AheadAction>(aa05));
  // act1.push_back(make_shared<EndAction>(ea));

  // act2.push_back(make_shared<StartAction>(sa));
  // act2.push_back(make_shared<AheadAction>(aa1));
  // act2.push_back(make_shared<AheadAction>(aa11));
  // act2.push_back(make_shared<AheadAction>(aa12));
  // act2.push_back(make_shared<AheadAction>(aa13));
  // act2.push_back(make_shared<AheadAction>(aa14));
  // act2.push_back(make_shared<AheadAction>(aa15));
  // act2.push_back(make_shared<EndAction>(ea));


  // genome gen1(act1), gen2(act2);


  // validateGen(gen1);
  // validateGen(gen2);

  // rob->evaluateActions(gen.actions);

  warn(cmap->getPosition(Index(100,110),pos));
  debug("Generated start pos: ", pos);
  cmap->getPosition(Index(30,22),end1);
  cmap->getPosition(Index(40,22),end2);
  int childs = 2;
  ga->populatePool(pool, pos, WPs{end1, end2}, childs, 10);

  rob->evaluateActions(pool.front().actions);
  cv::Mat gen1 = rob->gridToImg("map");
  rob->evaluateActions(pool.back().actions);
  cv::Mat gen2 = rob->gridToImg("map");

  ASSERT_EQ(pool.size(), childs);
  // ASSERT_NE(&pool.front(), &pool.back());

  ga->mating(pool.front(), pool.back(), newPop);
  ASSERT_EQ(newPop.size(), 2);

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
  next(gen.actions.begin(), 4)->get()->mendConfig(*(next(gen.actions.begin(), 3)->get()));

  rob->evaluateActions(gen.actions);

  // displayImage(rob->gridToImg("map"));


}




// TEST_F(GATest, sortingTest){

//   Genpool pool;
//   Genpool selection_pool;
//   Position pos, end1, end2;

//   debug("Generated start pos: ", pos);
//   cmap->getPosition(Index(30,22),end1);
//   cmap->getPosition(Index(40,22),end2);

//   ga->populatePool(pool, pos, WPs{end1, end2}, 100, 300);
//   ga->evalFitness(pool, *rob);
//   sort(pool.begin(), pool.end(), compareFitness);
//   for (auto gen : pool){
//     ASSERT_GE(pool.back().fitness, gen.fitness);
//   }

// }
// TEST_F(GATest, selectionTest){

//   Genpool pool;
//   Genpool selection_pool;
//   Position pos, end1, end2;

//   debug("Generated start pos: ", pos);
//   cmap->getPosition(Index(30,22),pos);
//   cmap->getPosition(Index(40,22),end2);

//   ga->populatePool(pool, pos, WPs{end1, end2}, 100, 300);
//   ga->evalFitness(pool, *rob);
//   ga->selection(pool, selection_pool, 3);

//   debug(selection_pool.front().fitness);
//   debug(selection_pool.back().fitness);
//   ASSERT_EQ(selection_pool.size(), 3);
// }

// TEST_F(GATest, calculateFreeArea){
//   // debug("Size: ", (cmap->get("obstacle").sum()));
//   // debug("Size: ", cmap->getSize().x()*cmap->getSize().y() - (cmap->get("obstacle").sum()));
//   ASSERT_EQ(cmap->getSize().x()*cmap->getSize().y() - (cmap->get("obstacle").sum()), rob->getFreeArea());
// }

// TEST_F(GATest, randTest){

//   list<int> l1, l2, c1, c2;

//   l1.push_back(1);
//   l1.push_back(2);
//   l1.push_back(3);

//   l2.push_back(4);
//   l2.push_back(5);
//   l2.push_back(6);

//   c1.insert(c1.begin(), l1.begin(), std::next(l1.begin(), 2));
//   c1.insert(c1.end(),std::next(l2.begin(), 2), l2.end());

//   for(auto i : c1){
//     debug("Value: ", i);
//   }


// }


// TEST_F(GATest, mutationConfigTest){
//   Genpool pool;
//   ga->populatePool(pool, Position(42,42), {Position(42,42)}, 2, 1);


//   genome gen1 = pool.front();
//   genome gen2 = pool.back();

//   auto size = gen1.actions.size();
//   addAction(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
//   ASSERT_EQ(gen1.actions.size(), size + 1);

//   removeAction(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
//   ASSERT_EQ(gen1.actions.size(), size);

//   auto action = *next(gen1.actions.begin(), 1);

//   auto dOffset = action->mod_config[PAP::DistanceOffset];
//   debug("DistOff: ", dOffset);
//   addDistanceOffset(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
//   ASSERT_NE(action->mod_config[PAP::DistanceOffset], dOffset);

//   auto aOffset = action->mod_config[PAP::AngleOffset];
//   debug("AngleOff: ", aOffset);
//   addAngleOffset(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
//   ASSERT_NE(action->mod_config[PAP::AngleOffset], aOffset);

// }


// TEST_F(GATest, adaptationTest){
//   Position start, end;

//   cmap->getPosition(Index(41,41), start);
//   cmap->getPosition(Index(22,22), end);
//   debug("Start: ", start);
//   debug("End: ", end);
//   debug("Distance: ", start - end);

//   StartAction sa(start);
//   EndAction ea({end});
//   AheadAction aa(PAT::CAhead, {{PAP::Distance, 20}, {PAP::Angle, 270}});
//   AheadAction aa1(PAT::CAhead, {{PAP::Distance, 30}, {PAP::Angle, 180}});
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
//   // addAngleOffset(gen, ga->distanceDistr, ga->angleDistr, ga->generator);
//   rob->evaluateActions(gen.actions);
//   addAction(gen, ga->distanceDistr, ga->angleDistr, ga->generator);
//   validateGen(gen);
//   rob->evaluateActions(gen.actions);

//   // displayImage(rob->gridToImg("map"));


// }



// TEST_F(GATest, boundaryMendingTest){
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
//   debug("1New size: ", gen.actions.size());
//   // addAngleOffset(gen, ga->distanceDistr, ga->angleDistr, ga->generator);
//   rob->evaluateActions(gen.actions);
//   // addAction(gen, ga->distanceDistr, ga->angleDistr, ga->generator);
//   // validatldeGen(gen);
//   // displayImage(rob->gridToImg("map"));
//   rob->evaluateActions(gen.actions);
//   rob->evaluateActions(gen.actions);

//   debug("2New size: ", gen.actions.size());

//   debug("Dist ", next(gen.actions.begin(), 1)->get()->mod_config[PAP::Distance]);
//   debug("Dist ", next(gen.actions.begin(), 2)->get()->mod_config[PAP::Distance]);

//   // displayImage(rob->gridToImg("map"));


// }


// TEST_F(GATest, basicApplicationTest){
//   Genpool pool, sel, newPop;
//   Position start, end;
//   Mutation_conf muta = {
//       {"addAction", make_pair(addAction, 100)},
//       // {"removeAction", make_pair(removeAction, 100)},
//       {"addAngleOffset", make_pair(addAngleOffset, 100)},
//       {"addDistanceOffset", make_pair(addDistanceOffset, 100)},
//       // {"swapRandomAction", make_pair(swapRandomAction, 100)},
//     };
//   cmap->getPosition(Index(100,100), start);
//   cmap->getPosition(Index(300,300), end);
//   debug("Start");
//   ga->populatePool(pool, start, {end}, 10, 300);
//   ASSERT_EQ(pool.size(), 10);
//   for(auto gen : pool){
//     ASSERT_GT(gen.actions.size(), 30);
//   }
//   ga->evalFitness(pool, *rob);
//   ASSERT_GT(pool.size(), 0);
//   debug("Select: ", pool.size());
//   ga->selection(pool, sel, 5);
//   for(auto gen : sel){
//     debug("Check");
//     ASSERT_GT(gen.actions.size(), 30);
//   }
//   ASSERT_EQ(sel.size(), 5);
//   debug("Cross");
//   pool.clear();
//   ga->crossover(sel, pool);
//   ASSERT_GT(pool.size(), 6);
//   for(auto gen : pool){
//     ASSERT_GT(gen.actions.size(), 2);
//   }

//   debug("Mutate");
//   ga->mutation(pool, muta);
//   ga->evalFitness(pool, *rob);
//   debug("Done");

//   // displayImage(rob->gridToImg("map"));

// }



class GAApplication : public ::testing::Test {
protected:
  void SetUp() override {

    Mutation_conf muta = {
      {"addAction", make_pair(addAction, 60)},
      {"removeAction", make_pair(removeAction, 10)},
      {"addAngleOffset", make_pair(addAngleOffset, 90)},
      {"addDistanceOffset", make_pair(addDistanceOffset, 50)},
      {"swapRandomAction", make_pair(swapRandomAction, 10)},
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
    rob = make_shared<Robot>(Robot(42, {}, map));

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



TEST_F(GAApplication, algorithmTest){
  Genpool pool, sel, newPop;
  Position start, end;
  Mutation_conf muta = {
      // {"addAction", make_pair(addAction, 100)},
      // {"removeAction", make_pair(removeAction, 10)},
      {"addAngleOffset", make_pair(addAngleOffset, 70)},
      {"addDistanceOffset", make_pair(addDistanceOffset, 70)},
      // {"swapRandomAction", make_pair(swapRandomAction, 10)},
    };
  int iter = 10000 ;
  int selected = 15;
  cmap->getPosition(Index(110,110), start);
  cmap->getPosition(Index(110,110), end);
  debug("Start");
  genome best;

  ga->populatePool(pool, start, {end}, 100, 200);
  ga->evalFitness(pool, *rob);
  ga->selection(pool, sel, selected);
  ASSERT_EQ(sel.size(), selected);
  best = pool.front();
  for(int i=0; i<iter; i++){

    pool.clear();
    // debug("Cross");
    // pool.push_back(genome(best));
    ga->crossover(sel, pool);

    sel.clear();
    // debug("Mut");
    // addAction(pool.back(), ga->angleDistr, ga->distanceDistr, ga->generator);
    ga->mutation(pool, muta);
    // debug("Cla");
    ga->evalFitness(pool, *rob);
    for (auto gen : pool){
      ASSERT_GT(gen.fitness, 0);
    }
    // debug("Sel");
    // for (auto gen : pool){
    //   ASSERT_GT(gen.actions.size(), 50);
    // }
    // for(auto it = pool.begin(); it != prev(pool.end(), 1);){

    //   if(it->fitness == next(it, 1)->fitness){
    // 	it = pool.erase(it);
    //   }else{
    // 	it++;
    //   }
    // }
    int size = pool.size();
    ga->selection(pool, sel, selected);
    ASSERT_EQ(pool.size(), size - selected);

    ASSERT_EQ(sel.size(), selected);
    // debug("Done");
    // info("Population size: ", pool.size());
    if(pool.back().fitness > best.fitness){
      best = pool.front().fitness;
    }
    int lowest = 1000;
    int highest = 0;
    if (true){
      for (auto &gen : pool){
	// cout << "Gens: " << gen.fitness << " ";
	for(auto ac : gen.actions){
	  ASSERT_TRUE(!isnan(ac->mod_config[PAP::Angle]));
	  ASSERT_TRUE(!isnan(ac->mod_config[PAP::Distance]));
	}
	int a_size = gen.actions.size();
	if(a_size < lowest){
	  lowest = a_size;
	}
	if(a_size > highest){
	  highest = a_size;
	}
      }
      // cout << endl;
      info("Best fitness: ", best.fitness,
	   " Pool best: ", pool.front().fitness,
	   " Worst fitness: ", pool.back().fitness,
	   " Max Actions: ", highest,
	   " Min Actions: ", lowest
	   );
      rob->evaluateActions(pool.back().actions);
      auto img = rob->gridToImg("map");
      // displayImage(img);
      cv::imshow("Training", img);
      cv::waitKey(1);
      // cv::imwrite("res/it_" + std::to_string(i) + ".jpg", img);
    }

  }
  // for (auto gen : pool){
  //   for (auto ac : gen.actions){
  //     cout << ac->pa_id << "|";
  //   }
  //   cout << endl;
  //   cout << "-------" << endl;
  // }
}


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

//   ASSERT_EQ(gen.actions.size(), size + iter);

//   for(int i=0; i<iter;i++){
//     removeAction(gen,  ga->angleDistr,  ga->distanceDistr,  ga->generator);
//     validateGen(gen);
//     rob->evaluateActions(gen.actions);
//     // displayImage(rob->gridToImg("map"));
//   }
//   ASSERT_EQ(gen.actions.size(), size );
// }


TEST(ActionCopy, copytest){
  AheadAction aa(PAT::CAhead, {{PAP::Distance, 42}});
  aa.generateWPs(Position(42,42));
  ASSERT_EQ(aa.wps.size(), 2);
  AheadAction aa_copy(aa);

  ASSERT_EQ(aa_copy.wps.size(), 2);
  ASSERT_EQ(aa_copy.wps.front(), aa.wps.front());
  ASSERT_TRUE(!aa_copy.modified);

}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
