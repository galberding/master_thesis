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
      {"swapRandomAction", make_pair(swapRandomAction, 10)},
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

  // PAs actions;
  shared_ptr<GridMap> cmap;
  shared_ptr<Robot> rob;
  shared_ptr<GA> ga;
};


TEST_F(GATest, initPopulationTest){
  Genpool pool;
  Genpool selection_pool;
  Position pos, end1, end2;

  cmap->getPosition(Index(22,22),pos);
  cmap->getPosition(Index(30,22),end1);
  cmap->getPosition(Index(40,22),end2);
  ga->populatePool(pool, pos, WPs{end1, end2}, 100, 300);

  // ASSERT_EQ(pool.size(), 12);
  // ASSERT_EQ(pool.front().actions.size(), 14);

  ASSERT_TRUE(rob->evaluateActions(pool.front().actions));
  debug("Remaining size: ", pool.front().actions.size());

  ga->evalFitness(pool, *rob);

  for(auto gen : pool){
    debug("Fitness: ", gen.fitness);
  }

  // displayImage(rob->gridToImg("map"));
}


TEST_F(GATest, selectionTest){
  Genpool pool{genome(0.0), genome(1.0), genome(0.5), genome(100.0)};
  Genpool selection_pool;
  ga->selection(pool, selection_pool, 3);

  // debug(selection_pool.front().fitness);
  ASSERT_EQ(selection_pool.size(), 3);
}

TEST_F(GATest, calculateFreeArea){
  // debug("Size: ", (cmap->get("obstacle").sum()));
  // debug("Size: ", cmap->getSize().x()*cmap->getSize().y() - (cmap->get("obstacle").sum()));
  ASSERT_EQ(cmap->getSize().x()*cmap->getSize().y() - (cmap->get("obstacle").sum()), rob->getFreeArea());
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
  ga->populatePool(pool, Position(42,42), {Position(42,42)}, 2, 1);


  genome gen1 = pool.front();
  genome gen2 = pool.back();

  auto size = gen1.actions.size();
  addAction(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
  ASSERT_EQ(gen1.actions.size(), size + 1);

  removeAction(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
  ASSERT_EQ(gen1.actions.size(), size);

  auto action = *next(gen1.actions.begin(), 1);

  auto dOffset = action->mod_config[PAP::DistanceOffset];
  debug("DistOff: ", dOffset);
  addDistanceOffset(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
  ASSERT_NE(action->mod_config[PAP::DistanceOffset], dOffset);

  auto aOffset = action->mod_config[PAP::AngleOffset];
  debug("AngleOff: ", aOffset);
  addAngleOffset(gen1, ga->angleDistr, ga->distanceDistr, ga->generator);
  ASSERT_NE(action->mod_config[PAP::AngleOffset], aOffset);

}


TEST_F(GATest, basicApplicationTest){
  Genpool pool, sel, newPop;
  Position start, end;
  Mutation_conf muta = {
      {"addAction", make_pair(addAction, 100)},
      {"removeAction", make_pair(removeAction, 100)},
      {"addAngleOffset", make_pair(addAngleOffset, 100)},
      {"addDistanceOffset", make_pair(addDistanceOffset, 100)},
      {"swapRandomAction", make_pair(swapRandomAction, 100)},
    };
  cmap->getPosition(Index(11,11), start);
  cmap->getPosition(Index(11,11), end);
  debug("Start");
  ga->populatePool(pool, start, {end}, 10, 30);
  ASSERT_EQ(pool.size(), 10);
  for(auto gen : pool){
    ASSERT_GT(gen.actions.size(), 30);
  }
  ga->evalFitness(pool, *rob);
  ASSERT_GT(pool.size(), 0);
  debug("Select: ", pool.size());
  ga->selection(pool, sel, 5);
  for(auto gen : sel){
    debug("Check");
    ASSERT_GT(gen.actions.size(), 2);
  }
  ASSERT_EQ(sel.size(), 5);
  debug("Cross");
  pool.clear();
  ga->crossover(sel, pool);
  ASSERT_GT(pool.size(), 6);
  for(auto gen : pool){
    ASSERT_GT(gen.actions.size(), 2);
  }

  debug("Mutate");
  ga->mutation(pool, muta);
  ga->evalFitness(pool, *rob);
  debug("Done");

}


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
    ga = make_shared<GA>(GA(42, 2, 0.5, 0, 30, muta));
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

  // PAs actions;
  shared_ptr<GridMap> cmap;
  shared_ptr<Robot> rob;
  shared_ptr<GA> ga;
};


TEST_F(GAApplication, algorithmTest){
  Genpool pool, sel, newPop;
  Position start, end;
  Mutation_conf muta = {
      {"addAction", make_pair(addAction, 10)},
      {"removeAction", make_pair(removeAction, 10)},
      {"addAngleOffset", make_pair(addAngleOffset, 70)},
      {"addDistanceOffset", make_pair(addDistanceOffset, 70)},
      {"swapRandomAction", make_pair(swapRandomAction, 10)},
    };
  int iter = 100 ;
  int selected = 15;
  cmap->getPosition(Index(100,100), start);
  cmap->getPosition(Index(100,100), end);
  debug("Start");
  genome best;
  ga->populatePool(pool, start, {end}, 100, 100);
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
    ga->mutation(pool, muta);
    // debug("Cla");
    ga->evalFitness(pool, *rob);
    // debug("Sel");
    for (auto gen : pool){
      ASSERT_GT(gen.actions.size(), 50);
    }

    // for (int i=0;  i<pool.size()-2= var.end(); ++ i=0) {

    // }
    for(auto it = pool.begin(); it != prev(pool.end(), 1);){

      if(it->fitness == next(it, 1)->fitness){
	it = pool.erase(it);
      }else{
	it++;
      }
    }
    ga->selection(pool, sel, selected);
    for (auto gen : pool){
      ASSERT_GT(gen.fitness, 0);
    }
    ASSERT_EQ(sel.size(), selected);
    debug("Done");


    info("Population size: ", pool.size());
       if(pool.front().fitness > best.fitness){
      best = pool.front().fitness;
    }
    if (true){
      for (auto gen : pool){
	cout << "Gens: " << gen.fitness << " ";
      }
      cout << endl;
      info("Best fitness: ", best.fitness, "Pool best: ", pool.front().fitness," Worst fitness: ", pool.back().fitness);
      rob->evaluateActions(pool.front().actions);
      auto img = rob->gridToImg("map");
      cv::imwrite("res/it_" + std::to_string(i) + ".jpg", img);
    }
  }
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
