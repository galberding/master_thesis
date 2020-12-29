#include <gtest/gtest.h>
#include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"
#include <opencv2/opencv.hpp>

using namespace ga;
using namespace path;


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
    ga = make_shared<GA_V2>(GA_V2(42, 4, 0.9, 0, 40, muta));
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

TEST_F(GAApplication, algorithmTest){
  Genpool pool, sel, newPop;
  Position start, end;
  Mutation_conf muta = {
      // {"addAction", make_pair(addAction, 10)},
      // {"removeAction", make_pair(removeAction, 10)},
      // {"addAngleOffset", make_pair(addAngleOffset, 70)},
      // {"addDistanceOffset", make_pair(addDistanceOffset, 70)},
      // {"swapRandomAction", make_pair(swapRandomAction, 10)},
    };
  int iter = 10000 ;

  int initPop = 200;
  int selected = 25;
  cmap->getPosition(Index(110,110), start);
  cmap->getPosition(Index(110,110), end);
  debug("Start");
  genome best;

  int actionSize = 60;

  ga->populatePool(pool, start, {end}, initPop, actionSize);
  for(auto gen : pool){
    EXPECT_EQ(gen.actions.size(), actionSize + 2);
  }
  debug("First eval fitness");
  ga->evalFitness(pool, *rob);
  for(auto gen : pool){
    EXPECT_EQ(gen.actions.size(), actionSize + 2);
  }
  debug("First Select");
  ga->selection(pool, sel, selected);
  for(auto gen : sel){
    EXPECT_EQ(gen.actions.size(), actionSize + 2);
  }

  EXPECT_EQ(sel.size(), selected);
  if (pool.size() > 0) {
    best = pool.front();
  }else{
    best = genome();
  }

  for(int i=0; i<iter; i++){

    // pool.clear();
    ga->crossover(sel, pool);
    sel.clear();
    ga->mutation(pool, muta);
    ga->evalFitness(pool, *rob);

    int size = pool.size();
    ga->selection(pool, sel, selected, 10);
    EXPECT_EQ(pool.size(), size - selected);
    EXPECT_EQ(sel.size(), selected);

    if(pool.back().fitness > best.fitness){
      best = pool.back().fitness;
    }
    int lowest = 1000;
    int highest = 0;
    if (true){
      for (auto &gen : pool){
	// cout << "Gens: " << gen.fitness << " ";
	for(auto ac : gen.actions){
	  EXPECT_TRUE(!isnan(ac->mod_config[PAP::Angle]));
	  EXPECT_TRUE(!isnan(ac->mod_config[PAP::Distance]));
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
	   " Pool best: ", pool.back().fitness,
	   " Worst fitness: ", pool.front().fitness,
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
}



int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
