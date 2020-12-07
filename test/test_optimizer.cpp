#include <gtest/gtest.h>
#include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"

using namespace ga;
using namespace path;

class GATest : public ::testing::Test{
protected:
  void SetUp() override {
    ga = make_shared<GA>(GA(42, 2, 1, 0, 90));
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
    debug("Finess: ", gen.fitness);
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



int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
