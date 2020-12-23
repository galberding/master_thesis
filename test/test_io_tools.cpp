#include <gtest/gtest.h>
#include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"
#include <opencv2/opencv.hpp>
#include "../src/optimizer/grid_search.h"


using namespace ga;
using namespace path;
namespace fs = std::filesystem;
// using namespace gsearch;

void displayGridmap(shared_ptr<GridMap> cmap){
    cv::Mat obstacle, rob_map;

    GridMapCvConverter::toImage<unsigned char, 1>(*cmap, "obstacle", CV_8U, 0.0, 255, obstacle);
    // GridMapCvConverter::toImage<unsigned char, 1>(*cmap, "map", CV_8U, 0.0, 1, rob_map);

    // cv::imwrite("testio.jpg", obstacle);
    cv::imshow("Grid Map Obstacle", obstacle);
    // cv::imshow("Grid Map Map", rob_map);
    // cv::moveWindow("Grid Map Map", 1000, 0);
    cv::waitKey();
  }

TEST(Map, generation){
  gsearch::Searcher se;
  auto mapptr = se.generateMapType(100, 50, 0.2, 1);

  displayGridmap(mapptr);
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
