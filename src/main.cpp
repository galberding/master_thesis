#include "environment/robot.hpp"
#include "tools/path_tools.h"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <experimental/random>
// #include "environment/ocm.hpp"
#include <iterator>
// #include <geometric_shapes/shapes.h>

#include <cstring>
#include <string>

// #include "pool.hpp"
// #include <boost/timer/timer.hpp>
#include <cmath>
#include <stack>
#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
// #include <grid_map/grid_map.hpp>

using namespace std;

using namespace cv;
using namespace grid_map;

#define w 400

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
using namespace octomap;


void MyEllipse( Mat img, double angle )
{
  int thickness = 2;
  int lineType = 8;
  ellipse( img,
	   Point( 100, 100 ),
       cv::Size( 10, 5 ),
       angle,
       90,
       360,
       Scalar(255),
       thickness,
       4 );
}



void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}



int main(int argc, char *argv[])
{
  // Robot rob;
  // foo::OccupancyMap ocm(10, 10);
  // cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  // const uint32_t width = 500;
  // const uint32_t height = 500;
  // const Point start(0,50);
  // const Point stop(height, 50);
  // const int rob_size = 30;
  // const float rob_speed = 10;
  // int offspringCount = 25;
  // uint8_t im[height][width] =  { 0 };
  // uint8_t im2[height][width] =  { 0 };
  // uint8_t im3[height][width] =  { 0 };
  // Mat map(height, width, CV_8U, im);
  // Mat map2(height, width, CV_8U, im2);
  // Mat map3(height, width, CV_8U, im3);



  return 0;
}
