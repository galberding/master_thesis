#include "environment/robot.hpp"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <experimental/random>
// #include "environment/ocm.hpp"
#include <iterator>
// #include <geometric_shapes/shapes.h>

#include <cstring>
#include <string>

#include "pool.hpp"
// #include <boost/timer/timer.hpp>
#include <cmath>
#include <stack>
#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
// #include <grid_map/grid_map.hpp>

using namespace std;

using namespace cv;
using namespace grid_map;


int main(int argc, char *argv[])
{
  Robot rob;
  // foo::OccupancyMap ocm(10, 10);
  // cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  const uint32_t width = 500;
  const uint32_t height = 500;
  const Point start(0,50);
  const Point stop(height, 50);
  const int rob_size = 30;
  const float rob_speed = 10;
  int offspringCount = 25;
  uint8_t im[height][width] =  { 0 };
  uint8_t im2[height][width] =  { 0 };
  uint8_t im3[height][width] =  { 0 };
  Mat map(height, width, CV_8U, im);
  Mat map2(height, width, CV_8U, im2);
  Mat map3(height, width, CV_8U, im3);

  vector<Point> waypoints = opti_ga::genWaypoints(width, height, start, 10);

  opti_ga::GenPool pool(width, height, start, stop, rob_size, rob_speed, offspringCount);
  pool.populatePool(1000, 5);
  // pool.update(10000);
  grid_map::GridMap gmap({"color"});
  gmap.setFrameId("map");
  gmap.setGeometry(Length(10,10), 0.5);
  cout << gmap.getLength().x() << endl;

  // Point A(2,2), B(1,2);

  // cout << A.dot(B) / (norm(A)*norm(B)) << endl;

  return 0;
}
