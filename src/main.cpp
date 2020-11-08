#include "environment/robot.hpp"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <experimental/random>
// #include "environment/ocm.hpp"
#include <iterator>
#include <geometric_shapes/shapes.h>

#include <cstring>
#include <string>

#include "pool.hpp"
// #include <boost/timer/timer.hpp>
#include <cmath>
#include <stack>

using namespace std;

using namespace cv;

int main(int argc, char *argv[])
{
  Robot rob;
  // foo::OccupancyMap ocm(10, 10);
  // cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  const uint32_t width = 500;
  const uint32_t height = 500;
  const Point start(0,50);
  const Point stop(height, 50);
  const int rob_size = 10;
  const float rob_speed = 10;
  int offspringCount = 100;
  uint8_t im[height][width] =  { 0 };
  uint8_t im2[height][width] =  { 0 };
  uint8_t im3[height][width] =  { 0 };
  Mat map(height, width, CV_8U, im);
  Mat map2(height, width, CV_8U, im2);
  Mat map3(height, width, CV_8U, im3);

  vector<Point> waypoints = opti_ga::genWaypoints(width, height, start, 10);

  // tac.push(1, "hello");
  opti_ga::GenPool pool(width, height, start, stop, rob_size, rob_speed, offspringCount);
  pool.populatePool(10000, 100);
  // cout << pool.gens.size() << endl;
  // // pool.crossover();
  // cout << pool.gens.size() << endl;
  pool.update(5000);
  // vector<Point> pp = {Point(1,2), Point(3,4), Point(3,3)};
  // cout << pp.size() << "\n";
  // pp.erase(pp.begin()+1);
  // cout << pp.size() << "\n";
  // cout << pool.pool_to_string();


  return 0;
}
