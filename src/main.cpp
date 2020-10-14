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

using namespace std;

using namespace cv;


int main(int argc, char *argv[])
{
  Robot rob;
  // foo::OccupancyMap ocm(10, 10);
  // cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  const uint32_t width = 1000;
  const uint32_t height = 1000;
  const Point start(0,50);
  const Point stop(999, 500);
  const int rob_size = 3;
  const float rob_speed = 5;
  uint8_t im[height][width] =  { 0 };
  Mat map(height, width, CV_8U, im);

  vector<Point> waypoints = opti_ga::genWaypoints(width, height, start);

  // auto occ_fit = cal_occ(waypoints, width, height);
  // auto time_fit = cal_time(waypoints, 3);



  opti_ga::GenPool pool(width, height, start, stop, rob_size, rob_speed);
  pool.populatePool(10, 50);
  cout << pool.calTime(pool.gens.at(0)) << endl;
  // cout << "Fitness: " << pool.getBest().fitness << endl;
  // for(const auto i : pool.gens){
  //   cout << i.fitness << endl;
  // }
  pool.update(100000);
  // imwrite("res/it_" + std::to_string(), *pool.getBest().map);
  // imshow("Res: ", *pool.getBest().map);
  // waitKey(0);


  // Optimization stage 1:
  // TODO: Waypoint generation
  // TODO: Geometric rotation and transformation on map

  // First assumption to ease the constraints is to get the robot a round shape




  // printf("Hello Rob: %d\n", rob.hello());
  return 0;
}
