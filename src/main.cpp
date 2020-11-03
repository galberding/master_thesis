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
  uint8_t im[height][width] =  { 0 };
  uint8_t im2[height][width] =  { 0 };
  uint8_t im3[height][width] =  { 0 };
  Mat map(height, width, CV_8U, im);
  Mat map2(height, width, CV_8U, im2);
  Mat map3(height, width, CV_8U, im3);

  vector<Point> waypoints = opti_ga::genWaypoints(width, height, start, 10);

  // auto occ_fit = cal_occ(waypoints, width, height);
  // auto time_fit = cal_time(waypoints, 3);

  Point A(10,30), B(300,30);
  int size = 40;

  // auto t_begin = chrono::steady_clock::now();
  // for(auto i=0; i<waypoints.size()-2; i++)
  //   cv::line(map, waypoints[i], waypoints[i+1], Scalar(255), size, LINE_8);
  // cout << "Line 1 " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count() << endl;

  // t_begin = chrono::steady_clock::now();
  // mainline(map2, A, B, size);
  // cout << "Line 2 " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count() << endl;

  // t_begin = chrono::steady_clock::now();
  // calOccRec(map3, A, B, size);
  // cout << "Rec: " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count() << endl;

  // t_begin = chrono::steady_clock::now();
  // getIndex(A, B);
  // cout << "Index 1 " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count() << endl;
  // t_begin = chrono::steady_clock::now();
  // // getIndex(Size(height, width),A, B, size);
  // for(auto &p: getIndex(Size(height, width),A, B, size)){
  //   map2.at<uchar>(p) = 255;
  // }
  // cout << "Index 2 " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count() << endl;



  // t_begin = chrono::steady_clock::now();
  // // calOccRec(map3, A, B, size);
  // calOccupiedArea(waypoints, size);
  // cout << "Initial intersection: " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count() << endl;
  // boost::timer::cpu_timer t;


  // tac.push(1, "hello");
  opti_ga::GenPool pool(width, height, start, stop, rob_size, rob_speed);
  pool.populatePool(8, 10);
  // // cout << pool.calTime(pool.gens.at(0)) << endl;
  // // cout << "Fitness: " << pool.getBest().fitness << endl;
  // // for(const auto i : pool.gens){
  // //   cout << i.fitness << endl;
  // // }
  pool.update(100000);
  // // imwrite("res/it_" + std::to_string(), *pool.getBest().map);
  // imshow("Res: ", map);
  // imshow("Self: ", map2);
  // moveWindow("Self: ", 100, 0);
  // imshow("Filling: ", map3);
  // waitKey(0);

  // opti_ga::Timer tt;
  // tt.t_start("Beginnig");
  // tt.t_start("inside");
  // tt.t_start("inside2");
  // cout << "test" << endl;
  // tt.t_end();
  // tt.t_end();
  // tt.t_end();

  // tt.printTiming();

  // Optimization stage 1:
  // TODO: Waypoint generation
  // TODO: Geometric rotation and transformation on map

  // First assumption to ease the constraints is to get the robot a round shape




  // printf("Hello Rob: %d\n", rob.hello());
  return 0;
}
