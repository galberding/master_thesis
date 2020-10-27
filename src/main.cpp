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

using namespace std;

using namespace cv;



float calOccRec(Mat &map, Point p0, Point p1, int size){
  Point2f V = p1-p0;
  // cout << "Start " << V << endl;
  float v_len = cv::norm(V);
  // Calculate unit vector:
  Point2f U(-V.y/v_len,V.x/v_len);
  Point2f Y_unit(0,1) ;

  // cout << "Unit: " << U << endl;

  // Calculate endpoints

  Point P[4];
  vector<Point> pts;

  auto scale_x = U.x*(size/2);
  auto scale_y = U.y*(size/2);

  pts.push_back(Point(p0.x - scale_x, p0.y - scale_y));
  pts.push_back(Point(p0.x + scale_x, p0.y + scale_y));
  pts.push_back(Point(p1.x - scale_x, p1.y - scale_y));
  pts.push_back(Point(p1.x + scale_x, p1.y + scale_y));

  auto center = (p0 + p1) / 2;


  // Determain the center of the rectangle
  auto dot = V.x*Y_unit.x + V.y*Y_unit.y;

  auto det = V.x*Y_unit.y - V.y*Y_unit.x;

  auto angle = atan2(det, dot)* 180 / CV_PI;
  // cout << "Angle " << angle << endl;

  // for(auto i : pts){
  //   cout << i << endl;
  // }

  // cout << "Center " << center << endl;
  auto rec = cv::RotatedRect(
			     center,
			     Size2f(size, v_len),
			     -angle
			     );
  // auto rec = cv::RotatedRect(
  // 			     Point2f(p1.x - scale_x, p1.y - scale_y),
  // 			     Point2f(p0.x - scale_x, p0.y - scale_y),
  // 			     Point2f(p0.x + scale_x, p0.y + scale_y)
  // 			     );

  // P[0][0] = p0 + U*(size/2);
  // cout << U << endl;
  // P[0][3] = p1 - U*(size/2);
  // P[0][2] = p1 + U*(size/2);
  // for(auto i : pts){
  //   cout << i << endl;
  // }
  // const Point* ppt[1] = {P};
  // int npt[] = { 4 };
  // // cv:rectangle(map, P0, P1, Scalar(255));
  // cv::rotated(map, rec, 255);
  // Point2f vertices[4];
  // rec.points(vertices);
  // for (int i = 0; i < 4; i++)
  //   line(map, vertices[i], vertices[(i+1)%4], Scalar(255), 1);

  return rec.size.area();
}

cv::Rect constructRec(Point &p0, Point &p1, int size){
  Point2f V = p1-p0;
  // cout << "Start " << V << endl;
  float v_len = cv::norm(V);
  // Calculate unit vector:
  Point2f U(-V.y/v_len,V.x/v_len);
  // cout << "Unit: " << U << endl;

  // Calculate endpoints

  vector<Point> pts;

  auto scale_x = U.x*(size/2);
  auto scale_y = U.y*(size/2);

  pts.push_back(Point(p0.x - scale_x, p0.y - scale_y));
  pts.push_back(Point(p0.x + scale_x, p0.y + scale_y));
  pts.push_back(Point(p1.x + scale_x, p1.y + scale_y));
  pts.push_back(Point(p1.x - scale_x, p1.y - scale_y));

  return cv::Rect(Point2f(p0.x - scale_x, p0.y - scale_y),
		  Point2f(p1.x + scale_x, p1.y + scale_y));

  return Rect();
}




void calOccupiedArea(){

}



// void line(int x0, int y0, int x1, int y1)
// void line(uint8_t map[500][500],  Point p0, Point p1)
void mainline(Mat &map,  Point p0, Point p1, int size)
{
  // https://de.wikipedia.org/wiki/Bresenham-Algorithmus

  int dx =  abs(p1.x-p0.x), sx = p0.x<p1.x ? 1 : -1;
  int dy = -abs(p1.y-p0.y), sy = p0.y<p1.y ? 1 : -1;
  int err = dx+dy, e2; /* error value e_xy */

  while (1) {
    // cout << "x: " << p0.x << " y " << p0.y << endl;
    for (auto row=0; row <= int(size/2); ++row) {
      for (auto col = 0; col <= int(size / 2); ++col) {
	map.at<uchar>(p0.x+col, p0.y+row) = 255;
      }
    }


    if (p0.x==p1.x && p0.y==p1.y) break;
    e2 = 2*err;
    if (e2 > dy) { err += dy; p0.x += sx; } /* e_xy+e_x > 0 */
    if (e2 < dx) { err += dx; p0.y += sy; } /* e_xy+e_y < 0 */
  }
}

// void line(Mat map,  Point p0, Point p1, int size){
//   for(auto i=int(-size/2); i < int(size/2); i++){
//     line(map, Point(p0.x+i, p0.y+i), Point(p1.x+i, p1.y+i));
//   }
// }

int main(int argc, char *argv[])
{
  Robot rob;
  // foo::OccupancyMap ocm(10, 10);
  // cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  const uint32_t width = 500;
  const uint32_t height = 500;
  const Point start(0,50);
  const Point stop(99, 50);
  const int rob_size = 12;
  const float rob_speed = 5;
  uint8_t im[height][width] =  { 0 };
  uint8_t im2[height][width] =  { 0 };
  uint8_t im3[height][width] =  { 0 };
  Mat map(height, width, CV_8U, im);
  Mat map2(height, width, CV_8U, im2);
  Mat map3(height, width, CV_8U, im3);

  vector<Point> waypoints = opti_ga::genWaypoints(width, height, start);

  // auto occ_fit = cal_occ(waypoints, width, height);
  // auto time_fit = cal_time(waypoints, 3);

  Point A(10,30), B(100,200);
  int size = 10;

  auto t_begin = chrono::steady_clock::now();
  cv::line(map, A, B, Scalar(255), size, LINE_8);
  cout << "Line 1 " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count() << endl;

  t_begin = chrono::steady_clock::now();
  mainline(map2, A, B, size);
  cout << "Line 2 " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count() << endl;

   t_begin = chrono::steady_clock::now();
  calOccRec(map3, A, B, size);
  cout << "Rec: " << std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count() << endl;
  // boost::timer::cpu_timer t;
  // opti_ga::GenPool pool(width, height, start, stop, rob_size, rob_speed);
  // pool.populatePool(8, 10);
  // // cout << pool.calTime(pool.gens.at(0)) << endl;
  // // cout << "Fitness: " << pool.getBest().fitness << endl;
  // // for(const auto i : pool.gens){
  // //   cout << i.fitness << endl;
  // // }
  // // pool.update(100000);
  // // imwrite("res/it_" + std::to_string(), *pool.getBest().map);
  imshow("Res: ", map);
  // imshow("Self: ", map2);
  imshow("Rec: ", map3);
  waitKey(0);


  // Optimization stage 1:
  // TODO: Waypoint generation
  // TODO: Geometric rotation and transformation on map

  // First assumption to ease the constraints is to get the robot a round shape




  // printf("Hello Rob: %d\n", rob.hello());
  return 0;
}
