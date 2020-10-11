#include "environment/robot.hpp"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <experimental/random>
#include "environment/ocm.hpp"
#include <iterator>
#include <geometric_shapes/shapes.h>

using namespace std;

using namespace cv;

void MyLine( Mat img, Point start, Point end );

class Gen{
public:
  Gen(shared_ptr<cv::Mat> map){
    this->map = map;
  }
  shared_ptr<Mat> getMap() {return map;}


private:
  shared_ptr<cv::Mat> map;
  list<Point> waypoints;
};


// class GenPool{

// public:
//   void populate(int size);
//   void update();

// };

Point randomPointShift(Point p, int magnitude = 200){

  Point p2(p);

  int shift_x = std::experimental::randint(-magnitude, magnitude);
  int shift_y = std::experimental::randint(-magnitude, magnitude);
  p2.x +=  shift_x;
  p2.y +=  shift_y;
  return p2;

}

list<Point> genWaypoints(int width, int height, Point start, int n = 30){
  // Generate list with valid waypoints
  // For now the only rule is that the points need to be placed on the map
  list<Point> waypoints;
  Point shift_p(start);
  for (int i = 0; i < n; ++i) {
    bool valid = false;

    while(true){
      Point tmp = randomPointShift(shift_p);
      if((tmp.x < width) && (tmp.y < height) && (tmp.x > 0) && tmp.y > 0){
	shift_p = tmp;
	waypoints.push_back(tmp);
	break;
      }
    }

  }
  return waypoints;
}


float cal_occ(list<Point> waypoints, int width, int height){

  Mat map(height, width, CV_8U, Scalar(0));
  // cout << "Initial cost: " << cv::sum(map) << endl;

  auto iter = waypoints.begin();
  Point current = *iter;
  iter++;
  do{
    MyLine(map, current, *iter);
    current = *iter;
    // cout << current << endl;
    iter++;
  } while(iter != waypoints.end());

  // MyLine(map, current, stop);
  // cout << "Result: " << cv::sum(map)[0] / (width * height) << endl;

  // Mat map2(height, width, CV_8U, Scalar(255));
  // cout << "Result2: " << cv::sum(map2)[0] / (width * height) << endl;
  // cv::imshow("Helllo", map);
  // cv::waitKey();
  return cv::sum(map)[0] / (width * height);
}


float cal_time(list<Point> waypoints, int speed){

  float dist = 0;
  auto iter = waypoints.begin();
  Point current = *iter;
  iter++;
  do{
    // MyLine(map, current, *iter);
    // Sum up all distances the roboter needs to travel
    dist += cv::norm(current - *iter);
    current = *iter;
    // cout << current << endl;
    iter++;
  } while(iter != waypoints.end());

  return dist / speed;

}



int main(int argc, char *argv[])
{
  Robot rob;
  // foo::OccupancyMap ocm(10, 10);
  // cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  const uint32_t width = 1000;
  const uint32_t height = 1000;
  const Point start(0,50);
  const Point stop(999, 500);

  uint8_t im[height][width] =  { 0 };
  Mat map(height, width, CV_8U, im);

  list<Point> waypoints = genWaypoints(width, height, start);

  auto occ_fit = cal_occ(waypoints, width, height);
  auto time_fit = cal_time(waypoints, 3);

  // Optimization stage 1:
  // TODO: Waypoint generation
  // TODO: Geometric rotation and transformation on map

  // First assumption to ease the constraints is to get the robot a round shape




  // printf("Hello Rob: %d\n", rob.hello());
  return 0;
}

void MyLine( Mat img, Point start, Point end )
{
  int thickness = 2;
  int lineType = LINE_4;
  line( img,
    start,
    end,
    Scalar( 255 ),
    thickness,
    lineType );
}
