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


struct genome
{
  shared_ptr<cv::Mat> map;
  vector<Point> waypoints;
  float fitness = 0;
};


Point randomPointShift(Point p, int magnitude = 200){

  Point p2(p);

  int shift_x = std::experimental::randint(-magnitude, magnitude);
  int shift_y = std::experimental::randint(-magnitude, magnitude);
  p2.x +=  shift_x;
  p2.y +=  shift_y;
  return p2;

}

vector<Point> genWaypoints(int width, int height, Point start, int n = 10){
  // Generate list with valid waypoints
  // For now the only rule is that the points need to be placed on the map
  vector<Point> waypoints;
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


float calOcc(vector<Point> waypoints, int width, int height){

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
  return cv::sum(map)[0] / (width * height);
}


float calTime(vector<Point> waypoints, int speed){

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




class GenPool{
  // Generate population
  // Perform crossover
  // perform mutation
  // perform selection

public:
  GenPool(int width, int height, Point start, Point end):width(width), height(height), start(start), end(end) {}
  void populatePool(int size);

  // Returns currently best fitness
  float update(int iterations = 100);

// private:
  vector<genome> gens;
  const int width;
  const int height;
  const Point start;
  const Point end;

  void crossover();
  void mutation();
  void selection();
  float calFittness(struct genome gen);
  float calOcc(struct genome gen);
  float calTime(struct genome gen, int speed = 5);
  struct genome getBest();
};

bool compareFitness(const struct genome &genA, const struct genome &genB){
  return genA.fitness > genB.fitness;
}

struct genome GenPool::getBest(){
  sort(gens.begin(), gens.end(), compareFitness);
  return gens[0];
}

void GenPool::populatePool(int size)
{
  // Create initial population of given Size

  for (int i=0; i<size; ++i) {
    struct genome gen;
    gen.map = make_shared<Mat>(Mat(height, width, CV_8U, Scalar(0)));
    gen.waypoints = genWaypoints(width, height, start);
    gen.waypoints.push_back(end);

    gen.fitness = this->calFittness(gen);
    this->gens.push_back(gen);
  }
}

float GenPool::calFittness(struct genome gen)
{
  // Maximize occ minimize time
  return calOcc(gen) - calTime(gen)/100;
}

float GenPool::calOcc(struct genome gen)
{

  *gen.map = Scalar(0);
  // cout << "Initial cost: " << cv::sum(map) << endl;

  auto iter = gen.waypoints.begin();
  Point current = *iter;
  iter++;
  do{
    MyLine(*gen.map, current, *iter);
    current = *iter;
    // cout << current << endl;
    iter++;
  } while(iter != gen.waypoints.end());
  return cv::sum(*gen.map)[0] / 100; // / (width * height);
}


float GenPool::calTime(struct genome gen, int speed)
{

  float dist = 0;
  auto iter = gen.waypoints.begin();
  Point current = *iter;
  iter++;
  do{
    // MyLine(map, current, *iter);
    // Sum up all distances the roboter needs to travel
    dist += cv::norm(current - *iter);
    current = *iter;
    // cout << current << endl;
    iter++;
  } while(iter != gen.waypoints.end());

  return dist / speed;

}

// Return elements of vec between start and stop index
template<typename T> vector<T> slice(vector<T>& vec, int start_idx, int stop_idx){

  return vector<T>(vec.begin() + start_idx, vec.begin() + stop_idx);
}

//Slice vec and delete sliced elements
template<typename T> vector<T> sliceErase(vector<T>& vec, int start_idx, int stop_idx){
  auto result = slice<T>(vec, start_idx, stop_idx);
  vec.erase(vec.begin() + start_idx, vec.begin() + stop_idx);
  return result;
}

// Append content of vecB to vecA
template<typename T> void joinSlices(vector<T>& vecA, vector<T>& vecB){
  vecA.reserve(vecA.size() + vecB.size());
  vecA.insert(vecA.end(), vecB.begin(), vecB.end());
}

void printWaypoints(vector<Point>& waypoints){
  for(const auto &i : waypoints){
    cout << i << endl;
  }
}

void GenPool::crossover()
{
  // get best two individuals
  // Assume that gens are already sorted according to their finess
  vector<Point> parent1 = gens[0].waypoints;
  vector<Point> parent2 = gens[1].waypoints;

  cout << "Parent1:" << endl;
  printWaypoints(parent1);
  cout << "Parent2:" << endl;
  printWaypoints(parent2);
  auto slice1 = sliceErase(parent1, (int) parent1.size()/2, parent1.size());
  auto slice2 = sliceErase(parent2, (int) parent2.size()/2, parent2.size());

  joinSlices(parent1, slice2);
  joinSlices(parent2, slice1);
  cout << "--------------------" << endl;
  cout << "Parent1:" << endl;
  printWaypoints(parent1);
  cout << "Parent2:" << endl;
  printWaypoints(parent2);

  // replace the worst 2 gens
  gens[gens.size()].waypoints = parent1;
  gens[gens.size()-1].waypoints = parent1;

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

  vector<Point> waypoints = genWaypoints(width, height, start);

  // auto occ_fit = cal_occ(waypoints, width, height);
  // auto time_fit = cal_time(waypoints, 3);



  GenPool pool(width, height, start, stop);
  pool.populatePool(20);
  // cout << "Fitness: " << pool.getBest().fitness << endl;
  // for(const auto i : pool.gens){
  //   cout << i.fitness << endl;
  // }
  pool.crossover();



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
    Scalar( 1 ),
    thickness,
    lineType );
}
