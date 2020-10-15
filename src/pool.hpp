#ifndef __OPTI_POOL__
#define __OPTI_POOL__

#include <memory>
#include <opencv2/opencv.hpp>
#include <experimental/random>
#include <cstdio>
#include <iostream>
#include <boost/timer/timer.hpp>
#include <chrono>

using namespace cv;
using namespace std;


namespace opti_ga {
  struct genome
  {
    shared_ptr<cv::Mat> map;
    vector<Point> waypoints;
    float fitness = 0;
  };


  //Helper functions:
  //--------------------------------------------------
  Point randomPointShift(Point p, int magnitude = 200);
  vector<Point> genWaypoints(int width, int height, Point start, int n = 10);
  void printFitness(vector<genome> &gens);
  void printWaypoints(vector<Point>& waypoints);

  template<typename T>
  vector<T> slice(vector<T> vec, int start_idx, int stop_idx);

  template<typename T>
  vector<T> sliceErase(vector<T>& vec, int start_idx, int stop_idx);

  template<typename T>
  void joinSlices(vector<T>& vecA, vector<T>& vecB);
  bool compareFitness(const struct genome &genA, const struct genome &genB);
  void markOcc( Mat &img, Point &start, Point &end, int val = 255, int size=10);
  void markPath(genome &gen);
  // --------------------------------------------------

#define QCM_TO_QM 10000

  class GenPool{
    // Generate population
    // Perform crossover
    // perform mutation
    // perform selection

  public:
    GenPool(int width, int height, Point start, Point end, int robot_size, float robot_speed):width(width), height(height), start(start), end(end), robot_size(robot_size), robot_speed(robot_speed) {


      estimation = (width * height) / robot_size / QCM_TO_QM / (robot_speed / 3.6);
      cout << "Estimated time: " << estimation << endl;
    }
    void populatePool(int size, int waypoints);

    // Returns currently best fitness
    float update(int iterations = 100);

    // private:
    vector<genome> gens;
    const int width;
    const int height;
    const Point start;
    const Point end;
    float estimation;
    const int robot_size;
    const float robot_speed;
    boost::timer::cpu_timer timer;
    std::unordered_map<string, int> summary = {
      {"sel", 0},
      {"cross", 0},
      {"mut", 0},
      {"eval_time", 0},
      {"eval_occ", 0},
    };

    void crossover();
    void mutation();
    // TODO: proper selection method
    void selection();
    double calFittness(struct genome &gen);
    float calOcc(struct genome &gen);
    float calTime(struct genome &gen, int speed = 3);
    struct genome getBest();
    void conditionalPointShift(Point &p, int magnitude = 30);
    void randomInsert(struct genome &gen);
    void randomRemove(struct genome &gen);

  };
}

#endif //__OPTI_POOL__
