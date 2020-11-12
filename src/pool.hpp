#ifndef __OPTI_POOL__
#define __OPTI_POOL__

#include <memory>
#include <opencv2/opencv.hpp>
#include <experimental/random>
#include <cstdio>
#include <iostream>
#include <boost/timer/timer.hpp>
#include <chrono>
#include <stack>
#include <thread>
#include <future>
#include <exception>
#include <cassert>

using namespace cv;
using namespace std;

#define throw_line(msg) { throw std::runtime_error(msg + string("\n") + string(__FILE__) + ":" + string(to_string(__LINE__)));}


namespace opti_ga {
  class Timer;
  struct genome
  {
    genome(){};
    genome(shared_ptr<cv::Mat> map, vector<Point> waypoints)
      :map(map),
       waypoints(waypoints){};

    shared_ptr<cv::Mat> map;
    vector<Point> waypoints;
    double fitness = 0;
  };


  //Helper functions:
  //--------------------------------------------------
  Point randomPointShift(Point p, int magnitude = 200);
  vector<Point> genWaypoints(int width, int height, Point start, int n = 10);
  void printFitness(vector<genome> &gens, bool first=false);
  void printWaypoints(vector<Point>& waypoints);

  template<typename T>
  vector<T> slice(vector<T> vec, int start_idx, int stop_idx);

  template<typename T>
  vector<T> sliceErase(vector<T>& vec, int start_idx, int stop_idx);

  template<typename T>
  void joinSlices(vector<T>& vecA, vector<T>& vecB);
  bool compareFitness(const struct genome &genA, const struct genome &genB);
  void markOcc( Mat &img, Point &start, Point &end, int size, int val);
  void markPath(genome &gen);
  void makeTest();

  // --------------------------------------------------

#define QCM_TO_QM 10000


  class Timer{
    // chrono::steady_clock::time_point t_begin;
  public:
      std::unordered_map<string, int> summary;
    //  = {
    //   {"sel", 0},
    //   {"cross", 0},
    //   {"mut", 0},
    //   {"eval_time", 0},
    //   {"eval_occ", 0},
    //   {"mat", 0}
    // };
      std::stack<tuple<string, chrono::steady_clock::time_point>> times;
      void t_start(const string name){
	if(summary.count(name) <= 0){
	  summary.insert(std::make_pair(name,0));
	}
	times.push(make_tuple(name, chrono::steady_clock::now()));
      };

      void t_end(){
	auto name = get<0>(times.top());
	auto t_begin = get<1>(times.top());
	if(summary[name] == 0){
	  summary[name] = std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - t_begin).count();
	}else{
	  summary[name] = (summary[name] +
			std::chrono::duration_cast<std::chrono::microseconds>
			  (chrono::steady_clock::now() - t_begin).count()) / 2;
	}
	// summary[name] = (summary[name] +
	// 		std::chrono::duration_cast<std::chrono::microseconds>
	// 		  (chrono::steady_clock::now() - t_begin).count()) / 2;
	times.pop();
	// t_begin = chrono::steady_clock::now();
    };

      void printTiming(){
	for(auto const& [key, val] : summary){
	  cout << key << " --\t " << val << " µs" << endl;
	}
      }

  };




  class GenPool : public Timer{
    // Generate population
    // Perform crossover
    // perform mutation
    // perform selection

  public:
    GenPool(int width, int height, Point start, Point end, int robot_size, float robot_speed, int offspringCount=15):width(width), height(height), start(start), end(end), robot_size(robot_size), robot_speed(robot_speed), shift_mag(20), distribution(0.0, 1.0), randgen(-shift_mag, shift_mag), offspringCount(offspringCount){


      estimation = (width * height) / robot_size / QCM_TO_QM / (robot_speed / 3.6);
      cout << "Estimated time: " << estimation << endl;
    }
    friend Timer;
    void populatePool(int size, int waypoints);

    // Returns currently best fitness
    float update(int iterations = 100);


    // private:
    vector<genome> gens;
    vector<genome> matingPool;
    vector<shared_ptr<cv::Mat>> unusedMaps;
    const int width;
    const int height;
    const Point start;
    const Point end;
    const int shift_mag;
    float estimation;
    int offspringCount;
    const int robot_size;
    const float robot_speed;
    // boost::timer::cpu_timer timer;
    double fitnessWeight = 0.5;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution, randgen;

    void crossover(genome &gen1, genome &gen2);
    void addGen(vector<Point> &waypoints);
    void mutation();
    // TODO: proper selection method
    void selection();
    void updateFitness();
    double calFittness(struct genome &gen);
    double calOcc(struct genome &gen);
    double calTime(struct genome &gen, int speed = 3);
    int detUncoverdRegions(Mat &map);
    struct genome getBest();
    void conditionalPointShift(Point &p, int magnitude = 30);
    void randomInsert(struct genome &gen);
    void randomRemove(struct genome &gen);
    void randomSwitch(struct genome &gen);
    genome roulettWheelSelection();
    bool eventOccurred(double probability);
    string pool_to_string()
    {
      ostringstream str;
      for(auto gen: gens){
	str << "--------------------" << endl;
	for(auto &w: gen.waypoints){
	  str << w << " | ";
	}
	str << endl;
	str << "Fitness: " << gen.fitness << endl;
      }

      return str.str();
    }

  };



}

#endif //__OPTI_POOL__
