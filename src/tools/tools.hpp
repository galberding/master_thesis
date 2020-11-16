#ifndef __OPTI_TOOLS__
#define __OPTI_TOOLS__
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>


namespace tools {
  using namespace std;

  template<typename T>
  struct genome {
    shared_ptr<cv::Mat> map;
    shared_ptr<vector<shared_ptr<T>>> sequence;
    double time = 0;
    double occ = 0;
    double fitness = 0;

    genome<T> (){};
    genome<T> (shared_ptr<T> sequence, shared_ptr<cv::Mat> map):sequence(sequence), map(map){};
  };



  namespace debugging {
    #ifndef __DEBUGGING__
    #define __DEBUGING__ true
    #endif

    void printFitness(vector<genome> &gens, bool first=false);
    void printWaypoints(vector<cv::Point>& waypoints);
  }
  namespace path {

  }
  namespace rand_events {
  }
  namespace para_search {}
  namespace exception {}
  namespace config {}
}

#endif //__OPTI_TOOLS__
