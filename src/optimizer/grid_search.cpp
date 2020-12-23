#include "grid_search.h"

using namespace ga;


void gsearch::Searcher::search(ga::executionConfig config) {
  // What should search do
  // Search in range of different population sizes at First


  GA(42, config);



}

shared_ptr<GridMap> gsearch::Searcher::generateMapType(int with, int height, float res, int type) {
  GridMap map;
  map.setGeometry(Size(height, with), res);
  switch(type){
  case 0:{ // empty
    map.add("obstacle", 0);
    break;
  }
  case 1:{ // bounds
    map.add("obstacle", 1);
    break;
  }
  case 2:{ // Center square
    break;
  }
  default:{
    warn("Unkown map type -- use empty map");
    map.add("obstacle", 0);
  }
  }

  return make_shared<GridMap>(map);
}
