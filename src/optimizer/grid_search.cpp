#include "grid_search.h"

using namespace ga;


void gsearch::Searcher::search(ga::executionConfig config) {
  // What should search do
  // Search in range of different population sizes at First


  GA(42, config);



}

shared_ptr<GridMap> gsearch::Searcher::generateMapType(int width, int height, float res, int type) {
  GridMap map;
  map.setGeometry(Length(height, width), res);
  switch(type){
  case 0:{ // empty
    map.add("obstacle", 0);
    break;
  }
  case 1:{ // bounds
    map.add("obstacle", 255);
    int h_offset = static_cast<int>(map.getSize()(0)*0.1);
    int w_offset = static_cast<int>(map.getSize()(1)*0.1);


    Index submapStartIndex(h_offset, w_offset);
    Index submapBufferSize(
			   static_cast<int>(map.getSize()(0) -2*h_offset),
			   static_cast<int>(map.getSize()(1) - 2*w_offset)
			   );


    for (grid_map::SubmapIterator iterator(map,submapStartIndex, submapBufferSize);
      !iterator.isPastEnd(); ++iterator) {
    map.at("obstacle", *iterator) = 0;

    }
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
