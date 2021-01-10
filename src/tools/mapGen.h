#ifndef MAPGEN_H
#define MAPGEN_H

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include "debug.h"
// #include "path_tools.h"


// using namespace path;

namespace mapgen {
  using namespace grid_map;
  using namespace std;

  std::shared_ptr<grid_map::GridMap> generateMapType(int with, int height, float res, int type, grid_map::Position& start);

  shared_ptr<GridMap> changeMapRes(shared_ptr<GridMap> gmap, float res);
  void drawPathOnMap(shared_ptr<GridMap> gmap, vector<Position>& path, bool inc);

}

#endif /* MAPGEN_H */
