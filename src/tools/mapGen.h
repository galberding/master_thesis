#ifndef MAPGEN_H
#define MAPGEN_H

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include "debug.h"
#include "path_tools.h"

using namespace grid_map;
using namespace path;

namespace mapgen {

  std::shared_ptr<grid_map::GridMap> generateMapType(int with, int height, float res, int type, grid_map::Position& start);

  void changeMapRes(GridMap& gmap, float res);
  void actionToPath(PAs& pas, vector<Position> &path);
  void drawPathOnMap(GridMap& gmap, vector<Position> &path);

}

#endif /* MAPGEN_H */
