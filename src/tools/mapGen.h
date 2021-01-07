#ifndef MAPGEN_H
#define MAPGEN_H

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include "debug.h"

namespace mapgen {

  std::shared_ptr<grid_map::GridMap> generateMapType(int with, int height, float res, int type, grid_map::Position& start);

}

#endif /* MAPGEN_H */
