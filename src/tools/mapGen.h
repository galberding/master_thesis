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
  /**
   * Two stages:
   * 1. Draw coverage pixel line, use the inc parameter for that -> inc == true
   * 2. Draw black line -> inc == false
   */
  void drawPathOnMap(shared_ptr<GridMap> gmap, vector<Position>& path, bool inc);
  /**
   * @brief      Generate OpenCV img from GridMap.
   *
   * @param      gmap shared pointer to grid map
   * @param      layer name of the layer
   * @param      upperThresh highest possible value in gridmap layer (used for visualization)
   *
   * @return     return OpenCV image
   */
  cv::Mat gmapToImg(const shared_ptr<GridMap> gmap, const string layer, uint8_t upperThresh=5);
  void saveMap(const string name, const shared_ptr<GridMap> gmap, const string layer, uint8_t upperThresh);
}

#endif /* MAPGEN_H */
