#include "environment/robot.hpp"
#include "tools/path_tools.h"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <experimental/random>
// #include "environment/ocm.hpp"
#include <iterator>
// #include <geometric_shapes/shapes.h>

#include <cstring>
#include <string>

#include "pool.hpp"
// #include <boost/timer/timer.hpp>
#include <cmath>
#include <stack>
#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
// #include <grid_map/grid_map.hpp>

using namespace std;

using namespace cv;
using namespace grid_map;

#define w 400

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
using namespace octomap;


void MyEllipse( Mat img, double angle )
{
  int thickness = 2;
  int lineType = 8;
  ellipse( img,
	   Point( 100, 100 ),
       cv::Size( 10, 5 ),
       angle,
       90,
       360,
       Scalar(255),
       thickness,
       4 );
}



void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}



int main(int argc, char *argv[])
{
  Robot rob;
  // foo::OccupancyMap ocm(10, 10);
  // cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  const uint32_t width = 500;
  const uint32_t height = 500;
  const Point start(0,50);
  const Point stop(height, 50);
  const int rob_size = 30;
  const float rob_speed = 10;
  int offspringCount = 25;
  uint8_t im[height][width] =  { 0 };
  uint8_t im2[height][width] =  { 0 };
  uint8_t im3[height][width] =  { 0 };
  Mat map(height, width, CV_8U, im);
  Mat map2(height, width, CV_8U, im2);
  Mat map3(height, width, CV_8U, im3);

  vector<Point> waypoints = opti_ga::genWaypoints(width, height, start, 10);

  // opti_ga::GenPool pool(width, height, start, stop, rob_size, rob_speed, offspringCount);
  // pool.populatePool(1000, 5);
  // pool.update(10000);
  // grid_map::GridMap gmap;
  // gmap.setFrameId("map");
  // gmap.setGeometry(Length(width,height), 1);
  // cout << gmap.getSize()(0) << endl;

  // // grid_map::GridMapRosConverter::initializeFromImage(const sensor_msgs::Image &image, const double resolution, grid_map::GridMap &gridMap)
  // // cout << "Img to map: " << grid_map::GridMapCvConverter::initializeFromImage(map3, 0.01, gmap, grid_map::Position(0,0)) << endl;
  // cout << "Img to map: " <<  GridMapCvConverter::addLayerFromImage<unsigned short, 1>(map3, "path", gmap) << endl;


  // grid_map::Index idx_start(18, 2);
  // grid_map::Index idx_end(2, 13);
  // for(grid_map::LineIterator it(gmap,idx_start, idx_end); !it.isPastEnd(); ++it){
  //   cout << *it << endl;
  //   gmap.at("path", *it) = 255;
  // }

  // // for(grid_map::EllipseIterator it())

  // // for(auto &lay : gmap.getLayers()){
  // //   cout << "Layer: " <<  lay << endl;
  // // }
  // cout << "Before" << "\n";
  // GridMapCvConverter::toImage<unsigned char, 1>(gmap, "path", CV_8U, 0, 255, map3);
  // cout << "After" << "\n";


  // MyEllipse(map3, 0);


  // imshow("Final Image", map3);
  // waitKey();

  cout << path::angleToDir(190) << "\n";

  cout << endl;
  cout << "generating example map" << endl;

  OcTree tree (0.1);  // create empty tree with resolution 0.1


  // insert some measurements of occupied cells

  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  cout << endl;
  cout << "performing some queries:" << endl;

  point3d query (0., 0., 0.);
  OcTreeNode* result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(1.,1.,1.);
  result = tree.search (query);
  print_query_info(query, result);


  cout << endl;
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;



  // Point A(2,2), B(1,2);

  // cout << A.dot(B) / (norm(A)*norm(B)) << endl;

  return 0;
}
