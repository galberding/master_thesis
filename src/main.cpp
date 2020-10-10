#include "environment/robot.hpp"
#include <cstdio>
#include <opencv2/opencv.hpp>


#include "environment/ocm.hpp"

#include <geometric_shapes/shapes.h>

using namespace std;

int main(int argc, char *argv[])
{
  Robot rob;
  foo::OccupancyMap ocm(10, 10);
  cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  // cv::imshow("Helllo", *ocm.get_ocm());
  // cv::waitKey();
  shapes::Cylinder cy(2, 1);

  cout << cy.type << endl;

  // Optimization stage 1:
  // TODO: Waypoint generation
  // TODO: Geometric rotation and transformation on map


  // printf("Hello Rob: %d\n", rob.hello());
  return 0;
}
