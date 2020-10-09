#include "environment/robot.hpp"
#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "environment/ocm.hpp"

using namespace std;

int main(int argc, char *argv[])
{
  Robot rob;
  foo::OccupancyMap ocm(10, 10);

  cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  cv::imshow("Helllo", *ocm.get_ocm());
  cv::waitKey();

  printf("Hello Rob: %d\n", rob.hello());
  return 0;
}
