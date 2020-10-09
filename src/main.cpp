#include "environment/robot.hpp"
#include <stdio.h>

#include "environment/ocm.hpp"

using namespace std;

int main(int argc, char *argv[])
{
  Robot rob;
  OccupancyMap ocm(10, 10);

  cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  imshow("Helllo", *ocm.get_ocm());
  waitKey();

  printf("Hello Rob: %d\n", rob.hello());
  return 0;
}
