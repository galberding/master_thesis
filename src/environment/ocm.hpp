#ifndef __OPTI_OCM__
#define __OPTI_OCM__

#include <opencv2/opencv.hpp>
// #include <cstdlib>
#include <memory>

namespace foo{



using namespace cv;

  class OccupancyMap{

  public:
    OccupancyMap(uint width, uint height){

      this->ocm = std::make_shared<Mat>(Mat(width, height, CV_8UC3, Scalar(0,0,0)));

    }
    void init_map();

    std::shared_ptr<Mat> &get_ocm() { return this->ocm; };

  private:
    std::shared_ptr<Mat> ocm;
    // std::unique_ptr<, typename _Dp>
  };

} // namespace foo

#endif // __OPTI_OCM__
