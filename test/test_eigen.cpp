#include <gtest/gtest.h>
// #include "../src/optimizer/ga_path_generator.h"
#include "../src/optimizer/optimizer.h"
#include "../src/tools/debug.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
// #include "../src/optimizer/grid_search.h"
#include "opencv2/imgproc.hpp"
#include <yaml-cpp/yaml.h>

using namespace op;

using Eigen::MatrixXd;
using namespace path;

TEST(Eigen, test){
  genome gen, gen2;

  executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
  vector<Position> pos = {Position(0,0), Position(1,0), Position(1,1)};
  mapgen::drawPathOnMap(eConf.gmap, pos, true);


  vector<Position> pos2 = {Position(4,4), Position(4,0), Position(3,3)};

  gen.mat = make_shared<Matrix>(Matrix(eConf.gmap->get("map")));
  debug("Sum1:", gen.mat->sum());
  mapgen::drawPathOnMap(eConf.gmap, pos2, true);

  gen2.mat = make_shared<Matrix>(Matrix(eConf.gmap->get("map")));
  debug("Sum2:", gen2.mat->sum());

  // cv::Mat img = mapgen::gmapToImg(eConf.gmap, "map");
  cv::Mat img, img2, hist;
   Matrix res = *gen.mat+*gen2.mat;
  // cv::eigen2cv(res, img);


  // cv::eigen2cv(, img2);

  cv::imshow("Test", img);
  // cv::imshow("Test2", img2);
  // cv::waitKey();

}


TEST(Eigen, diversity){
  op::InitStrategy init;
  op::FitnessStrategy fit;
  executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
  Genpool pool;

  auto rob = make_shared<Robot>(Robot(eConf.rob_conf,
				     eConf.gmap,
				     eConf.obstacleName));

  init(pool, eConf);
  fit(pool, *rob, eConf);
  float mean, stdev;
  mean = stdev = 0;
  Eigen::MatrixXf D(pool.size(), pool.size());
  Eigen::VectorXf v(pool.size()*(pool.size()+1)/2);
  calDistanceMat(pool, D, v);
  genome_tools::getDivMeanStd(pool, mean, stdev);
  cv::Mat img, hist;
  // cv::eigen2cv(D, img);

  // Calculate histogram
  int histSize = 10;
  float range[] = { 0, 256 }; //the upper boundary is exclusive
  const float* histRange = { range };
  bool uniform = true, accumulate = false;
  cv::calcHist(&img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
  int hist_w = 512, hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );
  cv::Mat histImage( hist_h, hist_w, CV_8UC1, cv::Scalar(0) );
  normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
  for( int i = 1; i < histSize; i++ )
    {
      line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ),
	    cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
	    cv::Scalar(255), 2, 8, 0  );
    }
  // cv::imshow("Distance", img);
  // cv::imshow("Histogram", histImage);
  // cv::imshow("RealHist", hist);
  // cv::waitKey();
  cout << D << endl;

  debug("Mean: ", mean, " Std: ", stdev);
}


TEST(SaveImages, MapScenarios){
  executionConfig eConf("../../../src/ros_optimizer/test/config.yml");
  mapgen::saveMap(to_string(eConf.mapType), eConf.gmap, "obstacle", 1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
