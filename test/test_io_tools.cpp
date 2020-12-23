#include <gtest/gtest.h>
#include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"
#include <opencv2/opencv.hpp>

using namespace ga;
using namespace path;
namespace fs = std::filesystem;

TEST(Logger, createDirs){

  // string dir = "helloLog/";
  // string name = "helloLogFile";
  // fs::create_directory("loggertest");

  // logging::Logger("Hello,World", dir, name);
  // logging::Logger("Hello,World", dir, name);

  executionConfig ex("test", "name");
  debug(ex.config_to_string());

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
