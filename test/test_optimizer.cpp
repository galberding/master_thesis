#include <gtest/gtest.h>
#include "../src/optimizer/ga_path_generator.h"
#include "../src/tools/debug.h"

using namespace ga;
using namespace path;

class GATest : public ::testing::Test{
protected:
  void SetUp() override {
    ga = make_shared<GA>(GA(42, 0, 70, 0, 90));


  }
  shared_ptr<GA> ga;
};


TEST_F(GATest, distroTest){
  // Check if distributions behave as expected
  std::map<int, int> hist{};
  for(int n=0; n<100000; ++n) {
    ++hist[std::round(ga->distanceDistr(ga->gen))];
  }
  for(auto p : hist) {
    std::cout << std::setw(2)
	      << p.first << ' ' << std::string(p.second/200, '*') << '\n';
  }

  int hello = 42;
  // warn("Max: ", 42);
  debug("blub", hello, "hh");
}

// template<class... Args>
// void print2(Args... args) {
//   std::cout << CYAN << "[DEBUG]" << RESET  << ": ";
//   (std::cout << ... << args) << "\n";
// }

TEST(WTF, why){
  int hello = 42;
  debug("blub", hello, "hh");

  print2("blub2", "etess", "dasdasd");

  debug("blub", hello); // does not compile
}

  int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
