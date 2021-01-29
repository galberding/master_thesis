#include "../src/optimizer/optimizer.h"

// using namespace ga;
using namespace path;
namespace fs = std::filesystem;


int main(int argc, char *argv[])
{

  string path;
  debug(argc);
  if (argc == 2){
    path = argv[1];
  }else{
    warn("Wrong number of arguments, expected path to config!");
    return 1;
  }

  executionConfig eConf(path);
  op::Optimizer opti(
		     make_shared<op::InitStrategy>(op::InitStrategy()),
		     make_shared<op::SelectionStrategy>(op::SelectionStrategy()),
		     make_shared<op::DualPointCrossover>(op::DualPointCrossover()),
		     make_shared<op::MutationStrategy>(op::MutationStrategy()),
		     make_shared<op::FitnessStrategy>(op::FitnessStrategy()),
		     eConf);
  // opti.optimizePath(true);
  opti.optimizePath_s_tourn_c_dp(true);


  return 0;
}
