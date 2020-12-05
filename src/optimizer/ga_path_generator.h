#ifndef GA_PATH_GENERATOR_H
#define GA_PATH_GENERATOR_H
#include <random>
#include <cmath>

#include "../tools/path_tools.h"

using namespace path;
namespace ga{

  struct genome{
    genome(PAs actions):actions(actions){};
    PAs actions;
    WPs waypoints;
    double fitness = 0;
  };

  using Genpool = std::deque<genome>;

  struct GA {
    std::mt19937 gen;
    std::normal_distribution<> distanceDistr, angleDistr;

    GA(int seed, double distMu, double distDev, double angleMu, double angleDev):
      gen(seed),
      distanceDistr{distMu, distDev},
      angleDistr{angleMu, angleDev}{};

    // virtual void selection(Genpool &currentPopuation, Genpool &selection);
    // virtual void crossover(Genpool &currentSelection, Genpool &newPopulation);
    // virtual void mutation(Genpool &currentPopulation);
  };


}

#endif /* GA_PATH_GENERATOR_H */
