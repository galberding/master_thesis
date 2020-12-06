#ifndef GA_PATH_GENERATOR_H
#define GA_PATH_GENERATOR_H
#include <random>
#include <cmath>

#include "../tools/path_tools.h"

using namespace path;
namespace ga{

  struct genome{
    genome();
    genome(PAs actions):actions(actions){};
    PAs actions;
    WPs waypoints;
    double fitness = 0;
  };

  using Genpool = std::list<genome>;

  bool compareFitness(const struct genome &genA, const struct genome &genB);
  genome roulettWheelSelection(Genpool &currentPopulation, std::uniform_real_distribution<> selDistr, std::mt19937 generator);
  struct GA {
    std::mt19937 gen;
    std::normal_distribution<> distanceDistr, angleDistr;
    std::uniform_int_distribution<> selectionDist;

    GA(int seed, double distMu, double distDev, double angleMu, double angleDev):
      gen(seed),
      distanceDistr{distMu, distDev},
      angleDistr{angleMu, angleDev},
      selectionDist{0,1}{};

    virtual void selection(Genpool &currentPopuation, Genpool &selection);
    virtual void crossover(Genpool &currentSelection, Genpool &newPopulation);
    virtual void mutation(Genpool &currentPopulation);
    virtual void calFitness(Genpool &currentPopulation, Robot &rob);
  };


}

#endif /* GA_PATH_GENERATOR_H */
