#ifndef GA_PATH_GENERATOR_H
#define GA_PATH_GENERATOR_H
#include <random>
#include <cmath>

#include "../tools/path_tools.h"

// #ifdef __DEBUG__
// #undef __DEBUG__
// #define __DEBUG__ true
// #endif

using namespace path;

namespace ga{

  struct genome{
    genome(){};
    genome(double fitness):fitness(fitness){};
    genome(PAs actions):actions(actions){};
    PAs actions;
    WPs waypoints;
    double fitness = 0;
  };

  using Genpool = std::vector<genome>;
  using mutaPair = pair<void (*)(genome &gen, std::normal_distribution<double> angleDist, std::normal_distribution<double> distanceDist, std::mt19937 generator), int>;
  using Mutation_conf = map<string, mutaPair>;



  bool compareFitness(const struct genome &genA, const struct genome &genB);
  genome roulettWheelSelection(Genpool &currentPopulation, std::uniform_real_distribution<> selDistr, std::mt19937 generator);

  int randRange(int lower, int upper);

///////////////////////////////////////////////////////////////////////////////
//                             Mutation Functions                            //
///////////////////////////////////////////////////////////////////////////////

  void addAction(genome &gen, std::normal_distribution<double> angleDist, std::normal_distribution<double> distanceDist, std::mt19937 generator);
  void removeAction(genome &gen, std::normal_distribution<double> angleDist, std::normal_distribution<double> distanceDist, std::mt19937 generator);
  void addAngleOffset(genome &gen, std::normal_distribution<double> angleDist, std::normal_distribution<double> distanceDist, std::mt19937 generator);
  void addDistanceOffset(genome &gen, std::normal_distribution<double> angleDist, std::normal_distribution<double> distanceDist, std::mt19937 generator);
  void swapRandomAction(genome &gen, std::normal_distribution<double> angleDist, std::normal_distribution<double> distanceDist, std::mt19937 generator);
  // void swapRandomActionRegion(genome &gen);





///////////////////////////////////////////////////////////////////////////////
//                                     GA                                    //
///////////////////////////////////////////////////////////////////////////////

  struct GA {
    std::mt19937 generator;
    std::normal_distribution<double> distanceDistr, angleDistr;
    std::uniform_real_distribution<double> selectionDist;
    Mutation_conf muta_conf;


    GA(int seed, double distMu, double distDev, double angleMu, double angleDev, Mutation_conf muta_conf):
      generator(seed),
      distanceDistr{distMu, distDev},
      angleDistr{angleMu, angleDev},
      selectionDist{0,1},
      muta_conf(muta_conf){};

    virtual void populatePool(Genpool &currentPopuation, Position start, WPs endpoints, int individuals, int initialActions);
    virtual void selection(Genpool &currentPopuation, Genpool &selection, int individuals);
    virtual void crossover(Genpool &currentSelection, Genpool &newPopulation);
    virtual void crossover(genome &par1, genome &par2, Genpool& newPopulation);
    virtual void mutation(Genpool& currentPopulation, Mutation_conf& muat_conf);
    virtual void evalFitness(Genpool &currentPopulation, Robot &rob);
    virtual double calFitness(double cdist,
			      double dist,
			      int crossed,
			      double cSpeed_m_s,
			      double speed_m_s,
			      int freeSpace);


  };
}
#endif /* GA_PATH_GENERATOR_H */
