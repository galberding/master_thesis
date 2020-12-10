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
    genome(float fitness):fitness(fitness){};
    genome(PAs actions):actions(actions){};
    PAs actions;
    WPs waypoints;
    float fitness = 0;
  };

  using Genpool = std::deque<genome>;
  using mutaPair = pair<void (*)(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator), int>;
  using Mutation_conf = map<string, mutaPair>;



  bool compareFitness(const struct genome &genA, const struct genome &genB);
  genome roulettWheelSelection(Genpool &currentPopulation, std::uniform_real_distribution<float> selDistr, std::mt19937 generator);

  int randRange(int lower, int upper);

///////////////////////////////////////////////////////////////////////////////
//                             Mutation Functions                            //
///////////////////////////////////////////////////////////////////////////////

  void addAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  void removeAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  void addAngleOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  void addDistanceOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  void swapRandomAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  // void swapRandomActionRegion(genome &gen);





///////////////////////////////////////////////////////////////////////////////
//                                     GA                                    //
///////////////////////////////////////////////////////////////////////////////

  struct GA {
    std::mt19937 generator;
    std::normal_distribution<float> distanceDistr, angleDistr;
    std::uniform_real_distribution<float> selectionDist;
    Mutation_conf muta_conf;


    GA(int seed, float distMu, float distDev, float angleMu, float angleDev, Mutation_conf muta_conf):
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
    virtual float calFitness(float cdist,
			      float dist,
			      int crossed,
			      float cSpeed_m_s,
			      float speed_m_s,
			      int freeSpace);


  };
}
#endif /* GA_PATH_GENERATOR_H */
