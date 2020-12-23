#ifndef GA_PATH_GENERATOR_H
#define GA_PATH_GENERATOR_H
#include <random>
#include <cmath>
#include <algorithm>
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
    bool operator < (const genome& gen) const
    {
        return (fitness < gen.fitness);
    }

    int id = 0;
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
  void validateGen(genome &gen);
///////////////////////////////////////////////////////////////////////////////
//                             Mutation Functions                            //
///////////////////////////////////////////////////////////////////////////////

  void addAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  void removeAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  void addAngleOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  void addDistanceOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  void swapRandomAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 generator);
  // void swapRandomActionRegion(genome &gen);



  struct executionConfig {

    executionConfig(string dir, string name, shared_ptr<GridMap> gmap):logDir(dir), logName(name), gmap(gmap){}
    // Logger
    string logDir = "";
    string logName = "";

        // Robot Config
    string obstacleName = "map";
    rob_config rob_conf = {
      {RobotProperty::Width_cm, 1},
      {RobotProperty::Height_cm, 1},
      {RobotProperty::Drive_speed_cm_s, 50},
      {RobotProperty::Clean_speed_cm_s, 20}};

    // Map
    shared_ptr<GridMap> gmap;

    // GA
    int maxIterations = 10000;
    int initIndividuals = 200;
    int initActions = 50;
    // Selection
    int selectIndividuals = 25;
    int selectKeepBest = 10;



    // Mutation
    vector<string> mutaFunctions = {"addAction", "removeAction"};
    float distMu = 4;
    float distDev = 0.9;
    float angleMu = 0;
    float angleDev = 40;

    string config_to_string(){
      string str;
      str.append("name,");
      str.append("maxIterations,");
      str.append("initIndividuals,");
      str.append("initIndividuals,");
      str.append("initActions,");
      str.append("selectIndividuals,");
      str.append("selectKeepBest,");
      for (auto &mut : mutaFunctions){
	str.append(mut+" ");
      }
      str.append(",");
      str.append("distMu,");
      str.append("distDev,");
      str.append("angleMu,");
      str.append("angleDev,");
      str.append("obstacleName\n");
      str.append(logDir+ "/" + logName+",");
      str.append(to_string(maxIterations)+",");
      str.append(to_string(initIndividuals)+",");
      str.append(to_string(initIndividuals)+",");
      str.append(to_string(initActions)+",");
      str.append(to_string(selectIndividuals)+",");
      str.append(to_string(selectKeepBest)+",");
      str.append(to_string(distMu)+",");
      str.append(to_string(distDev)+",");
      str.append(to_string(angleMu)+",");
      str.append(to_string(angleDev)+",");
      str.append(obstacleName+"\n");
      return str;
    }

  };



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
    GA(int seed, executionConfig conf);

    virtual void populatePool(Genpool &currentPopuation, Position start, WPs endpoints, int individuals, int initialActions);
    virtual void selection(Genpool &currentPopuation, Genpool &selection, int individuals, int keepBest = 0);
    virtual void crossover(Genpool &currentSelection, Genpool &newPopulation);
    virtual void mating(genome &par1, genome &par2, Genpool& newPopulation);
    virtual void mutation(Genpool& currentPopulation, Mutation_conf& muat_conf);
    virtual void evalFitness(Genpool &currentPopulation, Robot &rob);
    virtual float calFitness(float cdist,
			      float dist,
			      int crossed,
			      float cSpeed_m_s,
			      float speed_m_s,
			      int freeSpace);
    void optimizePath(executionConfig conf, GridMap obstacle);
    void gridSearch();


  };
}
#endif /* GA_PATH_GENERATOR_H */
