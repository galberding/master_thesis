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
using namespace logging;

namespace ga{

  struct genome{
    static int gen_id;
    genome():id(gen_id){gen_id++;};
    genome(float fitness):fitness(fitness),id(gen_id){gen_id++;};
    genome(PAs actions):actions(actions),id(gen_id){gen_id++;};
    bool operator < (const genome& gen) const
    {
        return (fitness < gen.fitness);
    }
    bool operator==(const genome& gen)const {
      return gen.id == id;
    }

    float getPathLen(){
      float len = 0;
      for(auto it = actions.begin(); it != actions.end(); it++){
	len += (*it)->mod_config[PAP::Distance];
      }
      return len / actions.size();
    }

    int id = 0;
    PAs actions;
    WPs waypoints;
    float fitness = 0;
  };

  using Genpool = std::deque<genome>;
  using mutaPair = pair<void (*)(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 &generator), int>;
  using Mutation_conf = map<string, mutaPair>;



  bool compareFitness(const struct genome &genA, const struct genome &genB);
  genome roulettWheelSelection(ga::Genpool &currentPopulation, float totalFitness, std::mt19937 &generator,  bool erase = true);

  int randRange(int lower, int upper);
  void validateGen(genome &gen);
  float calZeroActionPercent(genome &gen);
  float calZeroActionPercent(Genpool &pool);


  // struct executionConfig {
  //   executionConfig(){} // default constructor
  //   executionConfig(string dir, string name, shared_ptr<GridMap> gmap, Position start, vector<Position> ends)
  //     :logDir(dir),
  //      logName(name),
  //      gmap(gmap),
  //      start(start),
  //      ends(ends){
  //     fitnessStr = make_shared<std::ostringstream>(std::ostringstream());
  //     logStr = make_shared<std::ostringstream>(std::ostringstream());
  //     generator.seed(42);
  //   }
  //   // Logger
  //   string logDir = "";
  //   // string logFitness = "";
  //   string logName = "";

  //   // Snapshots
  //   bool restore = false;
  //   string snapshot = "";
  //   bool takeSnapshot = false;
  //   string tSnap = "";

  //   float actionLenMin = 0;
  //   float actionLenMax = 0;
  //   float actionLenAvg = 0;

  //   // If parameter is set we want to store it under this filename
  //   string fitnessName = "vv";
  //   float fitnessWeight = 0.5;
  //   // Those values will be updated after each iteration
  //   float fitnessMax = 0;
  //   float fitnessMin = 0;
  //   float fitnessAvg = 0;
  //   float fitnessAvgTime = 0;
  //   float fitnessAvgOcc = 0;
  //   float fitnessAvgCoverage = 0;

  //   shared_ptr<std::ostringstream> fitnessStr;
  //   shared_ptr<std::ostringstream> logStr;

  //   // Robot Config
  //   string obstacleName = "map";
  //   rob_config rob_conf = {
  //     {RobotProperty::Width_cm, 1},
  //     {RobotProperty::Height_cm, 1},
  //     {RobotProperty::Drive_speed_cm_s, 50},
  //     {RobotProperty::Clean_speed_cm_s, 20}};

  //   // Map
  //   std::mt19937 generator;
  //   shared_ptr<GridMap> gmap;

  //   // GA
  //   int currentIter = 0;
  //   int maxIterations = 1000;
  //   int initIndividuals = 1000;
  //   int initActions = 50;
  //   Position start;
  //   vector<Position> ends;

  //   // Fitness
  //   vector<float> fitnessWeights = {0.3, 0.05, 0.65};

  //   // Selection
  //   int selectIndividuals = 10;
  //   int selectKeepBest = 0;
  //   // Switch between roulettWheelSelection or best N selection
  //   bool toggleRoulettSelectionOn = false;

  //   // crossover
  //   // Length of the segment that will be transferred to the other gen
  //   float crossLength = 0.4;
  //   float crossoverProba = 0.8;

  //   // Mutation
  //   vector<string> mutaFunctions = {"addAction", "removeAction", "addAngleOffset", "addDistanceOffset"};
  //   // TODO: Not used can be removed
  //   vector<int> probas = {10, 10};
  //   Mutation_conf muta = {
  //     // {"addAction", make_pair(addAction, 10)},
  //     // {"removeAction", make_pair(removeAction, 10)},
  //     // {"addAngleOffset", make_pair(addAngleOffset, 70)},
  //     // {"addDistanceOffset", make_pair(addDistanceOffset, 70)},
  //     // {"swapRandomAction", make_pair(swapRandomAction, 10)},
  //   };
  //   float distMu = 4;
  //   float distDev = 0.9;
  //   float angleMu = 0;
  //   float angleDev = 40;

  //   // Mutation functions, configurations
  //   float mutaOrtoAngleProba = 0.1;
  //   float mutaRandAngleProba = 0.1;
  //   float mutaPosDistProba = 0.1;
  //   float mutaNegDistProba = 0.01;
  //   float mutaPosDistMax = 50;

  //   string config_to_string(){

  //     string str;
  //     str += argsToCsv("maxIterations", "initIndividuals", "selectIndividuals", "selectKeepBest", "time,occ,coverage");
  //     str += argsToCsv(maxIterations, initIndividuals, selectIndividuals, selectKeepBest, fitnessWeights[0], fitnessWeights[1], fitnessWeights[2]);

  //     return str;
  //   }

  // };


///////////////////////////////////////////////////////////////////////////////
//                             Mutation Functions                            //
///////////////////////////////////////////////////////////////////////////////

  void addAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 &generator);
  void removeAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 &generator);
  void addAngleOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 &generator);
  void addDistanceOffset(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 &generator);
  void swapRandomAction(genome &gen, std::normal_distribution<float> angleDist, std::normal_distribution<float> distanceDist, std::mt19937 &generator);
  // void swapRandomActionRegion(genome &gen);

  void addOrthogonalAngleOffset(genome& gen, executionConfig &eConf, std::mt19937& generator);
  void addRandomAngleOffset(genome& gen, executionConfig &eConf, std::mt19937& generator);
  void addPositiveDistanceOffset(genome& gen, executionConfig &eConf, std::mt19937& generator);
  void addNegativeDistanceOffset(genome& gen, executionConfig &eConf, std::mt19937& generator);





///////////////////////////////////////////////////////////////////////////////
//                                     GA                                    //
///////////////////////////////////////////////////////////////////////////////

  struct GA {
    std::mt19937 generator;
    std::normal_distribution<float> distanceDistr, angleDistr;
    std::uniform_real_distribution<float> selectionDist;
    Mutation_conf muta_conf;
    executionConfig eConf;
    genome secretBest;



    GA(int seed, float distMu, float distDev, float angleMu, float angleDev, Mutation_conf muta_conf):
      generator(seed),
      distanceDistr{distMu, distDev},
      angleDistr{angleMu, angleDev},
      selectionDist{0,1},
      muta_conf(muta_conf){};

    GA(int seed, executionConfig& conf)
      :generator(seed),
       distanceDistr{conf.distMu, conf.distDev},
       angleDistr{conf.angleMu, conf.angleDev},
       selectionDist{0,1},
       muta_conf(conf.muta),
       eConf(conf){}

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
    virtual void optimizePath(bool display = false);
    void gridSearch();




  };

  /////////////////////////////////////////////////////////////////////////////
  //                           Dual Point Crossover                          //
  /////////////////////////////////////////////////////////////////////////////

  struct _Dual_Point_Crossover : GA{
    // Use constructor of GA
    using GA::GA;
    virtual void selection(Genpool &currentPopuation, Genpool &selection, int individuals, int keepBest = 0) override;
    virtual void mating(genome &par1, genome &par2, Genpool& newPopulation) override;
    virtual void crossover(Genpool &currentSelection, Genpool &newPopulation) override;
    virtual void mutation(Genpool& currentPopulation, Genpool& nextPopulation);
    virtual void optimizePath(bool display=false) override;
  };



  /////////////////////////////////////////////////////////////////////////////
  //                                  GA V2                                   //
  /////////////////////////////////////////////////////////////////////////////

  struct GA_V2 : _Dual_Point_Crossover{
    // using _::GA;
    using _Dual_Point_Crossover::_Dual_Point_Crossover;
    // using _Mutation::_Mutation;
    // GA_V2(int seed, executionConfig& conf):GA(seed, conf){}
  };

}
#endif /* GA_PATH_GENERATOR_H */
