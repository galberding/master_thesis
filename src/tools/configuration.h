#ifndef __TOOLS_CONFIG__
#define __TOOLS_CONFIG__

#include "path_tools.h"
#include "genome_tools.h"
#include "mapGen.h"
#define MIN_CROSS_LEN 4
using namespace genome_tools;

namespace conf {


  struct executionConfig {
    executionConfig(){} // default constructor
    executionConfig(string dir, string name, shared_ptr<GridMap> gmap, Position start, vector<Position> ends)
      :logDir(dir),
       logName(name),
       gmap(gmap),
       start(start),
       ends(ends){
      fitnessStr = make_shared<std::ostringstream>(std::ostringstream());
      logStr = make_shared<std::ostringstream>(std::ostringstream());
      generator.seed(42);
      ends = {start};
    }
    executionConfig(const string loadPath){
      loadConfFromYaml(loadPath);
      gmap = mapgen::generateMapType(mapWidth, mapHeight, mapResolution, mapType, start);
      fitnessStr = make_shared<std::ostringstream>(std::ostringstream());
      logStr = make_shared<std::ostringstream>(std::ostringstream());
      generator.seed(42);
      ends = {start};
    }


    // Logger
    string logDir = "";
    // string logFitness = "";
    string logName = "";

    bool visualize = true;
    bool printInfo = true;
    int scenario = 0;
    int clearZeros = 0;
    bool penalizeZeroActions = true;
    bool penalizeRotation = false;
    int funSelect = 3;

    // Snapshots
    bool restore = false;
    string snapshot = "";
    bool takeSnapshot = true;
    int takeSnapshotEvery = 1;
    string tSnap = "pool.actions";
    string tPerformanceSnap = "pool.performance";

    float actionLenMin = 0;
    float actionLenMax = 0;
    float actionLenAvg = 0;

    float zeroActionPercent = 0;
    float deadGensCount = 0;

    // Diversity:
    float diversityMean = 0;
    float diversityStd = 0;

    // If parameter is set we want to store it under this filename
    string fitnessName = "vv";
    float fitnessWeight = 0.5;
    // Those values will be updated after each iteration
    float fitnessMax = 0;
    float fitnessMin = 0;
    float fitnessAvg = 0;
    float fitnessAvgTime = 0;
    float fitnessMaxTime = 0;
    float fitnessMinTime = 0;
    float fitnessAvgCoverage = 0;
    float fitnessMaxCoverage = 0;
    float fitnessMinCoverage = 0;
    float fitnessAvgAngleCost = 0;
    float fitnessMaxAngleCost = 0;
    float fitnessMinAngleCost = 0;

    shared_ptr<std::ostringstream> fitnessStr;
    shared_ptr<std::ostringstream> logStr;

    // Robot Config
    // TODO: what to do with those parameter
    string obstacleName = "map";
    rob_config rob_conf = {
      {RobotProperty::Width_cm, 1},
      {RobotProperty::Height_cm, 1},
      {RobotProperty::Drive_speed_cm_s, 50},
      {RobotProperty::Clean_speed_cm_s, 20}};

    // Map
    std::mt19937 generator;
    int mapType = 1;
    int mapWidth = 10;
    int mapHeight = 10;
    float mapResolution = 0.3;


    shared_ptr<GridMap> gmap;

    // GA
    int currentIter = 0;
    int maxIterations = 1000;
    int initIndividuals = 1000;
    int initActions = 50;
    Position start;
    vector<Position> ends = {Position()};

    // Fitness
    vector<float> fitnessWeights = {0.3, 0.05, 0.65};

    // Selection
    int selectIndividuals = 10;
    int selectKeepBest = 0;
    int tournamentSize = 2;
    genome best;
    // Switch between roulettWheelSelection or best N selection
    bool toggleRoulettSelectionOn = false;

    // crossover
    // Length of the segment that will be transferred to the other gen
    float crossLength = 0.4;
    float overallDMax = 0;
    float crossoverProba = 0.8;
    int crossAdapter = 0;
    float crossBestFit = 0;
    int crossChildSelector = 2;

    // Mutation functions, configurations
    float mutaOrtoAngleProba = 0.1;
    float mutaRandAngleProba = 0.1;
    float mutaPosDistProba = 0.1;
    float mutaNegDistProba = 0.01;
    float mutaRandScaleDistProba = 0.01;
    float mutaPosDistMax = 50;
    float mutaReplaceGen = 0.001;

    // Adaptive Parameter
    bool adaptParameter = false;
    float crossUpper = 0.8;
    float crossLower = 0.5;
    float cLenUpper = 0.5;
    float cLenLower = 0.1;
    float mutUpper = 0.1;
    float lastDmax = 0;

    // float mutUpperReplGen = 0.001;

    string config_to_string(){

      string str;
      str += argsToCsv("maxIterations", "initIndividuals", "selectIndividuals", "selectKeepBest", "time,occ,coverage");
      str += argsToCsv(maxIterations, initIndividuals, selectIndividuals, selectKeepBest, fitnessWeights[0], fitnessWeights[1], fitnessWeights[2]);

      return str;
    }

    bool loadConfFromYaml(const string path);
    shared_ptr<GridMap> generateMapType(int width, int height, float res, int type, Position& start);
    int getMinGenLen(){return MIN_CROSS_LEN / crossLength;}
    void adaptCrossover();
    void adaptMutation();
    void adaptCLen();
    // void adaptGenReplMutation();
  };

}

#endif /* __TOOLS_CONFIG__ */
