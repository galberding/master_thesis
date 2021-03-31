#ifndef __TOOLS_CONFIG__
#define __TOOLS_CONFIG__

#include "path_tools.h"
#include "genome_tools.h"
#include "mapGen.h"
#include <limits>
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
      warn("Robot properties are not set correctly!");
    }
    executionConfig(const string loadPath){
      loadConfFromYaml(loadPath);
      // mapResolution = Rob_width;
      // debug("Map Res: ", mapResolution);
      gmap = mapgen::generateMapType(mapWidth, mapHeight, mapResolution, Rob_width, mapType, start);
      fitnessStr = make_shared<std::ostringstream>(std::ostringstream());
      logStr = make_shared<std::ostringstream>(std::ostringstream());
      generator.seed(genSeed);
      ends = {start};
      Rob_angleSpeed = 2* M_PI * Rob_RPM * 1.0 / 60.0;
      // TODO :Possible that values will be lost
      rob_conf = {
	{RobotProperty::Width, Rob_width},
	{RobotProperty::Height, Rob_width},
	{RobotProperty::Dspeed, Rob_speed},
	{RobotProperty::Cspeed, Rob_speed},
	{RobotProperty::Rspeed, Rob_angleSpeed}
      };

    }


    // Logger
    string logDir = "";
    // string logFitness = "";
    string logName = "";
    int genSeed = 42;
    int retrain = 0;

    bool visualize = true;
    bool printInfo = true;
    int scenario = 0;
    int clearZeros = 0;
    bool penalizeZeroActions = true;
    int fitSselect = 0;
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
    float diversityMin = 0;
    float diversityMax = 0;

    // Population
    int popSelected = 0;
    int popCrossed = 0;
    int popMutated = 0;
    int popFilled = 0;
    int popMin = 20;
    int popSize = 0;

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
    float fitnessAvgObjCount = 0;
    float fitnessMinObjCount = 0;
    float fitnessMaxObjCount = 0;

    shared_ptr<std::ostringstream> fitnessStr;
    shared_ptr<std::ostringstream> logStr;

    // Robot Config
    // TODO: what to do with those parameter
    string obstacleName = "map";
    float Rob_width = 0.3; // [m]
    float Rob_speed = 0.2; // [m/s]
    float Rob_RPM = 12; // Rotations per minute
    float Rob_angleSpeed = 0; // 2*Pi * RPM * 1/60s [rad/s]
    rob_config rob_conf;

    // Map
    std::mt19937 generator;
    int mapType = 1;
    int mapWidth = 11;
    int mapHeight = 11;
    float mapResolution = 0.01;


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
    bool penalizeRotation = false;
    // Selection
    int selectIndividuals = 10;
    int selectKeepBest = 0;
    int tournamentSize = 2;
    float selPressure = 1.5;
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
    int crossStrategy = 0;
    int crossFailed = 0;

    // Mutation functions, configurations
    float mutaOrtoAngleProba = 0.1;
    float mutaRandAngleProba = 0.1;
    float mutaPosDistProba = 0.1;
    float mutaNegDistProba = 0.01;
    float mutaRandScaleDistProba = 0.01;
    float mutaPosDistMax = 50;
    float mutaReplaceGen = 0.001;
    int mutaCount = 0;

    // Adaptive Parameter
    bool adaptParameter = false;
    float crossUpper = 0.8;
    float crossLower = 0.5;
    float cLenUpper = 0.5;
    float cLenLower = 0.1;
    float mutUpper = 0.1;
    float lastDmax = 0;

    bool  adaptSP = false;
    float adaptSPupper = 2;
    float adaptSPlower = 1;

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
    void adaptSelPressure();
    // void adaptGenReplMutation();
  };

}

#endif /* __TOOLS_CONFIG__ */
