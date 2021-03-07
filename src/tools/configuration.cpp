#include "configuration.h"
#include <yaml-cpp/yaml.h>


bool conf::executionConfig::loadConfFromYaml(const string path){
  YAML::Node yConf = YAML::LoadFile(path);
  // if(yConf["weights"]){
  //   cout << yConf["weights"] << endl;
  // }
  maxIterations = yConf["maxIterations"].as<float>();
  // Time, occ, configverage

  if(yConf["visualize"])
    visualize = yConf["visualize"].as<bool>();
  if(yConf["printInfo"])
    printInfo = yConf["printInfo"].as<bool>();

  if(yConf["scenario"])
    scenario = yConf["scenario"].as<float>();
  if(yConf["logName"])
    logName = yConf["logName"].as<string>();
  if(yConf["logDir"])
    logDir = yConf["logDir"].as<string>();
  if(yConf["clearZeros"])
    clearZeros = yConf["clearZeros"].as<float>();
  if(yConf["penalizeZeroActions"])
    penalizeZeroActions = yConf["penalizeZeroActions"].as<bool>();
  if(yConf["fitSselect"])
    fitSselect = yConf["fitSselect"].as<float>();
  if(yConf["funSelect"])
    funSelect = yConf["funSelect"].as<float>();
  if(yConf["weights"]["time"])
    fitnessWeights[0] = yConf["weights"]["time"].as<float>();
  if(yConf["weights"]["occ"])
    fitnessWeights[1] = yConf["weights"]["occ"].as<float>();
  if(yConf["weights"]["coverage"])
    fitnessWeights[2] = yConf["weights"]["coverage"].as<float>();
  if(yConf["initActions"])
    initActions = yConf["initActions"].as<float>();
  if(yConf["initIndividuals"])
    initIndividuals = yConf["initIndividuals"].as<float>();
  if(yConf["keep"])
    selectKeepBest = yConf["keep"].as<float>();
  if(yConf["select"])
    selectIndividuals = yConf["select"].as<float>();
  if(yConf["selPressure"])
    selPressure = yConf["selPressure"].as<float>();
  if(yConf["tournamentSize"])
    tournamentSize = yConf["tournamentSize"].as<float>();
  if(yConf["crossoverProba"])
    crossoverProba = yConf["crossoverProba"].as<float>();
  if(yConf["crossLength"])
    crossLength = yConf["crossLength"].as<float>();
  if(yConf["crossStrategy"])
    crossStrategy = yConf["crossStrategy"].as<float>();
   if(yConf["crossChildSelector"])
    crossChildSelector = yConf["crossChildSelector"].as<float>();
  if(yConf["mutaRandAngleProba"])
    mutaRandAngleProba = yConf["mutaRandAngleProba"].as<float>();
  if(yConf["mutaOrtoAngleProba"])
    mutaOrtoAngleProba = yConf["mutaOrtoAngleProba"].as<float>();
  if(yConf["mutaPosDistProba"])
    mutaPosDistProba = yConf["mutaPosDistProba"].as<float>();
  if(yConf["mutaNegDistProba"])
    mutaNegDistProba = yConf["mutaNegDistProba"].as<float>();
  if(yConf["mutaRandScaleDistProba"])
    mutaRandScaleDistProba = yConf["mutaRandScaleDistProba"].as<float>();
  if(yConf["mutaPosDistMax"])
    mutaPosDistMax = yConf["mutaPosDistMax"].as<float>();
  if(yConf["mutaReplaceGen"])
    mutaReplaceGen = yConf["mutaReplaceGen"].as<float>();
  // Adaptive parameter:
  if(yConf["adaptParameter"])
    adaptParameter = yConf["adaptParameter"].as<bool>();
  if(yConf["crossUpper"])
    crossUpper = yConf["crossUpper"].as<float>();
  if(yConf["crossLower"])
    crossLower = yConf["crossLower"].as<float>();
  if(yConf["mutUpper"])
    mutUpper = yConf["mutUpper"].as<float>();
  if(yConf["cLenUpper"])
    cLenUpper = yConf["cLenUpper"].as<float>();
  if(yConf["cLenLower"])
    cLenLower = yConf["cLenLower"].as<float>();

  // Adapt Selection Pressure
  if(yConf["adaptSP"])
    adaptSP = yConf["adaptSP"].as<bool>();
  if(yConf["adaptSPupper"])
    adaptSPupper = yConf["adaptSPupper"].as<float>();
  if(yConf["adaptSPlower"])
    adaptSPlower = yConf["adaptSPlower"].as<float>();

  // Population
  if(yConf["popMin"])
    popMin = yConf["popMin"].as<float>();

  // Robot
  if(yConf["Rob_width"])
    Rob_width = yConf["Rob_width"].as<float>();
  if(yConf["Rob_speed"])
    Rob_speed = yConf["Rob_speed"].as<float>();
  if(yConf["Rob_RPM"])
    Rob_RPM = yConf["Rob_RPM"].as<float>();



  // Snapshots
  if(yConf["restore"])
    restore = yConf["restore"].as<bool>();
  if(yConf["snapshot"])
    snapshot = yConf["snapshot"].as<string>();
  if(yConf["takeSnapshot"])
    takeSnapshot = yConf["takeSnapshot"].as<bool>();
  if(yConf["takeSnapshotEvery"])
    takeSnapshotEvery = yConf["takeSnapshotEvery"].as<int>();
  // if(yConf["tSnap"])
  //   tSnap = yConf["tSnap"].as<string>();
  // Map
  if(yConf["mapType"])
    mapType = yConf["mapType"].as<float>();
  if(yConf["mapWidth"])
    mapWidth = yConf["mapWidth"].as<float>();
  if(yConf["mapHeight"])
    mapHeight = yConf["mapHeight"].as<float>();
  if(yConf["mapResolution"])
    mapResolution = yConf["mapResolution"].as<float>();

  assert(popMin <= initIndividuals);

  return true;
}


shared_ptr<GridMap> conf::executionConfig::generateMapType(int width, int height, float res, int type, Position &start){
  GridMap map;
  map.setGeometry(Length(height, width), res);
  bool startSet = false;
  switch(type){
  case 0:{ // empty
    map.add("obstacle", 0);
    assertm(map.getPosition(Index(0,0), start), "Cannot get position by index while map generation type 0");
    break;
  }
  case 1:{ // bounds
    map.add("obstacle", 1);
    int h_offset = static_cast<int>(map.getSize()(0)*0.1);
    int w_offset = static_cast<int>(map.getSize()(1)*0.1);


    Index submapStartIndex(h_offset, w_offset);
    Index submapBufferSize(
			   static_cast<int>(map.getSize()(0) -2*h_offset),
			   static_cast<int>(map.getSize()(1) - 2*w_offset)
			   );


    for (grid_map::SubmapIterator iterator(map,submapStartIndex, submapBufferSize);
      !iterator.isPastEnd(); ++iterator) {
      if(!startSet){
	startSet = true;
	assertm(map.getPosition(*iterator, start), "Cannot get position by index while map generation type 1");
      }
      map.at("obstacle", *iterator) = 0;

    }
    break;
  }
  // case 2:{ // Center square
  //   break;
  // }
  default:{
    warn("Unkown map type -- use empty map");
    map.add("obstacle", 0);
  }
  }

  return make_shared<GridMap>(map);
}

void conf::executionConfig::adaptCrossover(){
  //set crossover Proba
  if(not adaptParameter or currentIter == 0) return;

  float dMax = diversityMean + diversityStd;
  if (dMax > lastDmax)
    lastDmax = dMax;
  else
    dMax = lastDmax;

  crossoverProba = diversityMean / dMax * (crossUpper -crossLower) + crossLower;
}


void conf::executionConfig::adaptMutation(){
  if(not adaptParameter or currentIter == 0) return;

  float dMax = diversityMean + diversityStd;
  // if (dMax > lastDmax)
  //   lastDmax = dMax;
  // else
  //   dMax = lastDmax;

  float muta, mutaFit, mutaDiv;
  if (diversityMean > 0.001){
    if (dMax > overallDMax)
      overallDMax = dMax;
    else
      dMax = overallDMax;
    if(fitnessMax - fitnessMin < std::numeric_limits<float>::min())
      mutaFit = mutUpper;
    else
      mutaFit =  (fitnessMax - fitnessAvg) / (fitnessMax - fitnessMin) * mutUpper;
    mutaDiv = (dMax - diversityMean)  / dMax * mutUpper;
    muta = (mutaFit + mutaDiv) / 2;
  }else{
    muta = mutUpper;
  }

  assert(muta >= 0);
  mutaRandAngleProba = muta;
  mutaRandScaleDistProba = muta;
  mutaReplaceGen = muta;
}


void conf::executionConfig::adaptCLen(){
  // increase cLen over Time/diversity reduces
  if(not adaptParameter or currentIter == 0) return;

  float dMax = diversityMean + diversityStd;
  if (dMax > overallDMax)
    overallDMax = dMax;
  else
    dMax = overallDMax;

  crossLength =  diversityMean / dMax * (cLenUpper - cLenLower) + cLenLower;

}

// void conf::executionConfig::adaptGenReplMutation(){}

void conf::executionConfig::adaptSelPressure(){

  if(not adaptSP) return;
  assert(adaptSPupper >= adaptSPlower);
  float pIter = static_cast<float>(currentIter) / maxIterations;
  // debug("piter")
  float rest = adaptSPupper - adaptSPlower;
  float res = adaptSPlower + pIter*rest;
  // debug("SP: ", res);
  if(scenario == 1)		// Tournament Selection
    tournamentSize = ceil(res);
  else
    selPressure = res;

}
