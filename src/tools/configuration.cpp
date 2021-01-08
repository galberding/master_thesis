#include "configuration.h"
#include <yaml-cpp/yaml.h>

bool conf::executionConfig::loadConfFromYaml(const string path){
  YAML::Node yConf = YAML::LoadFile(path);
  // if(yConf["weights"]){
  //   cout << yConf["weights"] << endl;
  // }
  maxIterations = yConf["maxIterations"].as<int>();
  // Time, occ, configverage

  if(yConf["logName"])
    logName = yConf["logName"].as<string>();
  if(yConf["logDir"])
    logDir = yConf["logDir"].as<string>();
  if(yConf["weights"]["time"])
    fitnessWeights[0] = yConf["weights"]["time"].as<float>();
  if(yConf["weights"]["occ"])
    fitnessWeights[1] = yConf["weights"]["occ"].as<float>();
  if(yConf["weights"]["coverage"])
    fitnessWeights[2] = yConf["weights"]["coverage"].as<float>();
  if(yConf["initActions"])
    initActions = yConf["initActions"].as<int>();
  if(yConf["initIndividuals"])
    initIndividuals = yConf["initIndividuals"].as<float>();
  if(yConf["keep"])
    selectKeepBest = yConf["keep"].as<int>();
  if(yConf["select"])
    selectIndividuals = yConf["select"].as<int>();
  if(yConf["crossoverProba"])
    crossoverProba = yConf["crossoverProba"].as<float>();
  if(yConf["crossLength"])
    crossLength = yConf["crossLength"].as<float>();
  if(yConf["mutaRandAngleProba"])
    mutaRandAngleProba = yConf["mutaRandAngleProba"].as<float>();
  if(yConf["mutaOrtoAngleProba"])
    mutaOrtoAngleProba = yConf["mutaOrtoAngleProba"].as<float>();
  if(yConf["mutaPosDistProba"])
    mutaPosDistProba = yConf["mutaPosDistProba"].as<float>();
  if(yConf["mutaNegDistProba"])
    mutaNegDistProba = yConf["mutaNegDistProba"].as<float>();
  if(yConf["mutaPosDistMax"])
    mutaPosDistMax = yConf["mutaPosDistMax"].as<float>();

  // Snapshots
  if(yConf["restore"])
    restore = yConf["restore"].as<bool>();
  if(yConf["snapshot"])
    snapshot = yConf["snapshot"].as<string>();
  if(yConf["takeSnapshot"])
    takeSnapshot = yConf["takeSnapshot"].as<bool>();
  if(yConf["tSnap"])
    tSnap = yConf["tSnap"].as<string>();
  // Map
  if(yConf["mapType"])
    mapType = yConf["mapType"].as<int>();
  if(yConf["mapWidth"])
    mapWidth = yConf["mapWidth"].as<int>();
  if(yConf["mapHeight"])
    mapHeight = yConf["mapHeight"].as<int>();
  if(yConf["mapResolution"])
    mapResolution = yConf["mapResolution"].as<float>();

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
