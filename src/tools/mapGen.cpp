#include "mapGen.h"
#include "grid_map_core/iterators/SubmapIterator.hpp"

// using namespace std;
// using namespace grid_map;
using namespace mapgen;

void add_walls(GridMap& map, int width, int height, float thickness){
  map.add("obstacle", 1);
  bool check;
  // debug("W: ", floor(width - 2*thickness), " H: ", floor(height - 2*thickness));
  SubmapGeometry sMap(map,
		      Position(0,0),
		      Length(floor(height - thickness), floor(width - thickness)),
		      check);
  if(not check){
    warn("Cannot build walls");
    return;
  }
  for (grid_map::SubmapIterator iterator(sMap);
       !iterator.isPastEnd(); ++iterator) {
    map.at("obstacle", *iterator) = 0;
  }
}

bool getStartPosition(GridMap& map, Position& start, float robRad){
  for(GridMapIterator it(map); !it.isPastEnd(); ++it){
    if(map.at("obstacle", *it) == 0){
      if(map.getPosition(*it, start)){
	bool isfree = true;
	for (CircleIterator cit(map, start, 2*robRad); !cit.isPastEnd(); ++cit){
	  if(map.at("obstacle", *cit) > 0){
	    isfree = false;
	    break;
	  }
	}
	if(isfree)
	  return true;
      }else{
	warn("Start point cannot be set!");
	throw 42;
      }
    }
  }
  warn("No startpoint found!");
  exit(-1);
  return false;
}

shared_ptr<GridMap> mapgen::generateMapType(int width, int height, float res, float rob_width,  int type, Position& start) {
  GridMap map;
  map.setGeometry(Length(height, width), res);
  map.add("covered", 0);
  bool startSet = false;
  float wallsize = 3;
  switch(type){
  case 0:{ // empty
    // TODO: Why this case??
    map.add("obstacle", 0);
    getStartPosition(map, start, rob_width);
    // throw 42;
    // assertm(map.getPosition(Index(0,0), start), "Cannot get position by index while map generation type 0");
    break;
  }
  case 1:{ // bounds

    // Walls should be at least double the size of the robot:
    // float wallsize = 2* rob_width;

    add_walls(map, width, height, wallsize);
    getStartPosition(map, start, rob_width);

    break;
  }
  case 2:{ // Center circle

    add_walls(map, width, height, wallsize);
    for(grid_map::CircleIterator iter(map, Position(0,0), (width + height) / 8); !iter.isPastEnd(); ++iter){
      map.at("obstacle", *iter) = 1;
    }

    getStartPosition(map, start, rob_width);
    break;
  }
  default:{
    warn("Unkown map type -- use empty map");
    map.add("obstacle", 0);
  }
  }

  return make_shared<GridMap>(map);
}



shared_ptr<GridMap> mapgen::changeMapRes(shared_ptr<GridMap> gmap, float res) {
  GridMap nmap;
  float oldRes = gmap->getResolution();
  // debug("Old res: ", oldRes);
  GridMapCvProcessing::changeResolution(*gmap, nmap, res);
  // debug("New Res: ", nmap.getResolution());
  return make_shared<GridMap>(nmap);
}

void mapgen::drawPathOnMap(shared_ptr<GridMap> gmap, vector<Position>& path, bool inc) {

  // if(not gmap->exists("map"))
  gmap->add("map", 0);

  for (auto nWP = next(path.begin(),1); nWP != path.end(); ++nWP) {
    auto pWP = prev(nWP, 1);
    for (LineIterator it(*gmap, *pWP, *nWP);  !it.isPastEnd(); ++it) {
      gmap->at("map", *it) = inc ? gmap->at("map", *it) + 1 : 0;
      // debug("Draw: ", *it);
    }
  }
}



cv::Mat mapgen::gmapToImg(const shared_ptr<GridMap> gmap, const string layer, uint8_t upperThresh){
  cv::Mat rob_map;
  GridMapCvConverter::toImage<unsigned char, 1>(*gmap, layer, CV_8U, 0.0, upperThresh, rob_map);
  return rob_map;
}

void mapgen::saveMap(const string name, const shared_ptr<GridMap> gmap, const string layer, uint8_t upperThresh){
  cv::Mat img = gmapToImg(gmap, layer, upperThresh);
  cv::imwrite(name+".png", img);
}

bool mapgen::emulateCoveredMapSegment(shared_ptr<GridMap> map, Position& start) {
  int height = map->getLength().x();
  int width = map->getLength().y();
  bool check;
  SubmapGeometry sMap(*map,
		      Position(-height/2, -width/2),
		      Length(floor(height), floor(width)), check);

  // for(SubmapIterator())
  for (grid_map::SubmapIterator iterator(sMap);
       !iterator.isPastEnd(); ++iterator) {
    map->at("covered", *iterator) = 1;
  }

  return map->atPosition("covered", start) == 0;
}
