#include "grid_search.h"

using namespace ga;

void gsearch::Searcher::tSearch(){
  int pool_size = 4;
  std::future<int> t_pool[pool_size];
  vector<ga::executionConfig> confs = generateConfigs();
  int done = 0;
  int all = confs.size();

  // init the pool
  auto it = confs.begin();
  for(int i=0; i<pool_size; i++){

    t_pool[i] = std::async([this, it]{
      GA ga(42, *it);
      ga.optimizePath();
      return 0;
    });
    done++;
    it = next(it, 1);
  }
  std::future_status status;
  while(it != confs.end()){
    for (int i = 0; i < pool_size; i++) {
      status = t_pool[i].wait_for(std::chrono::seconds(1));
      if (status == std::future_status::deferred) {
	std::cout << "deferred\n";
      } else if (status == std::future_status::timeout) {
	std::cout << "timeout\n";
      } else if (status == std::future_status::ready) {
	done++;
	t_pool[i] = std::async([this, it]{
	  GA ga(42, *it);
	  ga.optimizePath();
	  return 0;
	});
	it = next(it, 1);
      }
      cout << done << "/" << all << " done" << endl;
    }
  }
}

void gsearch::Searcher::search(GA ga) {
  // What should search do
  // Search in range of different population sizes at First


}

shared_ptr<GridMap> gsearch::Searcher::generateMapType(int width, int height, float res, int type, Position& start) {
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

vector<ga::executionConfig> gsearch::Searcher::generateConfigs() {
  vector<ga::executionConfig> configs;
  // For now we hardcode all configs that we want to generate as well as the tests
  int runId = 0;

  // initial population
  int initialPop = 1000;
  // initial actions
  int stepsInitActions = 100;
  int stepsSelIndividuals = 10;
  int stepsKeepBest = 10;
  for(int initActions=stepsInitActions; initActions<=3*stepsInitActions; initActions += stepsInitActions){
    // selected individuals
    for(int selIndividuals=5; selIndividuals <= 10*stepsSelIndividuals; selIndividuals += stepsSelIndividuals){
      // keep best
      for(int keepBest = 0; keepBest <= 10*stepsKeepBest; keepBest += stepsKeepBest){
	// Fitness weight
	for(float fWeight = 0.1; fWeight <= 1.0; fWeight += 0.1){
	  Position startpos;
	  shared_ptr<GridMap> mapptr = generateMapType(50, 50, 0.3, 1, startpos);
	  ga::executionConfig conf("testrun/"+to_string(runId), "configuration", mapptr, startpos, {Position(42,42)});
	  conf.fitnessName = "fitness_record";
	  conf.initActions = initActions;
	  conf.selectIndividuals = selIndividuals;
	  conf.selectKeepBest = keepBest;
	  conf.fitnessWeight = fWeight;

	  configs.push_back(conf);
	  runId++;
	}
      }
    }
  }


  return configs;
}
