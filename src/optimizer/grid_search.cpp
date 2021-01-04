#include "grid_search.h"

using namespace ga;

void gsearch::Searcher::tSearch(){
  int pool_size = 1;
  std::future<int> t_pool[pool_size];
  vector<ga::executionConfig> confs = generateConfigs("testrunV1");
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
	std::cout << ".";
      } else if (status == std::future_status::ready) {
	std::cout << endl;
	done++;
	t_pool[i] = std::async([this, it]{
	  GA ga(42, *it);
	  ga.optimizePath();
	  return 0;
	});
	it = next(it, 1);
	cout << done << "/" << (all - pool_size) << " done" << endl;
	// Ensure to break the loop for checking if end is reached
	break;
      }

    }
  }
}

void gsearch::Searcher::tSearchV2(){
  int pool_size = 24;
  std::future<int> t_pool[pool_size];
  vector<ga::executionConfig> confs = generateConfigs("v2_mutation_roulett");
  int done = 0;
  int all = confs.size();

  // init the pool
  auto it = confs.begin();
  for(int i=0; i<pool_size; i++){

    t_pool[i] = std::async([this, it]{
      GA_V2 ga(42, *it);
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
	std::cout << ".";
      } else if (status == std::future_status::ready) {
	std::cout << endl;
	done++;
	t_pool[i] = std::async([this, it]{
	  GA_V2 ga(42, *it);
	  ga.optimizePath();
	  return 0;
	});
	it = next(it, 1);
	cout << done << "/" << (all - pool_size) << " done" << endl;
	// Ensure to break the loop for checking if end is reached
	break;
      }

    }
  }
  debug("Wait for all threads to get ready");
  for (int i = 0; i < pool_size; i++) {
    t_pool[i].wait();
    cout << done << "/" << (all - pool_size) << " done" << endl;
  }
}

void gsearch::Searcher::search() {
  // What should search do
  // Search in range of different population sizes at First

  auto confs = generateConfigs("blub2");
  // se.tSearchV2();
  // auto co = *next(confs.begin(), 4);
  for (auto &conf : confs){
    // co.initActions = 20;
    conf.maxIterations = 100;
    debug(conf.config_to_string());
    GA_V2 ga(42, conf);
    ga.optimizePath(true);
  }


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

vector<ga::executionConfig> gsearch::Searcher::generateConfigs(string dirname) {
  vector<ga::executionConfig> configs;
  // For now we hardcode all configs that we want to generate as well as the tests
  int runId = 0;
  vector<vector<float>> weightConfs = {
    {0.2, 0.3, 0.5},
    {0.3, 0.2, 0.5},
    {0.25, 0.3, 0.45},
    {0.3, 0.25, 0.45},
    {0.1, 0.5, 0.4},
    {0.5, 0.1, 0.4}
  };

  // initial population
  int initialPop = 1000;
  // initial actions
  int stepsInitActions = 100;
  int stepsSelIndividuals = 5;
  int stepsKeepBest = 10;
  // for(int initActions=20; initActions<=2*stepsInitActions; initActions += stepsInitActions){
    // selected individuals
    for(int selIndividuals=5; selIndividuals <= 4*stepsSelIndividuals; selIndividuals += stepsSelIndividuals){
      // keep best
      for(int keepBest = 0; keepBest <= 2*stepsKeepBest; keepBest += stepsKeepBest){
	// Fitness weight
	for(auto &wConf : weightConfs){
	  Position startpos;
	  shared_ptr<GridMap> mapptr = generateMapType(50, 50, 0.3, 1, startpos);
	  ga::executionConfig conf(dirname, to_string(runId)+ "_" + "log", mapptr, startpos, {Position(42,42)});
	  // conf.fitnessName = "fitness_record";
	  conf.initActions = 20;
	  conf.selectIndividuals = selIndividuals;
	  conf.selectKeepBest = keepBest;
	  conf.fitnessWeights = wConf;
	  conf.maxIterations = 100;

	  configs.push_back(conf);
	  runId++;
	}
      }
    }
  // }


  return configs;
}
