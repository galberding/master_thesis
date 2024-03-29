#ifndef GRID_SEARCH_H
#define GRID_SEARCH_H

#include "ga_path_generator.h"
#include <chrono>


namespace gsearch {


  struct Searcher{
    // The searcher will get a hardcoded definition in which range it should
    // perform the metaparamter search

    void tSearch();
    void tSearchV2(string dirName);
    // Will create grid map, robot and GA instance
    // search and log the progress
    shared_ptr<GridMap> generateMapType(int with, int height, float res, int type, Position& start);
    void search(string dirName);
    vector<ga::executionConfig> generateConfigs(string dirname);
    // Wrapper to call search in multiple threads
    /* void threadded_search(ga::executionConfig config); */
  };

}

#endif /* GRID_SEARCH_H */
