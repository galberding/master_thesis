#include "ga_path_generator.h"

///////////////////////////////////////////////////////////////////////////////
//                              HelperFunctions                               //
///////////////////////////////////////////////////////////////////////////////

bool ga::compareFitness(const struct genome &genA, const struct genome &genB){
  return genA.fitness > genB.fitness;
}


genome ga::roulettWheelSelection(Genpool &currentPopulation, std::uniform_real_distribution<> selDistr, std::mt19937 generator){
  // Calculate Probabilities for all individuals
  // std::default_random_engine generator;
  // std::uniform_real_distribution<double> distribution(0.0,1.0);

  double totalFitness = 0;
  for(auto &gen :currentPopulation){
    totalFitness += gen.fitness;
  }

  double rand = selDistr(generator);
  // cout << "Wheel: " << rand << endl;
  double offset = 0.0;
  genome gen;

  for(int i=0; i < currentPopulation.size(); i++){
    offset += currentPopulation.at(i).fitness / totalFitness;

    if(rand < offset){
      gen = currentPopulation.at(i);
      currentPopulation.erase(currentPopulation.begin() + i);
      break;
    }
  }
  return gen;
}


///////////////////////////////////////////////////////////////////////////////
//                                     GA                                    //
///////////////////////////////////////////////////////////////////////////////

void ga::GA::selection(Genpool& currentPopuation, Genpool& selection) {
  // Sort descending according to fitness
  sort(currentPopuation.begin(), currentPopuation.end(), compareFitness);

  // Perform turnament selection:


}

void ga::GA::crossover(Genpool& currentSelection, Genpool& newPopulation) {

}

void ga::GA::mutation(Genpool& currentPopulation) {

}
