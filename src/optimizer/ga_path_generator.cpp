#include "ga_path_generator.h"

// #ifdef __DEBUG__
// #undef __DEBUG__
// #define __DEBUG__ true
// #endif
///////////////////////////////////////////////////////////////////////////////
//                              HelperFunctions                               //
///////////////////////////////////////////////////////////////////////////////

using namespace ga;

bool ga::compareFitness(const struct genome &genA, const struct genome &genB){
  return genA.fitness > genB.fitness;
}


ga::genome ga::roulettWheelSelection(ga::Genpool &currentPopulation, std::uniform_real_distribution<> selDistr, std::mt19937 generator){
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

int ga::randRange(int lower, int upper){
  return (rand() % (upper -lower) + lower);
}




///////////////////////////////////////////////////////////////////////////////
//                                     GA                                    //
///////////////////////////////////////////////////////////////////////////////

void ga::GA::populatePool(Genpool &currentPopuation, Position start, WPs endpoints, int individuals, int initialActions){

  for(int j=0; j<individuals; j++){
    PAs actions;
    actions.push_back(make_shared<StartAction>(StartAction(start)));
    for(int i=0; i<initialActions; i++){
      PA_config config{{PAP::Angle, angleDistr(generator)}, {PAP::Distance, distanceDistr(generator)*100}};
      actions.push_back(make_shared<AheadAction>(AheadAction(PAT::CAhead, config)));
    }
    actions.push_back(make_shared<EndAction>(EndAction(endpoints)));
    currentPopuation.push_back(genome(actions));
  }

}


void ga::GA::selection(ga::Genpool& currentPopuation, ga::Genpool& selectionPool, int individuals) {
  // Sort descending according to fitness
  sort(currentPopuation.begin(), currentPopuation.end(), compareFitness);

  // Perform turnament selection:
  for(int i=0; i<individuals; i++){
    selectionPool.push_back(roulettWheelSelection(currentPopuation, selectionDist, generator));
  }
}


void ga::GA::crossover(genome &par1, genome &par2, Genpool& newPopulation){
  PAs parent1, parent2, child1, child2;
  if(par1.actions.size() > par2.actions.size()){
    parent1 = par2.actions;
    parent2 = par1.actions;
  }else{
    parent1 = par1.actions;
    parent2 = par2.actions;
  }

  int idx = randRange(1, parent1.size() - 2);
  // child1.insert(child1.begin(), parent1.begin(), parent1.begin() + idx);
  child1.insert(child1.begin(), parent1.begin(), std::next(parent1.begin(), idx));
  child2.insert(child2.begin(), parent2.begin(), std::next(parent2.begin(), idx));

  child1.insert(child1.end(), std::next(parent2.begin(), idx), parent2.end());
  child2.insert(child2.end(), std::next(parent1.begin(), idx), parent1.end());

  newPopulation.push_back(genome(child1));
  newPopulation.push_back(genome(child2));

}

void ga::GA::crossover(ga::Genpool& currentSelection, ga::Genpool& newPopulation) {
  // Single point Crossover
  for(int i=0; i<currentSelection.size(); i++){
    genome par1 = currentSelection.back();
    currentSelection.pop_back();
    for (auto par2 : currentSelection){
      crossover(par1, par2, newPopulation);
    }
  }
}


void mutation(Genpool& currentPopulation, Mutation_conf muat_conf){

}


void ga::GA::mutation(Genpool& currentPopulation) {
  // double


}


void ga::GA::evalFitness(Genpool &currentPopulation, path::Robot &rob){
  // First evaluate action on map
  // Sum multiple covered regions
  // Sum traveled Distance

  for(Genpool::iterator it = currentPopulation.begin(); it != currentPopulation.end();){
    if(rob.evaluateActions(it->actions)){

      double Cdistance = 0; // Cleand distance
      double distance = 0; // uncleand distance
      double occ = 0;
      int crossed = 0;
      for(auto action : it->actions){
	PA_config conf = action->getConfig();
	if(conf.count(PAP::Distance) > 0){
	  if(action->type == PAT::CAhead){
	    // Add cross count (default is 1)
	    crossed += conf[PAP::CrossCount] - 1;
	    Cdistance += conf[PAP::Distance];
	  }else{
	    distance += conf[PAP::Distance];
	  }
	}
      }
      it->fitness = calFitness(Cdistance, distance, crossed, rob.getConfig()[RP::Clean_speed_cm_s] / 100, rob.getConfig()[RP::Drive_speed_cm_s] / 100, rob.getFreeArea());
      it++;
    }else{
      it = currentPopulation.erase(it);
    }
  }
}


double ga::GA::calFitness(double cdist,
			  double dist,
			  int crossed,
			  double cSpeed_m_s,
			  double speed_m_s,
			  int freeSpace){
  // Use the different speeds for time calculation

  double actual_time = cdist / cSpeed_m_s + dist / speed_m_s;
  double optimal_time = (cdist - crossed) / cSpeed_m_s;
  double final_time = optimal_time / actual_time;


  // Calculate the final occ based on the
  double current_occ = cdist - crossed;
  double optimal_occ = cdist + crossed;

  double final_occ = current_occ / optimal_occ ;

  // debug("Actual time: ", actual_time);
  // debug("Optimal time: ", optimal_time);
  // debug("Final time: ", final_time);
  // debug("final_occ: ", final_occ);
  double weight = 0.6;
  return ((1-weight)*(final_time + final_occ) + weight*(current_occ / freeSpace)) / 3;

}

///////////////////////////////////////////////////////////////////////////////
//                             Mutation Functions                            //
///////////////////////////////////////////////////////////////////////////////

void ga::GA::addAction(genome &gen){
  int idx = randRange(1, gen.actions.size()-1);

  PA_config conf = (*next(gen.actions.begin(), idx))->getConfig();
  PAT type = (*next(gen.actions.begin(), idx))->get_type();
  gen.actions.insert(next(gen.actions.begin(), idx),make_shared<AheadAction>(AheadAction(type, conf)));
}

void ga::GA::removeAction(genome &gen){
  int idx = randRange(1, gen.actions.size()-1);
  gen.actions.erase(next(gen.actions.begin(), idx));
}

void ga::GA::addAngleOffset(genome &gen){
  int idx = randRange(1, gen.actions.size()-1);
  double offset = angleDistr(generator);
  next(gen.actions.begin(), idx)->get()->mod_config[PAP::Angle] += offset;
  next(gen.actions.begin(), idx)->get()->mod_config[PAP::AngleOffset] += offset;
}

void ga::GA::addDistanceOffset(genome &gen){
  int idx = randRange(1, gen.actions.size()-1);
  double offset = distanceDistr(generator);
  next(gen.actions.begin(), idx)->get()->mod_config[PAP::Distance] += offset;
  next(gen.actions.begin(), idx)->get()->mod_config[PAP::DistanceOffset] += offset;
}

void ga::GA::swapRandomAction(genome &gen){
  int idx1 = randRange(1, gen.actions.size()-1);
  int idx2 = randRange(1, gen.actions.size()-1);

  auto item1 = *next(gen.actions.begin(), idx1);

  *next(gen.actions.begin(), idx1) = *next(gen.actions.begin(), idx2);
  *next(gen.actions.begin(), idx2) = item1;

}
