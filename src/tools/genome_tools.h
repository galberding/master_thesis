#ifndef GENOME_TOOLS_H
#define GENOME_TOOLS_H

#include "path_tools.h"
#include "debug.h"

using namespace std;
using namespace grid_map;
using namespace path;

namespace genome_tools {
  struct genome{

    static int gen_id;
    grid_map::Matrix trail;
    genome():id(gen_id){gen_id++;};
    genome(float fitness):fitness(fitness),id(gen_id){gen_id++;};
    genome(PAs actions):actions(actions),id(gen_id){gen_id++;};
    bool operator < (const genome& gen) const
    {
        return (fitness < gen.fitness);
    }
    bool operator==(const genome& gen)const {
      return gen.id == id;
    }

    float getPathLen(){
      float len = 0;
      // int crossings = 0;
      for(auto it = actions.begin(); it != actions.end(); it++){
	len += (*it)->mod_config[PAP::Distance];
	// crossings += (*it)->c_config[Counter::CrossCount];
      }
      // traveledDist = len;
      return len / actions.size();
    }

    /**
       Calculate the traveled dist and cross count.
       The calues are stored in the traveledDist and coverage paremeter.
       Returns false if distance is 0 -> gen fitness does not need to be calculated
     */
    bool updateGenParameter();
    void setPathSignature(shared_ptr<GridMap> gmap);




    int id = 0;
    PAs actions;
    WPs waypoints;
    float fitness = -1;
    // Standard Population Diversity
    float spd = 0;
    float traveledDist = 0;
    float cross = 0;
    float p_obj = 0;
    float rotationCost = 0;
    float coverage = 0;

    float pixelCrossCoverage = 0;
    float pathLengh = 0;	// [m]
    float rotations = 0;	// [rad]
    bool reachEnd = false;

    float finalCoverage = 0;
    float finalTime = 0;
    float finalRotationTime = 0;
    bool crossed = false;
    bool mutated = false;
    bool selected = false;
    shared_ptr<Matrix> mat;
  };

  using Genpool = std::deque<genome>;
  using GenPair = pair<genome, genome>;
  using SelectionPool = list<GenPair>;
  using FamilyPool = deque<Genpool>;

  void validateGen(genome &gen);
  int countZeroActions(genome &gen);
  float calZeroActionPercent(genome &gen);
  float calZeroActionPercent(Genpool &pool);
  /**
     Count how many gens have action length less than minSize.
     Those gens cannot participate on crossover.
   **/
  int countDeadGens(Genpool &pool, int minSize);
  void removeZeroPAs(Genpool &pool);
  void removeZeroPAs(genome &gen);
  void calDistanceMat(Genpool &pool, Eigen::MatrixXf& D, Eigen::VectorXf& upperFlat);
  void getDivMeanStd(Genpool &pool, float& mean, float& stdev, float &min, float &max);
  // genome calMeanGen(Genpool& pool, int maxActionSize);
}




#endif /* GENOME_TOOLS_H */
