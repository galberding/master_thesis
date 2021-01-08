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
      for(auto it = actions.begin(); it != actions.end(); it++){
	len += (*it)->mod_config[PAP::Distance];
      }
      return len / actions.size();
    }

    int id = 0;
    PAs actions;
    WPs waypoints;
    float fitness = 0;
  };

  using Genpool = std::deque<genome>;
  using SelectionPool = list<pair<genome, genome>>;

  void validateGen(genome &gen);
  float calZeroActionPercent(genome &gen);
  float calZeroActionPercent(Genpool &pool);

}




#endif /* GENOME_TOOLS_H */
