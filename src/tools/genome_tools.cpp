#include "genome_tools.h"

int genome_tools::genome::gen_id = 0;


bool genome_tools::genome::updateGenParameter(){
  this->traveledDist = 0;
  this->cross = 0;
  // debug("Update");
  for (auto it = actions.begin(); it != actions.end(); ++it) {
    traveledDist += (*it)->c_config[Counter::StepCount];
    cross += (*it)->c_config[Counter::CrossCount];
  }

  return !(traveledDist == 0);
}

void genome_tools::validateGen(genome &gen){
 for(auto it = gen.actions.begin(); it != gen.actions.end(); it++){
   // What is needed to validate the gens?
   // Gens are not validated after initialization!
   // Gens need to be falidated after crossover to fix the crossing
   // Consequently each gen that is flagged as modified needs to be adapted or its changes need to be
   // propagated to the next Gen
   if(!(*it)->modified) continue;

   // Assumptions:
   // What about newly created actions? -> generate waypoints by adding!
   assertm((*it)->wps.size() >= 1, "Not enough gens to validate!");
   // Ensures that we can always access the previous action without segfault
   assertm(!((*it)->modified && ((*it)->type == PAT::Start)), "Not allowed!, Start points cannot be modified!");
   assertm(!((*it)->modified && ((*it)->type == PAT::End)), "Not allowed!, End points cannot be modified!");


    (*it)->applyModifications();
    // Change the configuration of the consecutive action
    // Propagate changes until properly applied
    // We do not want to override any changes here!
    auto it_current = it;
    auto it_next = next(it, 1);
    // debug("Type: ", int((*it_next)->type));
    while(!(*it_next)->mendConfig(*it_current)){
      assertm(!((*it_current)->modified && ((*it)->type == PAT::Start)), "Not allowed!, Start points cannot be modified!");
      assertm(!((*it_next)->modified && ((*it)->type == PAT::End)), "Not allowed!, End points cannot be modified!");
      // warn("Validation: Remove Action ", (*it_next)->pa_id, " because no distance!");
      // Cannot mend means that the current changes are applied and we need to mend the consecutive action
      it_current = it_next;
      assertm((*it_next)->type != PAT::End, "Try to access type after end!");
      it_next++;
      // TODO: goto end or just return because all changes had been applied
      if(it_next == gen.actions.end()) return;


    }
  }
}

float genome_tools::calZeroActionPercent(genome &gen){
  float res = 0;
  for (auto it = gen.actions.begin(); it != gen.actions.end(); ++it) {
    if(compareF(it->get()->mod_config[PAP::Distance], 0, 0.000001)){
      res += 1;
    }
  }
  return res / gen.actions.size();
}

float genome_tools::calZeroActionPercent(Genpool &pool){
  float res = 0;
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    if(compareF(it->fitness, 0.0, 0.00001)){
      res += 1;
    }
  }
  return res / pool.size();
}

void genome_tools::removeZeroPAs(Genpool &pool) {
  for(auto &gen : pool){
    removeZeroPAs(gen);
  }
}

void genome_tools::removeZeroPAs(genome &gen){
  // Remove consecutive zero actions and check if the third action is still a zero action
  // We will allow to consecutive zero actions, everything else will be removed
  // This will have no impact on the fitness thus it can safely be executed after the evaluation process
  PAs clone;
  clone.insert(clone.begin(), gen.actions.begin(), gen.actions.end());
  gen.actions.clear();
  for (auto &ac : clone){
    if(ac->type == PAT::Start || ac->type == PAT::End){
      gen.actions.push_back(ac);
    }else{
      if(compareF(ac->mod_config[PAP::Distance], 0)
	 && compareF(gen.actions.back()->mod_config[PAP::Distance], 0)){
	continue;
      }else{
	gen.actions.push_back(ac);
      }
    }
  }
  // gen.actions =
}
