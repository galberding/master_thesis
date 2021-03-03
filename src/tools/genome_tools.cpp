#include "genome_tools.h"

int genome_tools::genome::gen_id = 0;

// float calVectorAngle(Position a, Position b){
//   float dot = a.dot(b);
//   float norm = a.norm()*b.norm() + FLT_EPSILON;
//   return acos(dot/norm);

// }

int angleDiff(int a, int b){
  int diff = (a % 360) - (b % 360);

  // debug("A: ", a, " B: ", b, " d:", diff);
  diff += (diff > 180) ? -360 : (diff < -180) ? 360 : 0;

  // return abs((diff + 180) % 360 - 180);
  return abs(diff);
}

bool genome_tools::genome::updateGenParameter(){
  this->traveledDist = 0;
  this->cross = 0;
  this->rotationCost = 0;
  float continu = 0;
  // debug("Update");
  for (auto it = actions.begin(); it != actions.end(); ++it) {
    // Distance:
    continu += (*it)->mod_config[PAP::Distance];
    // Given in cm
    traveledDist += (*it)->c_config[Counter::StepCount];
    // Cross is given in cm^2
    cross += (*it)->c_config[Counter::CrossCount];



    auto it_next = next(it, 1);
    if((it != actions.end()) and ((*it)->type != PAT::End) and (it_next != actions.end()) and ((*it)->type != PAT::Start)){
      // Calculate rotation penalty
      // debug("Angle: ", (*it_next)->mod_config[PAP::Angle]);

      assert((*it)->wps.size() >= 2);
      // debug((*it_next)->wps.size());
      // assert((*it_next)->wps.size() >= 2);
      float diff = angleDiff((*it)->mod_config[PAP::Angle], (*it_next)->mod_config[PAP::Angle]);
      // debug("Diff: ", diff);
      rotationCost +=  diff / 180.0;
      // debug(rotationCost);
    }
  }
  // debug("Dist: ", traveledDist, " Cross: ", cross, " Continu: ", continu, " Error: ", continu - traveledDist*0.3);
  // Finalize rotation costs
  rotationCost /= actions.size();
  // debug(rotationCost);
  auto end = actions.back();
  // Panilty if endpoint cannot be reached
  // increase the cross count because the same distance needs to be taken twice
  // (robot is driving backwards until it reaches the startpoint)
  if (end->wps.back() != end->endPoint)
    cross += cross;
  return traveledDist > 0;
}

void genome_tools::genome::setPathSignature(shared_ptr<GridMap> gmap){
  mat = make_shared<Matrix>(Matrix(gmap->get("map")));
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
   // assertm(!((*it)->modified && ((*it)->type == PAT::Start)), "Not allowed!, Start points cannot be modified!");
   assertm(!((*it)->modified && ((*it)->type == PAT::End)), "Not allowed!, End points cannot be modified!");


    (*it)->applyModifications();
    // Change the configuration of the consecutive action
    // Propagate changes until properly applied
    // We do not want to override any changes here!
    auto it_current = it;
    auto it_next = next(it, 1);
    // debug("Type: ", int((*it_next)->type));
    while(!(*it_next)->mendConfig(*it_current)){
      // assertm(!((*it_current)->modified && ((*it)->type == PAT::Start)), "Not allowed!, Start points cannot be modified!");
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
  return res / (gen.actions.size()-1);
}

float genome_tools::calZeroActionPercent(Genpool &pool){
  // TODO: Does nothing!!
  float res = 0;
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    res += calZeroActionPercent(*it);
  }
  return res / pool.size();
}

void genome_tools::removeZeroPAs(Genpool &pool) {
  for(auto &gen : pool){
    removeZeroPAs(gen);
  }
}

int genome_tools::countDeadGens(Genpool &pool, int minSize){
  int res = 0;
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    if (it->actions.size() < minSize or not it->updateGenParameter())
      res++;
    // res += it->actions.size() > minSize ? 0 : 1;
  }
  return res;
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

void genome_tools::calDistanceMat(Genpool &pool, Eigen::MatrixXf& D, Eigen::VectorXf& upperFlat){
  int pSize = pool.size();
  assert(D.cols() == pSize);
  assert(D.rows() == pSize);
  assert(pSize*(pSize +1)/2 == upperFlat.size());

  int idx = 0;
  for(int row=0; row < pSize; row++)
    for(int col=row; col<pSize; col++){
      upperFlat[idx] = D(row, col) = D(col, row) = (*pool[row].mat - *pool[col].mat).cwiseAbs().sum() / ((*pool[row].mat).rows() * (*pool[row].mat).cols());
      idx++;
    }
}


void genome_tools::getDivMeanStd(Genpool &pool, float& mean, float& stdev){
  int pSize = pool.size();
  Eigen::MatrixXf D(pSize, pSize);
  Eigen::VectorXf upperFlat(pSize*(pSize +1)/2);
  calDistanceMat(pool, D, upperFlat);
  mean = upperFlat.mean();

  stdev = sqrt((upperFlat.array() - mean).square().sum()/(upperFlat.size()-1));

  // std = D.triangularView<Eigen::Upper>().std();
}

// TODO: Put this in path tools
// shared_ptr<PathAction> makeAction(PAT type, Position start, Position end){
//   switch(type){
//   case PAT::Start:{
//     return make_shared<StartAction>(StartAction(start));
//     break;
//   }
//   case PAT::Ahead: case PAT::CAhead:{
//     auto ac = AheadAction(static_cast<PAT>(type), {});
//     ac.setConfigByWaypoints(start, end);
//     return make_shared<AheadAction>(ac);
//     break;
//   }
//   case PAT::End:{
//     return make_shared<EndAction>(EndAction({start, end}));
//   }
//   }

// }


// void genome_tools::calMeanGen(Genpool& pool, vector<int> &contrib, vector<vector<float>> &meanActions, int maxActionSize){
//   //$G_{n}^{ave} = \frac{1}{P}\displaystyle\sum_{i=1}^{P} G_{i,n}$
//   // PAs meanActions;
//   // float actions[maxActionSize][5]; // [type, x_s, y_s, x_e, y_e]
//   // vector<vector<float>> meanActions(5, vector<float>(maxActionSize));




//   // iterate over all gens and maxsize of gens and add start and endpoint as
//   for (auto it = pool.begin(); it != pool.end(); ++it) {
//     for(int i = 0; i < maxActionSize; i++){
//       // Check if end of gen is reached
//       if (maxActionSize >= (*it).actions.size())
// 	break;
//       // Add contribution
//       contrib[i]++;
//       // Add action parameter
//       // meanActions[i][0] += static_cast<int>(it->actions[i]->type);
//       meanActions[i][0] += it->actions[i]->wps.front()[0];
//       meanActions[i][1] += it->actions[i]->wps.front()[1];
//       meanActions[i][2] += it->actions[i]->wps.back()[0];
//       meanActions[i][3] += it->actions[i]->wps.back()[1];
//     }
//   }

//   // Generate Actions
//   for(int i = 0; i < maxActionSize; i++){
//     // create mean actions
//     for(int j=0; j<4; j++)
//       meanActions[i][j] = meanActions[i][j] / contrib[i];
//   }
// }

// //
// vector<float> genome_tools::SPD(Genpool& pool, int maxActionSize){
//   // Action wise difference and
//   vector<int> contrib(maxActionSize);
//   vector<float> actionStd(maxActionSize);
//   vector<vector<float>> meanActions(5, vector<float>(maxActionSize));
//   calMeanGen(pool, contrib, meanActions maxActionSize);
//   for (auto it = pool.begin(); it != pool.end(); ++it) {
//     SPD(*it, meanGen, actionStd);
//   }
// }

// // // calculate the genwise SPD and patially calculate the action wise standard deviation
// void genome_tools::SPD(genome &gen, vector<vector<float>> &meanGen, vector<float> &actionStd) {
//   float spd = 0;
//   // $G_{n}^{ave} = \frac{1}{P}\displaystyle\sum_{i=1}^{P} G_{i,n}$
//   for(int i=0; i<gen.actions.size(); i++){
//     float contri = (gen.actions[i]->wps.front() - meanGen.actions[i]->wps.front()).norm()
//       + (gen.actions[i]->wps.back() - meanGen.actions[i]->wps.back()).norm();
//     actionStd[i] += contri;
//     spd += contri;
//   }
//   gen.spd = spd;
// }
