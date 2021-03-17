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
  // TODO: Introduce new variables
  // TODO: calculate coverage values
  // TODO: Calculate cross cov
  // Pixel coverage [Rw^2*m^2]

  this->traveledDist = 0;
  this->cross = 0;
  this->rotationCost = 0;
  this->diversityFactor = 0;

  // New collection values
  pixelCrossCoverage = 0;
  pathLengh = 0;
  rotations = 0;
  reachEnd = true;
  p_obj = 0;
  float continu = 0;
  // debug("Update");
  for (auto it = actions.begin(); it != actions.end(); ++it) {
    // Distance:
    (*it)->mod_config[PAP::Distance] = ((*it)->wps.front() - (*it)->wps.back()).norm();
    pathLengh += (*it)->mod_config[PAP::Distance];
    // Coverage in Pixel values
    pixelCrossCoverage += (*it)->c_config[Counter::CrossCount];
    p_obj += (*it)->c_config[Counter::ObjCount];

    // Given in cm
    traveledDist += (*it)->c_config[Counter::StepCount];
    // Cross is given in cm^2
    cross += (*it)->c_config[Counter::CrossCount]; //+ (*it)->c_config[Counter::ObjCount];

    auto it_next = next(it, 1);
    if((it != actions.end()) and ((*it)->type != PAT::End) and (it_next != actions.end()) and ((*it)->type != PAT::Start)){
      // Calculate rotation penalty
      // debug("Angle: ", (*it_next)->mod_config[PAP::Angle]);

      assert((*it)->wps.size() >= 2);
      // debug((*it_next)->wps.size());
      // assert((*it_next)->wps.size() >= 2);
      float diff = angleDiff((*it)->mod_config[PAP::Angle], (*it_next)->mod_config[PAP::Angle]);
      // debug("Diff: ", diff);
      rotationCost +=  diff/180;
      rotations += diff;
      // debug(rotationCost);
    }
  }
  // debug("Dist: ", traveledDist, " Cross: ", cross, " PathLen: ", pathLengh);
  // Finalize rotation costs
  rotationCost /= actions.size();
  // debug(rotationCost);
  auto end = actions.back();
  // Panilty if endpoint cannot be reached
  // increase the cross count because the same distance needs to be taken twice
  // (robot is driving backwards until it reaches the startpoint)
  if (end->wps.back() != end->endPoint){
    cross += cross;
    reachEnd = false;
  }
  // convert rotations from deg -> rad
  rotations *= M_PI/180;
  return pathLengh > 0;
}

void genome_tools::genome::setPathSignature(shared_ptr<GridMap> gmap){
  mat = make_shared<Matrix>(Matrix(gmap->get("map")));
}

void genome_tools::validateGen(genome &gen){
 for(auto it = gen.actions.begin(); it != gen.actions.end(); it++){
   // What is needed to validate the gens?
   // Gens are not validated after initialization!
   // Gens need to be validated after crossover to fix the crossing
   // Consequently each gen that is flagged as modified needs to be adapted or its changes need to be
   // propagated to the next Gen
   if(!(*it)->modified) {
     // debug(((*it)->wps.front() - (*it)->wps.back()).norm(), " -- ", (*it)->mod_config[PAP::Distance]);
     // assert(((*it)->wps.front() - (*it)->wps.back()).norm() - (*it)->mod_config[PAP::Distance] >= -0.001
     // 	    and ((*it)->wps.front() - (*it)->wps.back()).norm() - (*it)->mod_config[PAP::Distance] <= 0.001);
     continue;
   }

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

int genome_tools::countZeroActions(genome &gen){
  int res = 0;
  for (auto it = gen.actions.begin(); it != gen.actions.end(); ++it) {
    if(compareF(it->get()->mod_config[PAP::Distance], 0, 0.000001)){
      res += 1;
    }
  }
  return res;
}

float genome_tools::calZeroActionPercent(genome &gen){
  // float res = 0;
  // for (auto it = gen.actions.begin(); it != gen.actions.end(); ++it) {
  //   if(compareF(it->get()->mod_config[PAP::Distance], 0, 0.000001)){
  //     res += 1;
  //   }
  // }
  return countZeroActions(gen) / (gen.actions.size()-1);
}

float genome_tools::calZeroActionPercent(Genpool_shr &pool){
  // TODO: Does nothing!!
  float res = 0;
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    res += calZeroActionPercent(**it);
  }
  return res / pool.size();
}

void genome_tools::removeZeroPAs(Genpool_shr &pool) {
  for(auto gen : pool){
    removeZeroPAs(*gen);
  }
}

int genome_tools::countDeadGens(Genpool_shr &pool, int minSize){
  int res = 0;
  for (auto it = pool.begin(); it != pool.end(); ++it) {
    if ((*it)->actions.size() < minSize or not (*it)->updateGenParameter())
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

void genome_tools::calDistanceMat(Genpool_shr &pool, Eigen::VectorXf& upperFlat){
  int pSize = pool.size();
  assert(D.cols() == pSize);
  assert(D.rows() == pSize);
  // assert(pSize*(pSize +1)/2 == upperFlat.size());

  int idx = 0;
  for(int row=0; row < pSize-1; row++){
    // pool[row].diversityFactor = 0;
    for(int col=row+1; col<pSize; col++){
      // TODO: Only possible pixel
      float res = (*pool[row]->mat - *pool[col]->mat).norm();
      // debug("Row: ", row, " Col: ", col, " dist: ", res);
      // assert(res == (*pool[col].mat -*pool[row].mat).norm());
      upperFlat[row] += res;
      upperFlat[col] += res;
      pool[row]->diversityFactor += res;
      pool[col]->diversityFactor += res;
      idx++;
    }
  }
}


void genome_tools::getDivMeanStd(Genpool_shr &pool, float& mean, float& stdev, float &min_, float &max_){
  int pSize = pool.size();
  Eigen::VectorXf upperFlat(pSize);
  // debug("..", upperFlat);
  upperFlat.setZero();
  // debug("..", upperFlat);
  calDistanceMat(pool, upperFlat);
  mean = upperFlat.mean();
  min_ = upperFlat.minCoeff();
  max_ = upperFlat.maxCoeff();
  stdev = sqrt((upperFlat.array() - mean).square().sum()/(upperFlat.size()-1));
}
