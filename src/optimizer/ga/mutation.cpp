#include "mutation.h"

/////////////////////////////////////////////////////////////////////////////
//                             MutationStrategy                            //
/////////////////////////////////////////////////////////////////////////////

void mut::MutationStrategy::mutateGen(genome &gen, executionConfig &eConf) {
    addOrthogonalAngleOffset(gen, eConf);
    addRandomAngleOffset(gen, eConf);
    addPositiveDistanceOffset(gen, eConf);
    addNegativeDistanceOffset(gen, eConf);
    randomScaleDistance(gen, eConf);
    // randomReplaceGen(gen, eConf);
}

void mut::MutationStrategy::operator()(Genpool& currentPool, executionConfig& eConf){
  for (auto &gen : currentPool) {
    mutateGen(gen, eConf);
  }
}

void mut::MutationStrategy::operator()(FamilyPool& fPool, executionConfig& eConf){
  for (auto &family : fPool) {
    if(family.size() == 2) continue;
    assert(family.size() >= 4);
    for(int i=2; i<family.size(); i++){
      mutateGen(family[i], eConf);
    }
    // mutateGen(family[3], eConf);
  }
}


bool mut::MutationStrategy::addOrthogonalAngleOffset(genome& gen, executionConfig& eConf){
  if(!applyAction(eConf.mutaOrtoAngleProba, eConf)) return false;
  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_int_distribution<int> changeDistro(0,1);

  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  (*action)->mod_config[PAP::Angle] += changeDistro(eConf.generator) ? 90 : -90;
  (*action)->modified = true;
  return true;
}

bool mut::MutationStrategy::addRandomAngleOffset(genome& gen, executionConfig& eConf) {
  if(!applyAction(eConf.mutaRandAngleProba, eConf)) return false;

  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_int_distribution<int> changeDistro(0,360);
  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  (*action)->mod_config[PAP::Angle] += changeDistro(eConf.generator);
  (*action)->modified = true;
  return true;
}

void mut::MutationStrategy::addPositiveDistanceOffset(genome& gen, executionConfig& eConf) {

  if(!applyAction(eConf.mutaPosDistProba, eConf)) return;
  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_real_distribution<> changeDistro(0,eConf.mutaPosDistMax);
  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  (*action)->mod_config[PAP::Distance] += changeDistro(eConf.generator);
  (*action)->modified = true;
}

void mut::MutationStrategy::addNegativeDistanceOffset(genome& gen, executionConfig& eConf) {

  if(!applyAction(eConf.mutaNegDistProba, eConf)) return;
  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_real_distribution<> changeDistro(0,eConf.mutaPosDistMax);
  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  float offset = changeDistro(eConf.generator);
  if((*action)->mod_config[PAP::Distance] > offset){
    (*action)->mod_config[PAP::Distance] -= offset;
    (*action)->modified = true;
  }
}


bool mut::MutationStrategy::randomScaleDistance(genome& gen, executionConfig& eConf) {

  if(!applyAction(eConf.mutaRandScaleDistProba, eConf)) return false;
  uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
  uniform_real_distribution<float> changeDistro(0.5,2);
  // Select action and add the offset
  auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
  float offset = changeDistro(eConf.generator);
  // debug("Mut scale: ", offset);
  if((*action)->mod_config[PAP::Distance] > 0){
    (*action)->mod_config[PAP::Distance] *= offset;
    (*action)->modified = true;
  }else{
    // Use mean traveled distance to revive a zero action
    gen.updateGenParameter();
    (*action)->mod_config[PAP::Distance] = (gen.traveledDist / gen.actions.size()) * offset;
  }
  return true;
}

bool mut::MutationStrategy::randomReplaceGen(genome& gen, executionConfig& eConf) {

  // Apply mutation if a apply action is true or if a gen is dead
  if(not(applyAction(eConf.mutaReplaceGen, eConf) or gen.actions.size() < eConf.getMinGenLen())) return false;
  InitStrategy init;

  // Check if average length if bigger than minimal allowed length
  // if(eConf.actionLenAvg < eConf.getMinGenLen())
  init(gen, eConf.getMinGenLen() * 2, eConf);
  // else
    // init(gen, eConf.actionLenAvg, eConf);
  validateGen(gen);
  return true;
}

// void mut::MutationStrategy::reviveZeroAction(genome& gen, executionConfig& eConf) {

//   if(!applyAction(eConf.mutaReviveAction, eConf)) return;
//   // debug("Insert Random!");
//   InitStrategy init;
//   init(gen, gen.actions.size(), eConf);
// }


// void mut::MutationStrategy::addRandomDistanceOffset(genome& gen, executionConfig& eConf) {
//   if(!applyAction(eConf.mutaRandDistProba, eConf)) return;
//   uniform_int_distribution<int> actionSelector(2,gen.actions.size()-1);
//   uniform_real_distribution<> changeDistro(0,eConf.mutaPosDistMax);

//   // Select action and add the offset
//   auto action = next(gen.actions.begin(), actionSelector(eConf.generator));
//   float offset = changeDistro(eConf.generator);
//   if(offset < 0 && (*action)->mod_config[PAP::Distance] < abs(offset)) return;
//     (*action)->mod_config[PAP::Distance] -= offset;
//     (*action)->modified = true;

// }
