#include "crossover.h"


/////////////////////////////////////////////////////////////////////////////
//                            CrossoverStrategy                             //
/////////////////////////////////////////////////////////////////////////////
void cross::DualPointCrossover::operator()(SelectionPool& selPool, Genpool& nextPool , executionConfig& eConf){
  assert(selPool.size() > 0);
  for (auto it = selPool.begin(); it != selPool.end(); ++it) {
    if(!applyAction(eConf.crossoverProba, eConf) or !mating(it->first, it->second, nextPool, eConf)){
      // Add gens to pool if they are not already inside
      if(std::find(nextPool.begin(), nextPool.end(), it->first)
	 != nextPool.end())
	nextPool.push_back(it->first);
      if(std::find(nextPool.begin(), nextPool.end(), it->second)
	 != nextPool.end())
	nextPool.push_back(it->second);
	continue;
    }
    assert(it->first.actions.size() > 3);
    assert(it->second.actions.size() > 3);
    // mating(it->first, it->second, nextPool, eConf);
  }
}

void cross::DualPointCrossover::operator()(FamilyPool& fPool, Genpool& pool , executionConfig& eConf){
  assert(fPool.size() > 0);
  for (auto &family : fPool) {
    assert(family.size() == 2);
    // Check if crossover can be performed
    if(!applyAction(eConf.crossoverProba, eConf) or !mating(family[0], family[1], family, eConf)){
      // Add gens to pool if they are not already inside
      pool.push_back(family[0]);
      pool.push_back(family[1]);
	continue;
    }
    // assert(family[0].actions.size() > 3);
    // assert(family[1].actions.size() > 3);
    // mating(family[0], family[1], family, eConf);
    // assert(family.size() >= 4);
  }
  if(pool.size() > 0)
    sort(pool.begin(), pool.end());

}


void cross::CrossoverStrategy::copyActions(PAs::iterator begin, PAs::iterator end, PAs &child, bool modify){
  for(begin; begin != end; begin++){
    // debug("Type: ", (int) (*begin)->type);
    assert((*begin)->wps.size() > 0);
    switch((*begin)->type){
    case PAT::Start:{
      StartAction sa((*begin)->wps.front());
      child.push_back(make_shared<StartAction>(sa));
      break;
    }
    case PAT::Ahead: case PAT::CAhead:{
      AheadAction aa((*begin)->type, (*begin)->mod_config);
      aa.generateWPs((*begin)->wps.front());
      aa.modified = modify;
      child.push_back(make_shared<AheadAction>(aa));
      break;
    }
    case PAT::End:
      child.push_back(make_shared<EndAction>(EndAction((*begin)->wps)));
    }
  }
}

genome cross::DualPointCrossover::getChild(PAs par1, PAs par2, int sIdx[2], int len[2], bool move){
  PAs child;

  copyActions(par1.begin(),
	      next(par1.begin(), sIdx[0]),
	      child);
  // Insert cross over part from parent 2
  // Mark the inserted part as modified to recalculate waypoints
  copyActions(next(par2.begin(), sIdx[1]),
	      next(par2.begin(), (sIdx[1]+len[1])),
	      child, move); // if move is true we loose locality
  // Append remaining part
  copyActions(next(par1.begin(), sIdx[0] + len[0]),
	      par1.end(),
	      child);

  // Check if copy was successful
  assert(child.size() == (par1.size() - len[0] + len[1]));

  // Mark actions as
  (*next(child.begin(), sIdx[0] - 1))->modified = true;
  (*next(child.begin(), sIdx[0] + len[1] - 1))->modified = true;
  genome child_gen(child);
  validateGen(child_gen);
  assert(child_gen.actions.front()->type == PAT::Start
	 && child_gen.actions.back()->type == PAT::End);
  return child_gen;
}

template <typename T>
bool cross::DualPointCrossover::mating(genome &par1, genome &par2, T& newPopulation, executionConfig& eConf){
  // mate two parents
  // estimate the individual Length
  int minActionCount = eConf.getMinGenLen();

  if (par1.actions.size() < minActionCount
      or par2.actions.size() < minActionCount)
    return false;
  PAs parent1, parent2, child1, child2, child3, child4;
  int maxlen1 = static_cast<int>((par1.actions.size() -1) * eConf.crossLength);
  int maxlen2 = static_cast<int>((par2.actions.size() -1) * eConf.crossLength);
  // debug("Parent1: ", par1.actions.size(), " Parent2: ", par2.actions.size(), " ", maxlen1, " ", maxlen2, " ", minActionCount);
  uniform_int_distribution<int> lendist1(1, maxlen1);
  uniform_int_distribution<int> lendist2(1, maxlen2);

  int len[2];
  int len1 = len[0] = lendist1(eConf.generator);
  int len2 = len[1] = lendist2(eConf.generator);
  // Ensure that the generated index in still in range


  uniform_int_distribution<int> dist1(1,par1.actions.size() - (len1+1));
  uniform_int_distribution<int> dist2(1,par2.actions.size() - (len2+1));
  // calculate the start Index
  int sIdx[2];
  int sIdx1 = sIdx[0] = dist1(eConf.generator);
  int sIdx2= sIdx[1] = dist2(eConf.generator);

  assert(sIdx[0] + len1 < par1.actions.size());
  assert(sIdx[1] + len2 < par2.actions.size());

  vector<genome> loc, glob;

  // Calculate children for first parent
  loc.push_back(getChild(par1.actions, par2.actions, sIdx, len, true));
  glob.push_back(getChild(par1.actions, par2.actions, sIdx, len, false));
  sIdx[0] = sIdx2;
  sIdx[1] = sIdx1;
  len[0] = len2;
  len[1] = len1;
  // Calculate children for second parent
  loc.push_back(getChild(par2.actions, par1.actions, sIdx, len, true));
  glob.push_back(getChild(par2.actions, par1.actions, sIdx, len, false));

  // Insert to new Population
  switch(eConf.crossChildSelector){
  case 0:{
    newPopulation.insert(newPopulation.end(), loc.begin(), loc.end());
    break;
  }
  case 1:{
    newPopulation.insert(newPopulation.end(), glob.begin(), glob.end());
    break;
  }
  case 2:{
    newPopulation.insert(newPopulation.end(), loc.begin(), loc.end());
    newPopulation.insert(newPopulation.end(), glob.begin(), glob.end());
    break;
  }
  default:{
    assertm(false, "Wrong value for child selector");
  }
  }
  return true;
}
