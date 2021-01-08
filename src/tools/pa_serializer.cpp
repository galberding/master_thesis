#include "pa_serializer.h"

using namespace path;

bool pa_serializer::writeActionsToFile(vector<path::PAs> &paths, const fs::path& p){

  // Check if snapshot file exists


  // elements needs to be added to the string
  // id | x | y
  std::ostringstream f;
  std::ofstream ofs;
  for(auto &apath : paths){
    for (auto it = apath.begin(); it != apath.end(); ++it) {
      assert((*it)->wps.size() > 0);
      switch(it->get()->type){
      case PAT::Start:{
	// Syntax type|(x1|y1),
	Position start = (*it)->wps.front();
	f << static_cast<int>(PAT::Start)<< "|"
	   << start[0] << ":" << start[1] << ",";
	break;
      }
      case PAT::Ahead: case PAT::CAhead:{
	// Syntax type|(x1|y1)|(x2|y2),
	Position start = (*it)->wps.front();
	Position end = (*it)->wps.back();
	f << static_cast<int>((*it)->type) << "|"
	  << start[0] << ":" << start[1] << "|"
	  << end[0] << ":" << end[1] << ",";
	break;
      }
      case PAT::End:{
	// TODO: Not completely correct, endpoints are still ignored
	// Syntax type|(x1|y1)\n
	Position start = (*it)->wps.front();
	f << static_cast<int>((*it)->type)<< "|"
	  << start[0] << ":" << start[1] << "\n";
	break;
      }
      }
    }
  }

  // Write pool to file
  ofs.open(p, ios_base::out);
  ofs << f.str();
  ofs.close();
  if(!fs::exists(p)) return false;

  return true;
}

Position getPositionFromString(string pos){
  cout << "Position: " << pos << endl;
  vector<string> pp;
  stringstream ss(pos);
  string sub;
  while(ss.good()){
    getline(ss, sub, ':');
    pp.push_back(sub);
  }
  return Position(stof(pp[0]), stof(pp[1]));
}

shared_ptr<PathAction> getPAFromString(string action){
  shared_ptr<PathAction> pa;
  stringstream ss(action);
  string sub;
  vector<string> aParts;
  // Get id and points
  while(ss.good()){
    getline(ss, sub, '|');
    aParts.push_back(sub);
  }
  switch(stoi(aParts[0])){
  case static_cast<int>(PAT::Start):{
    assert(aParts.size() == 2);
    Position start = getPositionFromString(aParts[1]);
    pa = make_shared<StartAction>(StartAction(start));
    // cout << "Start" << start;
    break;
  }
  case static_cast<int>(PAT::Ahead): case static_cast<int>(PAT::CAhead):{
    assert(aParts.size() == 3);
    AheadAction aa(static_cast<PAT>(stoi(aParts[0])), {});
    Position start = getPositionFromString(aParts[1]);
    Position end = getPositionFromString(aParts[2]);
    aa.setConfigByWaypoints(start, end);
    pa = make_shared<AheadAction>(aa);
    break;
  }
  case static_cast<int>(PAT::End):{
    assert(aParts.size() == 2);
    Position start = getPositionFromString(aParts[1]);
    pa = make_shared<EndAction>(EndAction({start}));
    break;
  }
  }
  return pa;
}

bool pa_serializer::readActrionsFromFile(vector<path::PAs> &paths, const fs::path &p){

  ifstream Reader(p);
  string sequence;

  while(getline(Reader, sequence)){
    vector<string> actions;
    stringstream ss(sequence);
    PAs gen;
    while (ss.good()){
      string sub;
      getline(ss, sub, ',');
      cout << sub << endl;
      gen.push_back(getPAFromString(sub));
    }
    paths.push_back(gen);
  }
  return true;
}
