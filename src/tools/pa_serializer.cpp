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
	  << "(" << start[0] << "|" << start[1] << ")" << ",";
	break;
      }
      case PAT::Ahead: case PAT::CAhead:{
	// Syntax type|(x1|y1)|(x2|y2),
	Position start = (*it)->wps.front();
	Position end = (*it)->wps.back();
	f << static_cast<int>((*it)->type) << "|"
	  << "(" << start[0] << "|" << start[1] << ")" << "|"
	  << "(" << end[0] << "|" << end[1] << ")" << ",";
	break;
      }
      case PAT::End:{
	// TODO: Not completely correct, endpoints are still ignored
	// Syntax type|(x1|y1)\n
	Position start = (*it)->wps.front();
	f << static_cast<int>((*it)->type)<< "|"
	  << "(" << start[0] << "|" << start[1] << ")" << "\n";
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
