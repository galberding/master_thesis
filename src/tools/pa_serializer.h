#ifndef PA_SERIALIZER_H
#define PA_SERIALIZER_H

#include "path_tools.h"
#include <iostream>
#include <fstream>

using namespace std;
namespace fs = std::filesystem;

namespace pa_serializer {
  bool writeActionsToFile(vector<path::PAs>& paths, const fs::path& p);
  bool readActrionsFromFile(vector<path::PAs>& paths, const fs::path& p);
}
#endif /* PA_SERIALIZER_H */
