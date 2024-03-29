#include "debug.h"

// template<class... Args>
// void print(Args... args) {
//   std::cout << CYAN << "[DEBUG]" << RESET  << ": ";
//   (std::cout << ... << args) << "\n";
// }

namespace fs = std::filesystem;



inline std::string logging::getCurrentDateTime( std::string s ){
    time_t now = time(0);
    struct tm  tstruct;
    char  buf[80];
    tstruct = *localtime(&now);
    if(s=="now")
        strftime(buf, sizeof(buf), "%Y-%m-%d,%X", &tstruct);
    else if(s=="date")
        strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
    return std::string(buf);
  };

void logging::Logger(std::string logMsg, std::string dir, std::string name, bool append){

  fs::create_directories(dir);

  std::string filePath = dir + "/" + name + ".csv";
  std::string now = getCurrentDateTime("now");
  std::ofstream ofs;
  // ofs.open(filePath.c_str(), std::ios_base::out | std::ios_base::app);
  ofs.open(filePath.c_str(), append ? std::ios_base::out | std::ios_base::app : std::ios_base::out);
  // std::string endMsg = ((logMsg[logMsg.size() - 1] != '\n') ? '\n' : "");

  // ofs << now << ',' << logMsg;
  ofs << logMsg;
  if(logMsg[logMsg.size() - 1] != '\n'){
    ofs << std::endl;
  }

  ofs.close();
}
