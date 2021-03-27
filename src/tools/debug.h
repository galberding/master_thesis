#ifndef DEBUG_H
#define DEBUG_H
#undef NDEBUG
#include <iostream>
#include <cstdio>
#include <cassert>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <vector>
#include <any>

#define assertm(exp, msg) assert(((void)msg, exp))

#define __DEBUG__ true
#define __INFO__ true
#define __WARN__ true

#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define CYAN    "\033[36m"



template<class... Args>
inline void print(Args... args);

template<class... Args>
void print2(Args... args) {
  std::cout << CYAN << "[MUHAHA]" << RESET  << ": ";
  (std::cout << ... << args) << "\n";
}


template<class ... Args>
void debug(Args ... args)
{
#if __DEBUG__
  std::cout << CYAN << "[DEBUG]" << RESET  << ": ";
  (std::cout << ... << args) << "\n";
#endif
}


template<class... Args>
void info(Args... args)
{
#if __INFO__
  std::cout << GREEN << "[INFO]" << RESET  << ": ";
  (std::cout << ... << args) << "\n";
#endif
}

template<class... Args>
void warn(Args... args)
{
#if __WARN__
  std::cout << RED << "[WARN]" << RESET  << ": ";
  (std::cout << ... << args) << "\n";
#endif
}


template<class... Args>
std::string argsToCsv(Args... args)
{
  std::ostringstream msg;
  ((msg << args << ","), ...);
  return msg.str().substr(0, msg.str().size() - 1) + "\n";
}


// template<class... Args>
// std::string argsToCsv(Args... args)
// {
//   std::vector<std::any> vec = {args...};
//   std::stringstream msg;

//   for(auto it = vec.begin(); it != vec.end();){
//     msg << (*it).

//   }
// }


namespace logging{

  // The log config should be created to hold information about the current run
  // What kind of information do we need?

  // There are three kinds of information:
  // - Start comnfiguration
  // - Runtime information
  // - Completion information (can also be represented by the runtime information)

  // lets go with just the name for now!

  inline std::string getCurrentDateTime( std::string s );
  void Logger(std::string logMsg, std::string dir, std::string name, bool append=false);

}





#endif /* DEBUG_H */
