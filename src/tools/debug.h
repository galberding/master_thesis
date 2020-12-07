#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>
#include <cstdio>
#define __DEBUG__ true
#define __INFO__ true
// #define __WARN__ true

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



#endif /* DEBUG_H */
