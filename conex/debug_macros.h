#pragma once
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

namespace conex {

#define DUMP(x)                                              \
  std::cerr << __FILE__ << " line " << __LINE__ << std::endl \
            << #x ":" << std::endl                           \
            << x << std::endl;

#ifndef CONEX_VERBOSE
#define CONEX_VERBOSE 1
#endif

#ifndef CONEX_ENABLE_TIMER
#define CONEX_ENABLE_TIMER 1
#endif

#if CONEX_VERBOSE
#define LOG(x)        \
  std::cout << #x ":" \
            << " " << x << ", ";

#define REPORT(x)     \
  std::cout << #x ":" \
            << " " << x << ", ";

#define PRINTSTATUS(x) std::cout << "Status: " << x << "\n\n";


#define START_LOG_TIMER \
  {                                                          \
    auto start1 = std::chrono::high_resolution_clock::now(); \

#define END_LOG_TIMER(x)                                                            \
  auto stop1 = std::chrono::high_resolution_clock::now();                    \
   x =  std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start1).count(); }

#if CONEX_ENABLE_TIMER
#define START_TIMER(x)                                       \
  {                                                          \
    auto start1 = std::chrono::high_resolution_clock::now(); \
    std::cerr << #x << "(us)"                                \
              << ":";

#define END_TIMER                                                            \
  auto stop1 = std::chrono::high_resolution_clock::now();                    \
  std::cerr << " "                                                           \
            << std::chrono::duration_cast<std::chrono::microseconds>(stop1 - \
                                                                     start1) \
                   .count()                                                  \
            << ", " << std::endl;                                                         \
  }
#else

#define START_TIMER(x)
#define END_TIMER
#endif

#else

#define LOG(x)
#define REPORT(x)
#define PRINTSTATUS(x)
#define START_TIMER(x)
#define END_TIMER

#endif

template <typename T1, typename T2>
inline std::ostream& operator<<(std::ostream& os, const std::pair<T1, T2>& P) {
  os << P.first << " " << P.second << "\n\n";
  return os;
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const std::vector<T>& P) {
  os << "\n{";
  if (P.size() > 0) {
    os << P.at(0);
  }
  for (size_t i = 1; i < P.size(); i++) {
    os << "," << P.at(i);
  }
  os << "}\n";
  return os;
}

template <typename T, int size>
inline std::ostream& operator<<(std::ostream& os,
                                const std::array<T, size>& P) {
  for (auto e : P) {
    os << e << "\n\n";
  }
  return os;
}

}  // namespace conex
