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

#define DUMPINIT(x)                                                            \
  std::cerr << __FILE__ << " line " << __LINE__ << std::endl                   \
            << #x                                                              \
            << x.format(Eigen::IOFormat(Eigen::StreamPrecision, 1, ", ", ", ", \
                                        "", "", " << ", ";"))                  \
            << std::endl;

#ifndef CONEX_VERBOSE
#define CONEX_VERBOSE 1
#endif

#ifndef CONEX_VERBOSE
#define CONEX_TIMER 1
#endif

#if CONEX_VERBOSE
#define LOG(x)        \
  std::cout << #x "," \
            << " " << x << ", ";

#define REPORT(x)     \
  std::cout << #x "," \
            << " " << x << ", ";

#define PRINTSTATUS(x) std::cout << "Status: " << x << "\n\n";

#else

#define LOG(x)
#define REPORT(x)
#define PRINTSTATUS(x)

#endif

#if CONEX_TIMER

#define START_TIMER(x)                                       \
  {                                                          \
    auto start1 = std::chrono::high_resolution_clock::now(); \
    std::cout << #x << "(us)"                                \
              << ",";

#define END_TIMER                                                            \
  auto stop1 = std::chrono::high_resolution_clock::now();                    \
  std::cout << " "                                                           \
            << std::chrono::duration_cast<std::chrono::microseconds>(stop1 - \
                                                                     start1) \
                   .count()                                                  \
            << ", ";                                                         \
  }
#else

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
  os << "\n--\n";
  for (auto e : P) {
    os << e << "\n-\n";
  }
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
