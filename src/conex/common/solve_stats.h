#pragma once
#include <chrono>

namespace conex {

struct SolveStats {
  double factor_us = 0;
  double solve_us = 0;
  double cone_us = 0;
  double other_us = 0;
  int factor_count = 0;
  int solve_count = 0;
};

// Scoped timer: accumulates elapsed time into *target on destruction.
// Zero cost when target == nullptr.
struct ScopedTimer {
  double* target;
  std::chrono::high_resolution_clock::time_point start;

  explicit ScopedTimer(double* t)
      : target(t),
        start(t ? std::chrono::high_resolution_clock::now()
                : std::chrono::high_resolution_clock::time_point{}) {}

  ~ScopedTimer() {
    if (target) {
      *target += std::chrono::duration<double, std::micro>(
          std::chrono::high_resolution_clock::now() - start).count();
    }
  }

  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;
};

// Convenience: create a ScopedTimer for a stats field.
// Usage: CONEX_TIMER(stats, factor_us);
#define CONEX_TIMER(stats_ptr, field) \
  ::conex::ScopedTimer _timer_##field( \
      (stats_ptr) ? &(stats_ptr)->field : nullptr)

}  // namespace conex
