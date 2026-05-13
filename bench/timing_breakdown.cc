#include <chrono>
#include <cstdio>
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/sdpa_reader.h"
#include "conex/algorithms/solve_strategies.h"

using hrclock = std::chrono::high_resolution_clock;

int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "Usage: timing_breakdown <file.dat-s> [algorithm]\n"); return 1; }
  const char* alg_name = (argc > 2) ? argv[2] : "lp";

  auto t0 = hrclock::now();
  auto [model, info] = conex::ReadSDPA(argv[1]);
  auto t_read = hrclock::now();
  auto solver = conex::Solver::Build(model);
  auto t_build = hrclock::now();

  auto us = [](auto a, auto b) {
    return std::chrono::duration<double, std::micro>(b - a).count();
  };

  auto print_stats = [&](const char* name, const conex::SolveResult& result,
                          const conex::SolveStats& stats, double total_us) {
    printf("\n=== %s on %s ===\n", name, argv[1]);
    printf("Read:         %10.0f us\n", us(t0, t_read));
    printf("Build:        %10.0f us\n", us(t_read, t_build));
    printf("Total solve:  %10.0f us  (%d iter, %d fac)\n",
           total_us, result.iterations, result.factorizations);
    printf("  Factor:     %10.0f us  (%d calls, %.0f us/call)\n",
           stats.factor_us, stats.factor_count,
           stats.factor_count > 0 ? stats.factor_us / stats.factor_count : 0.0);
    printf("  Solve:      %10.0f us  (%d calls, %.0f us/call)\n",
           stats.solve_us, stats.solve_count,
           stats.solve_count > 0 ? stats.solve_us / stats.solve_count : 0.0);
    printf("  Cone ops:   %10.0f us\n", stats.cone_us);
    double accounted = stats.factor_us + stats.solve_us + stats.cone_us;
    printf("  Other:      %10.0f us\n", total_us - accounted);
    printf("Objective:    %.6f\n", result.objective);
    printf("Gap:          %.2e\n", result.gap);
  };

  std::string alg(alg_name);

  if (alg == "lp") {
    conex::GeodesicLP algo{1e-6, 30, 0, false};
    auto t1 = hrclock::now();
    auto result = solver.Solve(algo);
    auto t2 = hrclock::now();
    print_stats("GeodesicLP", result, algo.stats, us(t1, t2));
  } else if (alg == "barrierlp") {
    conex::GeodesicBarrierLP algo{1e-6, 30, 0, false};
    auto t1 = hrclock::now();
    auto result = solver.Solve(algo);
    auto t2 = hrclock::now();
    print_stats("BarrierLP", result, algo.stats, us(t1, t2));
  } else if (alg == "thetacont") {
    conex::ThetaContinuation algo{1e-6, 50, 10, false};
    auto t1 = hrclock::now();
    auto result = solver.Solve(algo);
    auto t2 = hrclock::now();
    print_stats("ThetaCont", result, algo.stats, us(t1, t2));
  } else if (alg == "thetacontr") {
    conex::ThetaContinuationR algo{1e-6, 100, false};
    auto t1 = hrclock::now();
    auto result = solver.Solve(algo);
    auto t2 = hrclock::now();
    print_stats("ThetaContR", result, algo.stats, us(t1, t2));
  } else {
    fprintf(stderr, "Unknown: %s (lp, barrierlp, thetacont, thetacontr)\n", alg_name);
    return 1;
  }
  return 0;
}
