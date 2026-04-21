// Run HybridOnly with different policies on a QPS benchmark instance.
//
// Usage: ./tune_qps <file.qps> [--eps 1e-6]

#include <cstdio>
#include <cstdlib>
#include <string>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/qps_reader.h"
#include "conex/common/solver.h"

using namespace conex;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    printf("Usage: %s <file.qps> [--eps 1e-6] [--verbose]\n", argv[0]);
    return 1;
  }
  std::string path = argv[1];
  double eps = 1e-6;
  bool verbose = false;
  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--eps" && i + 1 < argc) eps = std::atof(argv[++i]);
    if (arg == "--verbose") verbose = true;
  }

  auto [model, info] = ReadQPS(path);
  printf("QPS: %s  vars=%d eq=%d ineq=%d quad=%d bounds=%d\n",
         info.name.c_str(), info.num_variables,
         info.num_equality_rows, info.num_inequality_rows,
         info.num_quadratic_entries, info.num_bounded_vars);

  SolverConfiguration cfg;
  cfg.row_scale = true;
  auto solver = Solver::Build(model, cfg);
  printf("KKT vars=%d (reduced=%s)\n\n",
         solver.kkt()->number_of_variables(),
         solver.was_reduced() ? "yes" : "no");

  struct PolicyDef {
    const char* name;
    HybridSwitchPolicy policy;
  };
  PolicyDef policies[] = {
    {"default (gap<0)", DefaultHybridPolicy},
    {"center d_inf>1.5", [](double g, double d, int) { return g < 0 || d > 1.5; }},
    {"center d_inf>1.1", [](double g, double d, int) { return g < 0 || d > 1.1; }},
    {"max_r=1", [](double g, double, int r) { return g < 0 || r >= 1; }},
    {"max_r=3", [](double g, double, int r) { return g < 0 || r >= 3; }},
    {"max_r=5", [](double g, double, int r) { return g < 0 || r >= 5; }},
  };

  printf("%-22s | %4s | %10s %8s %10s | %s\n",
         "Policy", "fac", "gap", "d_inf", "mu", "conv");
  printf("%s\n", std::string(75, '-').c_str());

  bool first = true;
  for (auto& [name, policy] : policies) {
    // Rebuild solver for each policy (fresh state).
    auto s = Solver::Build(model, cfg);
    HybridOnly algo;
    algo.tolerance = eps;
    algo.verbose = (verbose && first);
    algo.policy = policy;
    first = false;
    auto result = s.Solve(algo);

    bool conv = std::abs(result.gap) < eps && result.d_inf <= 1.001;
    printf("%-22s | %4d | %10.2e %8.4f %10.2e | %s\n",
           name, result.factorizations, result.gap, result.d_inf,
           result.mu, conv ? "yes" : "NO");
  }
  printf("\n");
}
