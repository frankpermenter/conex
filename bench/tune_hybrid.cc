// Tuning program for the hybrid geodesic IPM switching policy.
//
// Creates LP problems centered on the central path, then runs
// Solver::Solve(PhaseOneHybrid{...policy...}) with different switching
// policies. Measures factorizations + solves to reach target eps
// accuracy (all KKT residuals < eps).
//
// Usage:
//   ./tune_hybrid [--eps 1e-6] [--n 20] [--m 40] [--seeds 10]

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "kkt_residuals.h"

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

// =====================================================================
// Problem generation: LP centered on the central path at W=I, k=1.
// =====================================================================

struct TestLP {
  Model model;
  MatrixXd A_dense;
  VectorXd b;
  VectorXd c;
  int n, m;
};

TestLP MakeCenteredLP(int n, int m, int seed, double noise = 0) {
  srand(seed);
  TestLP lp;
  lp.n = n;
  lp.m = m;
  lp.A_dense = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);

  // Central path at W = diag(e + perturbation).
  VectorXd w = VectorXd::Ones(m);
  if (noise > 0) {
    w += noise * VectorXd::Random(m).cwiseAbs();
  }
  lp.b = w;
  lp.c = lp.A_dense.transpose() * w;

  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, lp.A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  lp.model.AddLinearConstraint(A, lp.b, vars);
  lp.model.SetLinearCost(lp.c);
  return lp;
}

// =====================================================================
// Switching policies.
// =====================================================================

// Default: center if gap < 0.
HybridSwitchPolicy MakeDefaultPolicy() {
  return DefaultHybridPolicy;
}

// Center if gap < 0 OR d_inf > threshold.
HybridSwitchPolicy MakeCenterOnLargeDInf(double threshold) {
  return [threshold](double gap, double d_inf, int) {
    return gap < 0 || d_inf > threshold;
  };
}

// Always center.
HybridSwitchPolicy MakeAlwaysCenter() {
  return [](double, double, int) { return true; };
}

// Always shrink.
HybridSwitchPolicy MakeAlwaysShrink() {
  return [](double, double, int) { return false; };
}

// Max r-updates before forced centering.
HybridSwitchPolicy MakeMaxRUpdates(int max_r) {
  return [max_r](double gap, double, int r_updates) {
    return gap < 0 || r_updates >= max_r;
  };
}

// Alternate: center every K-th shrink step.
HybridSwitchPolicy MakeAlternateEveryK(int k) {
  auto step = std::make_shared<int>(0);
  return [k, step](double gap, double, int) {
    if (gap < 0) return true;
    (*step)++;
    return (*step % k) == 0;
  };
}

// =====================================================================
// Run one policy and extract residuals via Solver::Solve.
// =====================================================================

struct TuningResult {
  int factorizations = 0;
  // Algorithm-internal convergence: |gap| < eps and d_inf <= 1.
  double gap = 0;
  double d_inf = 0;
  double mu = 0;
  bool converged = false;       // internal: |gap| < eps && d_inf <= 1
  // Model-space KKT residuals (for reference).
  KKTResiduals model_residuals;
};

TuningResult RunPolicy(const TestLP& lp, HybridSwitchPolicy policy,
                       double eps) {
  auto solver = Solver::Build(lp.model);

  HybridOnly algo;
  algo.tolerance = eps;
  algo.policy = policy;
  auto result = solver.Solve(algo);

  TuningResult out;
  out.factorizations = result.factorizations;
  out.gap = result.gap;
  out.d_inf = result.d_inf;
  out.mu = result.mu;
  out.converged = std::abs(result.gap) < eps && result.d_inf <= 1.001;

  // Model-space residuals for reference.
  if (!result.duals.lambda.empty()) {
    out.model_residuals = ComputeLinearKKTResiduals(
        lp.model, result.x, result.duals.lambda);
  }
  return out;
}

// =====================================================================
// Main
// =====================================================================

void PrintHeader() {
  printf("%-22s | %4s | %10s %8s %10s | %10s %10s | %s\n",
         "Policy", "fac", "gap", "d_inf", "mu",
         "kkt_compl", "kkt_stat", "conv");
  printf("%s\n", std::string(100, '-').c_str());
}

void PrintResult(const char* name, const TuningResult& r) {
  printf("%-22s | %4d | %10.2e %8.4f %10.2e | %10.2e %10.2e | %s\n",
         name, r.factorizations, r.gap, r.d_inf, r.mu,
         r.model_residuals.complementarity, r.model_residuals.stationarity,
         r.converged ? "yes" : "NO");
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  int n = 20, m = 40, num_seeds = 10;
  double eps = 1e-6;
  double noise = 0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--eps" && i + 1 < argc) eps = std::atof(argv[++i]);
    else if (arg == "--n" && i + 1 < argc) n = std::atoi(argv[++i]);
    else if (arg == "--m" && i + 1 < argc) m = std::atoi(argv[++i]);
    else if (arg == "--seeds" && i + 1 < argc) num_seeds = std::atoi(argv[++i]);
    else if (arg == "--noise" && i + 1 < argc) noise = std::atof(argv[++i]);
    else {
      printf("Usage: %s [--eps 1e-6] [--n 20] [--m 40] [--seeds 10] [--noise 0.5]\n", argv[0]);
      return 1;
    }
  }

  printf("Hybrid switching policy tuning\n");
  printf("Problem: LP with n=%d vars, m=%d constraints, eps=%.0e, noise=%.1f\n\n",
         n, m, eps, noise);

  struct PolicyDef {
    const char* name;
    conex::HybridSwitchPolicy policy;
  };

  PolicyDef policies[] = {
    {"default (gap<0)",   conex::MakeDefaultPolicy()},
    {"center d_inf>1.5",  conex::MakeCenterOnLargeDInf(1.5)},
    {"center d_inf>1.1",  conex::MakeCenterOnLargeDInf(1.1)},
    {"always center",     conex::MakeAlwaysCenter()},
    {"max_r=1",           conex::MakeMaxRUpdates(1)},
    {"max_r=3",           conex::MakeMaxRUpdates(3)},
    {"max_r=5",           conex::MakeMaxRUpdates(5)},
    {"alternate(2)",      conex::MakeAlternateEveryK(2)},
    {"always shrink",     conex::MakeAlwaysShrink()},
  };
  const int num_policies = sizeof(policies) / sizeof(policies[0]);

  // Aggregate.
  struct Aggregate {
    int total_fac = 0;
    int num_converged = 0;
  };
  std::map<std::string, Aggregate> agg;

  for (int seed = 1; seed <= num_seeds; ++seed) {
    auto lp = conex::MakeCenteredLP(n, m, seed, noise);
    printf("=== seed=%d ===\n", seed);
    conex::PrintHeader();
    for (int p = 0; p < num_policies; ++p) {
      auto r = conex::RunPolicy(lp, policies[p].policy, eps);
      conex::PrintResult(policies[p].name, r);
      auto& a = agg[policies[p].name];
      a.total_fac += r.factorizations;
      if (r.converged) a.num_converged++;
    }
    printf("\n");
  }

  // Summary.
  printf("==========================================\n");
  printf("SUMMARY (averaged over %d seeds, n=%d, m=%d, eps=%.0e)\n\n",
         num_seeds, n, m, eps);
  printf("%-22s | %6s | %5s\n", "Policy", "fac", "conv");
  printf("%s\n", std::string(40, '-').c_str());
  for (int p = 0; p < num_policies; ++p) {
    auto& a = agg[policies[p].name];
    printf("%-22s | %6.1f | %3d/%d\n",
           policies[p].name,
           (double)a.total_fac / num_seeds,
           a.num_converged, num_seeds);
  }
  printf("\n");

  return 0;
}
