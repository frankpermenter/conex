// Tuning program for the hybrid geodesic IPM switching policy.
//
// Creates LP problems centered on the central path, then runs the
// hybrid algorithm with different switching policies to find the
// one that minimizes factorizations + solves to reach a target
// epsilon accuracy (all KKT residuals < eps).
//
// Usage:
//   ./tune_hybrid [--eps 1e-6] [--n 20] [--m 40] [--seeds 10]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/common/kkt_system.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/test/kkt_residuals.h"

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

TestLP MakeCenteredLP(int n, int m, int seed) {
  srand(seed);
  TestLP lp;
  lp.n = n;
  lp.m = m;
  lp.A_dense = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  lp.b = VectorXd::Ones(m);
  // c = A^T 1 so the central path at W=I has d=0.
  lp.c = lp.A_dense.transpose() * VectorXd::Ones(m);

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
// Extract lambda from the hybrid algorithm's (W, r, delta) state.
// For nonneg cone: lambda = P(W^{1/2})(r + delta) / tau.
// =====================================================================

std::vector<VectorXd> ExtractLambdaFromRowSpace(
    const RowSpace& lambda_rs, const Model& model) {
  std::vector<VectorXd> result;
  const auto& lv = lambda_rs.col();
  int offset = 0;
  for (int i = 0; i < model.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        int m = data.A.rows();
        result.push_back(lv.segment(offset, m));
        offset += m;
      }
    }, model.constraint(i));
  }
  return result;
}

// =====================================================================
// Switching policies.
//
// A policy decides: given the current (gap, d_inf, d_sq, r_updates
// since last centering), should we center (update W) or shrink (update r)?
//
// The hybrid algorithm's default: center if gap < 0, else shrink.
// We parameterize alternatives.
// =====================================================================

struct PolicyResult {
  bool center;  // true = W update, false = r update
};

// Policy 1: Default (gap < 0 → center).
struct DefaultPolicy {
  const char* name() const { return "default (gap<0)"; }
  PolicyResult operator()(double gap, double d_inf, int r_updates) const {
    (void)d_inf; (void)r_updates;
    return {gap < 0};
  }
};

// Policy 2: Center if gap < 0 OR d_inf > threshold.
struct CenterOnLargeDInf {
  double d_inf_threshold = 1.5;
  const char* name() const { return "center if d_inf>1.5"; }
  PolicyResult operator()(double gap, double d_inf, int r_updates) const {
    (void)r_updates;
    return {gap < 0 || d_inf > d_inf_threshold};
  }
};

// Policy 3: Always center (no r-updates — baseline).
struct AlwaysCenter {
  const char* name() const { return "always center"; }
  PolicyResult operator()(double, double, int) const {
    return {true};
  }
};

// Policy 4: Alternate — center every K-th step, shrink otherwise.
struct AlternateEveryK {
  int k = 2;
  mutable int step = 0;
  const char* name() const { return "alternate(K)"; }
  PolicyResult operator()(double gap, double, int) const {
    if (gap < 0) return {true};
    step++;
    return {(step % k) == 0};
  }
};

// Policy 5: Max r-updates before forced centering.
struct MaxRUpdates {
  int max_r = 3;
  const char* name() const { return "max_r_updates"; }
  PolicyResult operator()(double gap, double, int r_updates) const {
    if (gap < 0) return {true};
    return {r_updates >= max_r};
  }
};

// Policy 6: Shrink only — never center after initial factorization.
struct AlwaysShrink {
  const char* name() const { return "always shrink"; }
  PolicyResult operator()(double, double, int) const {
    return {false};
  }
};

// =====================================================================
// Run the hybrid algorithm with a custom switching policy.
// Returns (factorizations, solves, converged, final residuals).
// =====================================================================

struct TuningResult {
  int factorizations = 0;
  int solves = 0;
  int iterations = 0;
  bool converged = false;
  KKTResiduals residuals;
  double time_ms = 0;
};

template <typename Policy>
TuningResult RunHybridWithPolicy(
    const TestLP& lp, Policy policy,
    double eps, int max_iterations = 500) {

  auto solver = Solver::Build(lp.model);
  auto* kkt = solver.kkt();

  auto cost_rhs = solver.MakeCostRHS();
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // Phase 1: get to theta=0 via PhaseOne.
  auto p1 = SolveGeodesicPhaseOne(*kkt, cost_rhs, W, max_iterations, 1, eps, false,
                                   /*phase1_only=*/true);
  double k_init = (p1.mu > 0) ? 1.0 / std::sqrt(p1.mu) : -1;
  double tau = p1.tau;

  // --- Now run the hybrid with custom switching ---
  RowSpace b = kkt->GetAffineTerm();
  const int m = b.total_rows();

  // Scale by tau.
  auto cost_scaled = kkt->MakeSolverRHS(); cost_scaled = cost_rhs;
  RowSpace b_unscaled = b;
  if (tau != 1.0) {
    b *= tau;
    cost_scaled *= tau;
  }

  RowSpace r = kkt->MakeRowSpace();
  setOnes(r);
  if (k_init > 0) {
    r *= (1.0 / k_init);
  }

  int total_fac = p1.total_factorizations;
  int total_sol = p1.total_solves;
  int r_updates_since_center = 0;

  kkt->SetScaling(W);
  kkt->AssembleAndFactor();
  total_fac++;

  auto t_start = std::chrono::high_resolution_clock::now();

  RowSpace last_delta = kkt->MakeRowSpace();
  double last_gap = 1e30, last_d_inf = 1e30;

  for (int iter = 0; iter < max_iterations; ++iter) {
    RowSpace d = kkt->MakeRowSpace();
    RowSpace delta = kkt->MakeRowSpace();
    auto info = ComputeHybridDirection(*kkt, cost_scaled, b, W, r, d, delta);
    last_delta = delta;
    total_sol++;

    if (info.d_inf > 10 || !std::isfinite(info.d_inf)) break;
    last_gap = info.gap;
    last_d_inf = info.d_inf;

    auto decision = policy(info.gap, info.d_inf, r_updates_since_center);

    if (decision.center) {
      double alpha = std::min(1.0, 2.0 / (info.d_inf * info.d_inf));
      updateAutomorphism(W, r, alpha, d);
      kkt->SetScaling(W);
      if (!kkt->AssembleAndFactor()) break;
      total_fac++;
      r_updates_since_center = 0;
    } else {
      shrinkR(r, delta);
      kkt->SetScaling(W);
      if (!kkt->AssembleAndFactor()) break;
      total_fac++;
      r_updates_since_center++;
    }

    // Recompute direction at updated state.
    {
      RowSpace d2 = kkt->MakeRowSpace();
      RowSpace delta2 = kkt->MakeRowSpace();
      auto info2 = ComputeHybridDirection(*kkt, cost_scaled, b, W, r, d2, delta2);
      last_delta = delta2;
      last_gap = info2.gap;
      last_d_inf = info2.d_inf;
      total_sol++;
    }

    // Internal convergence: gap ≈ 0 and d_inf ≤ 1.
    if (std::abs(last_gap) < eps && last_d_inf <= 1.001) break;
  }

  // Final: recover x and lambda, compute optimality via CheckOptimality.
  {
    kkt->SetScaling(W);
    kkt->AssembleAndFactor();
    total_fac++;

    // Solve for x.
    auto y = kkt->MakeSolverRHS();
    y = cost_scaled;
    y *= -1;
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace v = addScaled(quadraticRepresentation(W, b),
                           quadraticRepresentation(sqrtW, r), -1, 2.0);
    kkt->AccumulateAtranspose(v, y);
    kkt->SolveSolverRHS(y);
    total_sol++;
    int nr = kkt->number_of_variables();
    VectorXd x_reduced(nr);
    y.supernodes->GatherInto(x_reduced);
    if (tau != 1.0 && tau > 0) x_reduced /= tau;

    // Lambda and optimality via the internal CheckOptimality.
    auto x_rhs = kkt->MakeSolverRHS();
    x_rhs = kkt->MakeBlockVariable(x_reduced);
    RowSpace lambda_rs = quadraticRepresentation(sqrtW, r + last_delta);
    if (tau != 1.0 && tau > 0) lambda_rs *= (1.0 / tau);
    auto opt = CheckOptimality(*kkt, cost_rhs, x_rhs, lambda_rs);

    auto t_end = std::chrono::high_resolution_clock::now();
    TuningResult out;
    out.factorizations = total_fac;
    out.solves = total_sol;
    out.iterations = max_iterations;
    out.residuals.primal_infeasibility = opt.min_slack;
    out.residuals.dual_infeasibility = opt.min_dual;
    out.residuals.complementarity = opt.complementarity;
    out.residuals.stationarity = opt.dual_residual;
    out.converged = out.residuals.MaxResidual() < eps;
    out.time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return out;
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  TuningResult out;
  out.factorizations = total_fac;
  out.solves = total_sol;
  out.iterations = max_iterations;
  out.converged = false;
  out.time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  return out;
}

// =====================================================================
// Main: run all policies on a range of problems and report.
// =====================================================================

void PrintHeader() {
  printf("%-22s | %4s %5s %5s | %6s | %10s %10s %10s %10s\n",
         "Policy", "fac", "sol", "iter", "ms",
         "compl", "dual_res", "primal", "station");
  printf("%s\n", std::string(100, '-').c_str());
}

void PrintResult(const char* name, const TuningResult& r) {
  const char* tag = r.converged ? "" : " *";
  printf("%-22s | %4d %5d %5d | %6.1f | %10.2e %10.2e %10.2e %10.2e%s\n",
         name, r.factorizations, r.solves, r.iterations, r.time_ms,
         r.residuals.complementarity, r.residuals.stationarity,
         r.residuals.primal_infeasibility, r.residuals.dual_infeasibility,
         tag);
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  int n = 20, m = 40, num_seeds = 10;
  double eps = 1e-6;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--eps" && i + 1 < argc) eps = std::atof(argv[++i]);
    else if (arg == "--n" && i + 1 < argc) n = std::atoi(argv[++i]);
    else if (arg == "--m" && i + 1 < argc) m = std::atoi(argv[++i]);
    else if (arg == "--seeds" && i + 1 < argc) num_seeds = std::atoi(argv[++i]);
    else {
      printf("Usage: %s [--eps 1e-6] [--n 20] [--m 40] [--seeds 10]\n", argv[0]);
      return 1;
    }
  }

  printf("Hybrid switching policy tuning\n");
  printf("Problem: LP with n=%d vars, m=%d constraints, eps=%.0e\n\n", n, m, eps);

  // Aggregate results per policy.
  struct Aggregate {
    const char* name;
    int total_fac = 0, total_sol = 0;
    int num_converged = 0;
    double total_ms = 0;
  };

  // Define policies.
  auto run_all = [&](int seed) {
    auto lp = conex::MakeCenteredLP(n, m, seed);
    printf("=== seed=%d ===\n", seed);
    conex::PrintHeader();

    std::vector<std::pair<const char*, conex::TuningResult>> results;

    // Policy 1: Default.
    {
      auto r = conex::RunHybridWithPolicy(lp, conex::DefaultPolicy{}, eps);
      conex::PrintResult("default (gap<0)", r);
      results.push_back({"default", r});
    }

    // Policy 2: Center on large d_inf (threshold 1.5).
    {
      auto r = conex::RunHybridWithPolicy(lp, conex::CenterOnLargeDInf{1.5}, eps);
      conex::PrintResult("center if d_inf>1.5", r);
      results.push_back({"d_inf>1.5", r});
    }

    // Policy 3: Center on large d_inf (threshold 1.1).
    {
      auto r = conex::RunHybridWithPolicy(lp, conex::CenterOnLargeDInf{1.1}, eps);
      conex::PrintResult("center if d_inf>1.1", r);
      results.push_back({"d_inf>1.1", r});
    }

    // Policy 4: Always center (baseline).
    {
      auto r = conex::RunHybridWithPolicy(lp, conex::AlwaysCenter{}, eps);
      conex::PrintResult("always center", r);
      results.push_back({"always_center", r});
    }

    // Policy 5: Max 1 r-update.
    {
      auto r = conex::RunHybridWithPolicy(lp, conex::MaxRUpdates{1}, eps);
      conex::PrintResult("max_r=1", r);
      results.push_back({"max_r=1", r});
    }

    // Policy 6: Max 3 r-updates.
    {
      auto r = conex::RunHybridWithPolicy(lp, conex::MaxRUpdates{3}, eps);
      conex::PrintResult("max_r=3", r);
      results.push_back({"max_r=3", r});
    }

    // Policy 7: Max 5 r-updates.
    {
      auto r = conex::RunHybridWithPolicy(lp, conex::MaxRUpdates{5}, eps);
      conex::PrintResult("max_r=5", r);
      results.push_back({"max_r=5", r});
    }

    // Policy 8: Alternate every 2.
    {
      auto r = conex::RunHybridWithPolicy(lp, conex::AlternateEveryK{2}, eps);
      conex::PrintResult("alternate(2)", r);
      results.push_back({"alternate(2)", r});
    }

    // Policy 9: Always shrink.
    {
      auto r = conex::RunHybridWithPolicy(lp, conex::AlwaysShrink{}, eps);
      conex::PrintResult("always shrink", r);
      results.push_back({"always_shrink", r});
    }

    printf("\n");
    return results;
  };

  // Run across seeds.
  std::map<std::string, Aggregate> agg;
  for (int seed = 1; seed <= num_seeds; ++seed) {
    auto results = run_all(seed);
    for (auto& [name, r] : results) {
      auto& a = agg[name];
      a.name = name;
      a.total_fac += r.factorizations;
      a.total_sol += r.solves;
      a.total_ms += r.time_ms;
      if (r.converged) a.num_converged++;
    }
  }

  // Summary.
  printf("==========================================\n");
  printf("SUMMARY (averaged over %d seeds, n=%d, m=%d, eps=%.0e)\n\n",
         num_seeds, n, m, eps);
  printf("%-22s | %6s %6s | %5s | %6s\n",
         "Policy", "fac", "sol", "conv", "ms");
  printf("%s\n", std::string(60, '-').c_str());

  for (auto& [name, a] : agg) {
    printf("%-22s | %6.1f %6.1f | %3d/%d | %6.1f\n",
           a.name, (double)a.total_fac / num_seeds,
           (double)a.total_sol / num_seeds,
           a.num_converged, num_seeds,
           a.total_ms / num_seeds);
  }
  printf("\n");

  return 0;
}
