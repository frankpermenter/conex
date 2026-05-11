// Evaluation of the extended embedding on random LPs.
//
// Generates random LP instances, builds the extended embedding, and
// runs multiple algorithm strategies + hybrid switching policies.
// Reports convergence, factorizations, and solution quality.
//
// Usage:
//   ./eval_embedding [--n 10] [--m 20] [--seeds 5] [--eps 1e-6] [--verbose]

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/conex.h"
#include "conex/common/extended_embedding.h"
#include "conex/common/solver.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {
namespace {

// Switching policies (same as tune_hybrid.cc).
HybridSwitchPolicy MakeCenterOnLargeDInf(double threshold) {
  return [threshold](double gap, double d_inf, int) {
    return gap < 0 || d_inf > threshold;
  };
}
HybridSwitchPolicy MakeAlwaysCenter() {
  return [](double, double, int) { return true; };
}
HybridSwitchPolicy MakeAlwaysShrink() {
  return [](double, double, int) { return false; };
}
HybridSwitchPolicy MakeMaxRUpdates(int max_r) {
  return [max_r](double gap, double, int r_updates) {
    return gap < 0 || r_updates >= max_r;
  };
}

using Eigen::MatrixXd;
using Eigen::VectorXd;

// =====================================================================
// Random LP generation.
// =====================================================================

struct RandomLP {
  Eigen::SparseMatrix<double> A;
  VectorXd b;
  VectorXd c;
  int n, m;
  // Known optimal value (if available).
  double opt_val = NAN;
};

// Generate a random LP:  min c'x  s.t. Ax >= b, x >= 0.
// The embedding uses Ax - btau = rp*theta form internally.
RandomLP MakeRandomLP(int n, int m, int seed) {
  srand(seed);
  RandomLP lp;
  lp.n = n;
  lp.m = m;

  // Random A with positive entries (well-conditioned).
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);

  // Random feasible x* > 0, then b = A x* (so x* is feasible).
  VectorXd x_star = VectorXd::Random(n).cwiseAbs() + VectorXd::Ones(n);
  lp.b = A_dense * x_star;

  // Cost: random positive c.
  lp.c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

  // Build sparse A.
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  lp.A.resize(m, n);
  lp.A.setFromTriplets(trips.begin(), trips.end());

  return lp;
}

// LP with known optimal: x* = e, optimal value = c'e.
RandomLP MakeFeasibleLP(int n, int m, int seed) {
  srand(seed);
  RandomLP lp;
  lp.n = n;
  lp.m = m;

  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);
  lp.b = A_dense * VectorXd::Ones(n);  // x*=e is feasible.
  lp.c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);
  lp.opt_val = lp.c.sum();  // Upper bound: c'e.

  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  lp.A.resize(m, n);
  lp.A.setFromTriplets(trips.begin(), trips.end());

  return lp;
}

// =====================================================================
// Solution extraction from embedding result.
//
// The embedding solves:
//   Primal: min c'x  s.t. Ax = b, x >= 0
//   Dual:   max b'y  s.t. s = c - A'y, s >= 0
// =====================================================================

struct LPSolution {
  // Embedding variables.
  double theta;
  double tau;
  double kappa;

  // Status.
  enum Status { OPTIMAL, PRIMAL_INFEASIBLE, DUAL_INFEASIBLE, INDETERMINATE };
  Status status = INDETERMINATE;

  // LP residuals (valid when status == OPTIMAL, i.e. tau > 0).
  double primal_obj;      // c' x_lp
  double dual_obj;        // b' y_lp
  double duality_gap;     // c' x_lp - b' y_lp
  double primal_infeas;   // ||A x_lp - b||
  double dual_infeas;     // ||c - A' y_lp - s_lp||
  double primal_predicted;// ||rp|| * |theta| / tau
  double dual_predicted;  // ||rd|| * |theta| / tau
  double complementarity; // x_lp' s_lp
  double min_x;           // min(x_lp)
  double min_s;           // min(s_lp)

  // Ray residuals (valid when status == PRIMAL_INFEASIBLE or DUAL_INFEASIBLE).
  double ray_residual;    // ||Ax|| or ||A'y + s||
  double ray_objective;   // c'x or b'y
  double ray_min_cone;    // min(x) or min(s)
};

LPSolution ExtractLPSolution(const SolveResult& result,
                              const EmbeddingInfo& info,
                              const Eigen::SparseMatrix<double>& A,
                              const VectorXd& b,
                              const VectorXd& c) {
  const int n = info.n, m = info.m;
  const MatrixXd A_dense = MatrixXd(A);

  LPSolution sol{};
  sol.theta = result.x(info.theta_idx());
  sol.tau = result.x(info.tau_idx());
  sol.kappa = result.x(info.kappa_idx());

  VectorXd x_emb = result.x.segment(info.x_start(), n);
  VectorXd y_emb = result.x.segment(info.y_start(), m);
  VectorXd s_emb = result.x.segment(info.s_start(), n);

  const double tau_tol = 1e-8;
  const double kappa_tol = 1e-8;

  if (sol.tau > tau_tol) {
    // Feasible: recover LP solution by dividing by tau.
    sol.status = LPSolution::OPTIMAL;
    VectorXd x_lp = x_emb / sol.tau;
    VectorXd y_lp = y_emb / sol.tau;
    VectorXd s_lp = s_emb / sol.tau;

    sol.primal_obj = c.dot(x_lp);
    sol.dual_obj = b.dot(y_lp);
    sol.duality_gap = sol.primal_obj - sol.dual_obj;
    sol.primal_infeas = (A_dense * x_lp - b).norm();
    sol.dual_infeas = (c - A_dense.transpose() * y_lp - s_lp).norm();
    sol.primal_predicted = info.rp.norm() * std::abs(sol.theta) / sol.tau;
    sol.dual_predicted = info.rd.norm() * std::abs(sol.theta) / sol.tau;
    sol.complementarity = x_lp.dot(s_lp);
    sol.min_x = x_lp.minCoeff();
    sol.min_s = s_lp.minCoeff();
  } else if (sol.kappa > kappa_tol) {
    // tau ≈ 0, kappa > 0: check for improving rays.
    // Primal ray (unbounded): Ax ≈ 0, x >= 0, c'x < 0.
    double cx = c.dot(x_emb);
    double Ax_norm = (A_dense * x_emb).norm();
    double min_x = x_emb.minCoeff();

    // Dual ray (infeasible): A'y + s ≈ 0, s >= 0, b'y > 0.
    double by = b.dot(y_emb);
    double Ays_norm = (A_dense.transpose() * y_emb + s_emb).norm();
    double min_s = s_emb.minCoeff();

    if (by > 0 && Ays_norm < 1e-4 * by) {
      sol.status = LPSolution::PRIMAL_INFEASIBLE;
      sol.ray_residual = Ays_norm;
      sol.ray_objective = by;
      sol.ray_min_cone = min_s;
    } else if (cx < 0 && Ax_norm < 1e-4 * std::abs(cx)) {
      sol.status = LPSolution::DUAL_INFEASIBLE;
      sol.ray_residual = Ax_norm;
      sol.ray_objective = cx;
      sol.ray_min_cone = min_x;
    } else {
      sol.status = LPSolution::INDETERMINATE;
    }
  }
  return sol;
}

// =====================================================================
// Run one algorithm on one embedding.
// =====================================================================

struct EvalResult {
  const char* name;
  int factorizations;  // loop W-updates only (setup+recovery = +2)
  double mu;
  double gap;
  double d_inf;
  bool converged;
  LPSolution sol;
  // Embedding-level residuals from Solver::Solve.
  double emb_stationarity;   // ||c + Qx - A'λ - C'ν|| (embedding KKT)
  double emb_complementarity; // <s, λ> (embedding cones)
  double emb_eq_error;        // max ||Cx - d|| across equality constraints
  double emb_min_slack;
  double emb_min_dual;
};

template <typename Algorithm>
EvalResult RunAlgorithm(const char* name,
                        const Model& emb_model,
                        const CliqueTree& emb_tree,
                        const EmbeddingInfo& info,
                        const RandomLP& lp,
                        const Algorithm& algo,
                        bool use_dense = false,
                        bool use_lu = false) {
  printf("\n--- %s ---\n", name);
  Solver solver = [&]() {
    if (use_dense) return Solver::BuildDense(emb_model);
    SolverConfiguration cfg;
    if (use_lu) cfg.tree.use_lu_for_indefinite = true;
    return Solver::Build(emb_model, emb_tree, cfg);
  }();
  auto result = solver.Solve(algo);

  EvalResult out;
  out.name = name;
  out.factorizations = result.factorizations;
  out.mu = result.mu;
  out.gap = result.gap;
  out.d_inf = result.d_inf;
  out.converged = result.converged;
  out.sol = ExtractLPSolution(result, info, lp.A, lp.b, lp.c);
  out.emb_stationarity = result.duals.stationarity_gradient.norm();
  out.emb_complementarity = result.optimality.complementarity;
  out.emb_eq_error = 0;
  for (const auto& r : result.duals.eq_residual)
    out.emb_eq_error = std::max(out.emb_eq_error, r.norm());
  out.emb_min_slack = result.optimality.min_slack;
  out.emb_min_dual = result.optimality.min_dual;
  return out;
}

const char* StatusStr(LPSolution::Status s) {
  switch (s) {
    case LPSolution::OPTIMAL: return "opt";
    case LPSolution::PRIMAL_INFEASIBLE: return "p_inf";
    case LPSolution::DUAL_INFEASIBLE: return "d_inf";
    case LPSolution::INDETERMINATE: return "indet";
  }
  return "?";
}

void PrintEvalHeader() {
  printf("%-26s | %4s | %8s %8s %8s | %5s %10s %10s %10s %10s\n",
         "Algorithm", "fac", "theta", "tau", "kappa",
         "stat", "p_obj", "d_obj", "p_infeas", "d_infeas");
  printf("%s\n", std::string(120, '-').c_str());
}

void PrintEvalResult(const EvalResult& r) {
  auto& s = r.sol;
  if (s.status == LPSolution::OPTIMAL) {
    printf("%-26s | %4d | %8.2e %8.2e %8.2e | %5s %10.4f %10.4f %10.2e %10.2e\n",
           r.name, r.factorizations, s.theta, s.tau, s.kappa,
           StatusStr(s.status), s.primal_obj, s.dual_obj,
           s.primal_infeas, s.dual_infeas);
    printf("%-26s   Algorithm Residuals: stationarity=%8.2e  complementarity=%8.2e  eq_error=%8.2e\n",
           "", r.emb_stationarity, r.emb_complementarity, r.emb_eq_error);
  } else if (s.status == LPSolution::PRIMAL_INFEASIBLE ||
             s.status == LPSolution::DUAL_INFEASIBLE) {
    printf("%-26s | %4d | %8.2e %8.2e %8.2e | %5s ray_res=%8.2e ray_obj=%8.2e ray_cone=%8.2e\n",
           r.name, r.factorizations, s.theta, s.tau, s.kappa,
           StatusStr(s.status), s.ray_residual, s.ray_objective, s.ray_min_cone);
  } else {
    printf("%-26s | %4d | %8.2e %8.2e %8.2e | %5s\n",
           r.name, r.factorizations, s.theta, s.tau, s.kappa,
           StatusStr(s.status));
  }
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  int n = 10, m = 10, num_seeds = 5;
  double eps = 1e-6;
  bool verbose = false;
  bool use_dense = false;
  bool use_lu = false;
  bool print_tree = false;
  std::string algo_filter;  // empty = run all

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--eps" && i + 1 < argc) eps = std::atof(argv[++i]);
    else if (arg == "--n" && i + 1 < argc) n = std::atoi(argv[++i]);
    else if (arg == "--m" && i + 1 < argc) m = std::atoi(argv[++i]);
    else if (arg == "--seeds" && i + 1 < argc) num_seeds = std::atoi(argv[++i]);
    else if (arg == "--algo" && i + 1 < argc) algo_filter = argv[++i];
    else if (arg == "--verbose") verbose = true;
    else if (arg == "--dense") use_dense = true;
    else if (arg == "--lu") use_lu = true;
    else if (arg == "--tree") print_tree = true;
    else {
      printf("Usage: %s [--n 10] [--m 20] [--seeds 5] [--eps 1e-6] [--algo name] [--verbose] [--dense] [--lu]\n",
             argv[0]);
      return 1;
    }
  }

  const char* solver_label = use_dense ? " (DENSE KKT)" : (use_lu ? " (LU)" : "");
  printf("Extended Embedding Evaluation%s\n", solver_label);
  printf("LP: n=%d vars, m=%d constraints, eps=%.0e, seeds=%d\n\n", n, m, eps, num_seeds);

  // Aggregate statistics.
  struct Agg {
    int total_fac = 0;
    int num_optimal = 0;
    double total_theta = 0;
    double total_pinfeas = 0;
    double total_dinfeas = 0;
    double total_compl = 0;
    double total_gap = 0;
    int count = 0;
  };
  std::map<std::string, Agg> agg;

  for (int seed = 1; seed <= num_seeds; ++seed) {
    auto lp = conex::MakeRandomLP(n, m, seed);
    auto [emb_model, info, emb_tree] = conex::BuildExtendedEmbedding(lp.A, lp.b, lp.c);

    printf("=== seed=%d  (n=%d, m=%d, emb_vars=%d, alpha=%.4f) ===\n",
           seed, n, m, info.total_vars(), info.alpha);

    if (print_tree && seed == 1) {
      // AMD tree.
      auto tmp = conex::Solver::Build(emb_model);
      auto* ts = dynamic_cast<conex::SymmetricLinearSystemTreeSolver*>(tmp.kkt());
      if (ts) {
        auto amd_tree = ts->GetCliqueTree();
        const auto& pinv = ts->perm_inv();
        int ns = (int)amd_tree.supernodes.size();
        printf("  AMD tree: %d cliques, %d vars\n", ns, ts->number_of_variables());
        for (int k = 0; k < ns; ++k) {
          printf("    clique %d: sn={", k);
          for (int v : amd_tree.supernodes[k]) printf("%d,", pinv(v));
          printf("} sep={");
          for (int v : amd_tree.separators[k]) printf("%d,", pinv(v));
          printf("} parent=%d\n", amd_tree.node_to_parent[k]);
        }
        // Print variable legend.
        printf("  vars: x=[%d,%d) y=[%d,%d) s=[%d,%d) tau=%d kappa=%d theta=%d\n",
               info.x_start(), info.x_start()+n,
               info.y_start(), info.y_start()+m,
               info.s_start(), info.s_start()+n,
               info.tau_idx(), info.kappa_idx(), info.theta_idx());
        int N = info.total_vars();
        printf("  duals: nu1=[%d,%d) nu2=[%d,%d) nu3=%d nu4=%d\n",
               N, N+m, N+m, N+m+n, N+m+n, N+m+n+1);
      }
      // Custom tree.
      printf("  Custom tree: %d cliques, RIP=%s\n",
             (int)emb_tree.supernodes.size(),
             emb_tree.CheckRunningIntersectionProperty() ? "ok" : "FAIL");
      for (int k = 0; k < (int)emb_tree.supernodes.size(); k++) {
        printf("    clique %d: sn=%d sep=%d parent=%d\n",
               k, (int)emb_tree.supernodes[k].size(),
               (int)emb_tree.separators[k].size(),
               emb_tree.node_to_parent[k]);
      }
    }

    conex::PrintEvalHeader();

    auto record = [&](const conex::EvalResult& r) {
      conex::PrintEvalResult(r);
      auto& a = agg[r.name];
      a.total_fac += r.factorizations;
      a.num_optimal += (r.sol.status == conex::LPSolution::OPTIMAL);
      a.total_theta += std::abs(r.sol.theta);
      if (r.sol.status == conex::LPSolution::OPTIMAL) {
        a.total_pinfeas += r.sol.primal_infeas;
        a.total_dinfeas += r.sol.dual_infeas;
        a.total_compl += r.sol.complementarity;
        a.total_gap += std::abs(r.sol.duality_gap);
      }
      a.count++;
    };

    auto should_run = [&](const char* name) {
      return algo_filter.empty() ||
             std::string(name).find(algo_filter) != std::string::npos;
    };

    // 1. ThetaContinuation.
    if (should_run("ThetaContinuation")) {
      conex::ThetaContinuation algo;
      algo.tolerance = eps;
      algo.verbose = verbose;
      record(conex::RunAlgorithm("ThetaContinuation", emb_model, emb_tree, info, lp, algo, use_dense, use_lu));
    }

    // 2. HybridOnly with different switching policies.
    auto run_hybrid = [&](const char* name, conex::HybridSwitchPolicy policy) {
      if (!should_run(name)) return;
      conex::HybridOnly algo;
      algo.tolerance = eps;
      algo.verbose = verbose;
      algo.policy = policy;
      record(conex::RunAlgorithm(name, emb_model, emb_tree, info, lp, algo, use_dense, use_lu));
    };

    run_hybrid("Hybrid(default)",       conex::DefaultHybridPolicy);
    run_hybrid("Hybrid(DR only)",       conex::MakeAlwaysShrink());

    // 3. ThetaContinuationR.
    if (should_run("ThetaContR")) {
      conex::ThetaContinuationR algo;
      algo.tolerance = eps;
      algo.verbose = verbose;
      record(conex::RunAlgorithm("ThetaContR", emb_model, emb_tree, info, lp, algo, use_dense, use_lu));
    }



    // 4. GeodesicLP.
    if (should_run("GeodesicLP")) {
      conex::GeodesicLP algo;
      algo.tolerance = eps;
      algo.verbose = verbose;
      record(conex::RunAlgorithm("GeodesicLP", emb_model, emb_tree, info, lp, algo, use_dense, use_lu));
    }

    printf("\n");
  }

  // Summary.
  printf("==========================================================================\n");
  printf("SUMMARY (n=%d, m=%d, %d seeds, eps=%.0e)\n\n", n, m, num_seeds, eps);
  printf("%-26s | %6s | %5s | %10s %10s %10s %10s %10s\n",
         "Algorithm", "fac", "opt", "avg|theta|",
         "p_infeas", "d_infeas", "compl", "gap");
  printf("%s\n", std::string(110, '-').c_str());

  const char* order[] = {
    "ThetaContinuation",
    "Hybrid(default)",
    "Hybrid(DR only)",
    "GeodesicLP",
  };
  for (auto name : order) {
    auto it = agg.find(name);
    if (it == agg.end()) continue;
    auto& a = it->second;
    int nopt = std::max(a.num_optimal, 1);  // avoid /0
    printf("%-26s | %6.1f | %3d/%d | %10.2e %10.2e %10.2e %10.2e %10.2e\n",
           name,
           (double)a.total_fac / a.count,
           a.num_optimal, a.count,
           a.total_theta / a.count,
           a.total_pinfeas / nopt,
           a.total_dinfeas / nopt,
           a.total_compl / nopt,
           a.total_gap / nopt);
  }
  printf("\n");

  return 0;
}
