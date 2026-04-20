// QP benchmark: runs geodesic IPM algorithms on QPS (Maros-Meszaros) instances.
//
// Usage:
//   ./benchmark_qp <file.qps>
//   ./benchmark_qp <directory>         (runs all .QPS files, sorted by size)
//   ./benchmark_qp <dir> --limit N     (stop after N instances)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
#include "conex/common/qps_reader.h"
#include "conex/common/solver.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {
namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::high_resolution_clock;

struct AlgoResult {
  const char* name;
  int iterations;
  int factorizations;
  double mu;
  double primal_cost;
  double dual_residual;
  double complementarity;
  double time_ms;
  bool converged;
};

AlgoResult RunAlgo(const char* name, KKTSolverBase& kkt,
                   const SolverRHS& cost_rhs,
                   const Model& problem,
                   const Solver& solver,
                   auto solve_fn) {
  RowSpace W = kkt.MakeRowSpace();
  setOnes(W);
  auto t0 = Clock::now();
  auto result = solve_fn(kkt, cost_rhs, W);
  auto t1 = Clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  double primal_cost = 0;
  if (result.x.size() > 0) {
    Eigen::VectorXd x = solver.ExpandSolution(result.x);
    // c'x
    if (problem.has_linear_cost()) {
      int nc = std::min((int)problem.linear_cost().size(), (int)x.size());
      primal_cost += problem.linear_cost().head(nc).dot(x.head(nc));
    }
    // (1/2) x'Qx
    for (const auto& c : problem.constraints()) {
      if (auto* qc = std::get_if<Model::QuadraticCostData>(&c)) {
        const auto& Q = qc->Q_sparse;
        const auto& vars = qc->vars;
        for (int k = 0; k < Q.outerSize(); ++k) {
          for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k); it; ++it) {
            int i = vars[it.row()], j = vars[it.col()];
            if (i < (int)x.size() && j < (int)x.size())
              primal_cost += 0.5 * it.value() * x(i) * x(j);
          }
        }
      }
    }
  }

  return {name, result.iterations, result.total_factorizations,
          result.mu, primal_cost,
          result.optimality.dual_residual,
          result.optimality.complementarity,
          ms, result.mu < 1e-6};
}

void RunBenchmark(const Model& problem, const QPSInfo& info,
                  const std::string& filename) {
  printf("=== %s ===\n", filename.c_str());
  printf("  vars=%d, eq=%d, ineq=%d, quad=%d, bounds=%d, constraints=%d",
         info.num_variables, info.num_equality_rows,
         info.num_inequality_rows, info.num_quadratic_entries,
         info.num_bounded_vars, problem.num_constraints());
  if (info.objective_constant != 0)
    printf(", c0=%.6e", info.objective_constant);
  printf("\n");

  auto t0 = Clock::now();
  auto solver = Solver::Build(problem);
  auto t1 = Clock::now();
  double build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  auto* kkt = solver.kkt();
  int nv = kkt->number_of_variables();
  printf("  KKT vars=%d, build=%.1f ms\n", nv, build_ms);

  auto cost_rhs = solver.MakeCostRHS();

  const int max_iters = 500;
  const double tol = 1e-8;

  // --- ThetaContinuation ---
  auto r1 = RunAlgo("ThetaCont", *kkt, cost_rhs, problem, solver,
    [&](KKTSolverBase& k, const SolverRHS& c, RowSpace& W) {
      return SolveGeodesicThetaContinuation(k, c, W, max_iters, 1, tol, true);
    });

  // --- PhaseOne ---
  auto r2 = RunAlgo("PhaseOne", *kkt, cost_rhs, problem, solver,
    [&](KKTSolverBase& k, const SolverRHS& c, RowSpace& W) {
      return SolveGeodesicPhaseOne(k, c, W, max_iters, 1, tol, true);
    });

  // --- Phase1+Hybrid ---
  auto r3 = RunAlgo("Ph1+Hybrid", *kkt, cost_rhs, problem, solver,
    [&](KKTSolverBase& k, const SolverRHS& c, RowSpace& W) {
      auto p1 = SolveGeodesicPhaseOne(k, c, W, max_iters, 1, tol, true,
                                       /*phase1_only=*/true);
      double k_init = (p1.mu > 0) ? 1.0 / std::sqrt(p1.mu) : -1;
      auto result = SolveGeodesicHybrid(k, c, W, max_iters, tol, true,
                                         k_init, p1.tau);
      result.total_factorizations += p1.total_factorizations;
      result.total_solves += p1.total_solves;
      return result;
    });

  // --- Summary table ---
  printf("  %-12s %5s %5s %10s %14s %10s %10s %8s %s\n",
         "Algorithm", "iters", "fac", "mu", "cost", "dual_res",
         "compl", "ms", "ok");
  printf("  %s\n", std::string(90, '-').c_str());
  double c0 = info.objective_constant;
  for (const auto* r : {&r1, &r2, &r3}) {
    printf("  %-12s %5d %5d %10.2e %14.6e %10.2e %10.2e %8.1f %s\n",
           r->name, r->iterations, r->factorizations,
           r->mu, r->primal_cost + c0, r->dual_residual,
           r->complementarity, r->time_ms,
           r->converged ? "yes" : "NO");
  }

  // Print final objective values prominently.
  printf("\n  Objective = c0 + c'x + (1/2)x'Qx");
  if (c0 != 0) printf("   (c0 = %.6e)", c0);
  printf("\n");
  for (const auto* r : {&r1, &r2, &r3}) {
    printf("    %-12s  obj = %14.6e\n", r->name, r->primal_cost + c0);
  }
  printf("\n");
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  if (argc < 2) {
    printf("Usage:\n");
    printf("  %s <file.qps>               Run a single QPS file\n", argv[0]);
    printf("  %s <directory> [--limit N]   Run all .QPS files, sorted by size\n", argv[0]);
    return 1;
  }

  std::string path = argv[1];
  int limit = 0;
  for (int i = 2; i < argc; ++i) {
    if (std::string(argv[i]) == "--limit" && i + 1 < argc)
      limit = std::atoi(argv[++i]);
  }

  namespace fs = std::filesystem;

  if (fs::is_directory(path)) {
    // Collect all .QPS files.
    std::vector<std::pair<uintmax_t, std::string>> files;
    for (const auto& entry : fs::directory_iterator(path)) {
      auto p = entry.path();
      auto ext = p.extension().string();
      // Case-insensitive .QPS check.
      if (ext == ".QPS" || ext == ".qps") {
        files.push_back({entry.file_size(), p.string()});
      }
    }
    std::sort(files.begin(), files.end());
    printf("Found %d QPS files (sorted by size)\n\n", (int)files.size());

    int count = 0;
    for (const auto& [sz, filepath] : files) {
      if (limit > 0 && count >= limit) break;
      try {
        auto [problem, info] = conex::ReadQPS(filepath);
        conex::RunBenchmark(problem, info,
                            fs::path(filepath).filename().string());
        count++;
      } catch (const std::exception& e) {
        printf("SKIP %s: %s\n\n",
               fs::path(filepath).filename().c_str(), e.what());
      }
    }
    printf("Completed %d / %d instances.\n", count, (int)files.size());
  } else {
    try {
      auto [problem, info] = conex::ReadQPS(path);
      conex::RunBenchmark(problem, info,
                          fs::path(path).filename().string());
    } catch (const std::exception& e) {
      printf("Error: %s\n", e.what());
      return 1;
    }
  }
}
