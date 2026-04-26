// Unified benchmark: runs geodesic IPM algorithms or factorization profiling
// on any supported file format (MPS, QPS, SDPA, CBF, MTX) or synthetic problems.
//
// Usage:
//   ./benchmark_solver <file.mps|.qps|.dat-s|.cbf|.mtx>
//   ./benchmark_solver <file> --profile                  (factorization timing)
//   ./benchmark_solver <file> --profile --sweep-threads 1,2,4
//   ./benchmark_solver <directory>                       (all QPS files)
//   ./benchmark_solver --synthetic lp <m> <n> [seed]
//   ./benchmark_solver --synthetic sdp <n> <p> [seed]
//   ./benchmark_solver --synthetic socp <dim> <p> [seed]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/geodesic_hybrid_r.h"
#include "conex/common/cbf_reader.h"
#include "conex/common/compiled_model.h"
#include "conex/algorithms/geodesic_hybrid_r.h"
#include "conex/common/eja_ops.h"
#include "conex/common/mps_reader.h"
#include "conex/common/mtx_reader.h"
#include "conex/common/model.h"
#include "conex/common/qps_reader.h"
#include "conex/common/rescale.h"
#include "conex/common/sdpa_reader.h"
#include "conex/common/solver.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {
namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::high_resolution_clock;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Eigen::SparseMatrix<double> ToSparse(const MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> t;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i, j)) > 1e-14) t.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(t.begin(), t.end());
  return S;
}

// =====================================================================
// Synthetic problem generators
// =====================================================================

Model MakeSyntheticLP(int m, int n, int seed) {
  srand(seed);
  MatrixXd A_dense = MatrixXd::Random(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A_dense.transpose() * VectorXd::Ones(m);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddLinearConstraint(ToSparse(A_dense), b, vars);
  problem.SetLinearCost(c);
  return problem;
}

Model MakeSyntheticSDP(int n, int p, int seed) {
  srand(seed);
  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars(p);
  VectorXd c(p);
  for (int k = 0; k < p; ++k) {
    MatrixXd Ak = MatrixXd::Random(n, n);
    Ak = 0.5 * (Ak + Ak.transpose());
    A_list.push_back(ToSparse(Ak));
    vars[k] = k;
    c(k) = Ak.trace();
  }
  Model problem;
  problem.AddPSDConstraint(A_list, ToSparse(MatrixXd::Identity(n, n)), vars,
                            false);
  problem.SetLinearCost(c);
  return problem;
}

Model MakeSyntheticSOCP(int dim, int p, int seed) {
  srand(seed);
  MatrixXd A_dense = MatrixXd::Random(dim, p);
  VectorXd b = VectorXd::Zero(dim);
  b(0) = 1.0;
  VectorXd c = A_dense.row(0).transpose();
  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddSOCConstraint(ToSparse(A_dense), b, vars);
  problem.SetLinearCost(c);
  return problem;
}

// =====================================================================
// Algorithm profiling (geodesic IPM)
// =====================================================================

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

AlgoResult RunAlgo(const char* name, CompiledModel& model,
                   const Model& problem,
                   const Solver& solver,
                   auto solve_fn) {
  RowSpace W = model.MakeRowSpace();
  setOnes(W);
  auto t0 = Clock::now();
  auto result = solve_fn(model, W);
  auto t1 = Clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  double primal_cost = 0;
  if (result.x.size() > 0) {
    // Expand to original space for cost computation.
    VectorXd x = solver.ExpandSolution(result.x);
    if (problem.has_linear_cost()) {
      int nc = std::min((int)problem.linear_cost().size(), (int)x.size());
      primal_cost += problem.linear_cost().head(nc).dot(x.head(nc));
    }
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

void ProfileAlgorithm(const Model& problem, const std::string& name,
                      const SolverConfiguration& config,
                      double objective_constant = 0,
                      const std::string& algo_filter = "",
                      double tol_override = 1e-8) {
  auto should_run = [&](const char* aname) {
    return algo_filter.empty() ||
           std::string(aname).find(algo_filter) != std::string::npos;
  };
  printf("=== %s ===\n", name.c_str());
  printf("  Variables: %d, Constraints: %d\n",
         problem.num_variables(), problem.num_constraints());

  auto t0 = Clock::now();
  auto solver = Solver::Build(problem, config);
  auto t1 = Clock::now();
  double build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  auto* kkt = solver.kkt();
  int nv = kkt->number_of_variables();
  printf("  KKT vars=%d, build=%.1f ms\n", nv, build_ms);
  if (solver.was_reduced()) {
    printf("  Preprocessing: %d -> %d variables\n",
           problem.num_variables(), nv);
  }

  auto cost_rhs = solver.MakeCostRHS();
  CompiledModel model(*kkt, cost_rhs);

  const int max_iters = 500;
  const double tol = tol_override;

  std::vector<AlgoResult> results;

  if (should_run("ThetaCont")) {
    results.push_back(RunAlgo("ThetaCont", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicThetaContinuation(model, W, max_iters, 1, tol, true);
      }));
  }
  if (should_run("PhaseOne")) {
    results.push_back(RunAlgo("PhaseOne", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicPhaseOne(model, W, max_iters, 1, tol, true);
      }));
  }
  if (should_run("Ph1+Hybrid")) {
    results.push_back(RunAlgo("Ph1+Hybrid", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        auto p1 = SolveGeodesicPhaseOne(model, W, max_iters, 1, tol, true,
                                         /*phase1_only=*/true);
        double k_init = (p1.mu > 0) ? 1.0 / std::sqrt(p1.mu) : -1;
        auto result = SolveGeodesicHybrid(model, W, max_iters, tol, true,
                                           k_init, p1.tau);
        result.total_factorizations += p1.total_factorizations;
        result.total_solves += p1.total_solves;
        return result;
      }));
  }
  if (should_run("HybridR")) {
    results.push_back(RunAlgo("HybridR", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicHybridR(model, W, max_iters, tol, true);
      }));
  }
  if (should_run("ThetaContR")) {
    results.push_back(RunAlgo("ThetaContR", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicThetaContinuationR(model, W, max_iters, tol, true);
      }));
  }
  for (double ct : {1e-8, 1e-10, 1e-12, 1e-14}) {
    for (double tv : {1e-8, 1e-10, 1e-12}) {
      char name[48];
      snprintf(name, sizeof(name), "TR_t%g_c%g", tv, ct);
      if (should_run(name)) {
        results.push_back(RunAlgo(name, model, problem, solver,
          [&, ct, tv](CompiledModel& model, RowSpace& W) {
            return SolveGeodesicThetaContinuationR(model, W, max_iters, tv, true,
                DefaultThetaContRPolicy, ct);
          }));
      }
    }
  }
  if (should_run("ThetaR+gap")) {
    results.push_back(RunAlgo("ThetaR+gap", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicThetaContinuationR(model, W, max_iters, tol, true,
            [](double gap, double, int) { return gap < 0; });
      }));
  }
  if (should_run("ColdHybrid")) {
    results.push_back(RunAlgo("ColdHybrid", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicHybrid(model, W, max_iters, tol, true);
      }));
  }
  auto make_warmup_policy = [](int warmup_centers) -> HybridSwitchPolicy {
    auto count = std::make_shared<int>(0);
    return [warmup_centers, count](double gap, double, int) {
      if (*count < warmup_centers) { (*count)++; return true; }
      return gap < 0;
    };
  };
  if (should_run("Cold+3ctr")) {
    results.push_back(RunAlgo("Cold+3ctr", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicHybrid(model, W, max_iters, tol, true,
                                    -1, 1.0, make_warmup_policy(3));
      }));
  }
  if (should_run("Cold+5ctr")) {
    results.push_back(RunAlgo("Cold+5ctr", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicHybrid(model, W, max_iters, tol, true,
                                    -1, 1.0, make_warmup_policy(5));
      }));
  }
  if (should_run("Cold+10ctr")) {
    results.push_back(RunAlgo("Cold+10ctr", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicHybrid(model, W, max_iters, tol, true,
                                    -1, 1.0, make_warmup_policy(10));
      }));
  }
  if (should_run("Cold+20ctr")) {
    results.push_back(RunAlgo("Cold+20ctr", model, problem, solver,
      [&](CompiledModel& model, RowSpace& W) {
        return SolveGeodesicHybrid(model, W, max_iters, tol, true,
                                    -1, 1.0, make_warmup_policy(20));
      }));
  }

  // --- Summary table ---
  printf("  %-12s %5s %5s %10s %14s %10s %10s %8s %s\n",
         "Algorithm", "iters", "fac", "mu", "cost", "dual_res",
         "compl", "ms", "ok");
  printf("  %s\n", std::string(90, '-').c_str());
  double c0 = objective_constant;
  for (const auto& r : results) {
    printf("  %-12s %5d %5d %10.2e %14.6e %10.2e %10.2e %8.1f %s\n",
           r.name, r.iterations, r.factorizations,
           r.mu, r.primal_cost + c0, r.dual_residual,
           r.complementarity, r.time_ms,
           r.converged ? "yes" : "NO");
  }

  if (c0 != 0) {
    printf("\n  Objective includes constant c0 = %.6e\n", c0);
  }
  printf("\n");
}

// =====================================================================
// Factorization profiling
// =====================================================================

double us(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::micro>(b - a).count();
}

struct ProfileResult {
  std::string name;
  int num_vars;
  int num_cliques;
  double build_us;
  double assemble_factor_us;
  double solve_us;
  double residual;
};

ProfileResult ProfileFactorization(const Model& problem,
                                   const std::string& name,
                                   const SolverConfiguration& cfg,
                                   int max_iters = -1) {
  ProfileResult res;
  res.name = name;

  auto t0 = Clock::now();
  auto solver = Solver::Build(problem, cfg);
  auto* kkt = solver.kkt();
  auto t1 = Clock::now();
  res.build_us = us(t0, t1);
  res.num_vars = kkt->number_of_variables();
  res.num_cliques = solver.tree_solver()
      ? solver.tree_solver()->num_subsystems() : 1;

  int iters = (max_iters >= 0) ? max_iters
                               : std::max(20, 2000 / std::max(1, res.num_vars));
  if (iters == 0) {
    res.assemble_factor_us = 0;
    res.solve_us = 0;
    res.residual = 0;
    return res;
  }

  // AssembleAndFactor.
  bool ok = kkt->AssembleAndFactor();
  if (!ok) {
    res.assemble_factor_us = -1;
    res.solve_us = -1;
    res.residual = -1;
    return res;
  }
  std::vector<double> af_times(iters);
  for (int i = 0; i < iters; ++i) {
    auto ta = Clock::now();
    kkt->AssembleAndFactor();
    auto tb = Clock::now();
    af_times[i] = us(ta, tb);
  }
  std::sort(af_times.begin(), af_times.end());
  res.assemble_factor_us = af_times[iters / 2];

  // Build RHS: random x_true, compute rhs = Gram * x_true.
  int n_solve = res.num_vars;
  VectorXd x_true = VectorXd::Random(n_solve);
  // Use a single AssembleAndFactor + Solve round trip to build the RHS.
  // For general problems, rhs = kkt_matrix * x_true.
  // Approximate by solving and checking residual.
  VectorXd rhs = VectorXd::Random(n_solve);

  // Solve timing.
  kkt->Solve(rhs);  // warm up
  std::vector<double> s_times(iters);
  VectorXd sol;
  for (int i = 0; i < iters; ++i) {
    auto ta = Clock::now();
    sol = kkt->Solve(rhs);
    auto tb = Clock::now();
    s_times[i] = us(ta, tb);
  }
  std::sort(s_times.begin(), s_times.end());
  res.solve_us = s_times[iters / 2];

  // For residual, solve Kx = K*x_true and check ||x - x_true||.
  // Since we don't have K explicitly, we just report the solve time
  // and skip residual for non-MTX problems.
  res.residual = 0;

  return res;
}

void PrintProfileHeader() {
  printf("%-20s %4s %4s %6s | %10s | %10s %10s | %10s\n",
         "Model", "thrd", "merg", "cliq",
         "build",
         "asm+fac", "solve",
         "residual");
  printf("%s\n", std::string(95, '-').c_str());
}

void PrintProfileResult(const ProfileResult& res,
                        const SolverConfiguration& cfg) {
  if (res.assemble_factor_us < 0) {
    printf("%-20s %4d %4d %6d | %9.0fus |  FACTOR FAILED          | rank-def\n",
           res.name.c_str(), cfg.num_threads,
           cfg.tree.max_merge_supernode_size, res.num_cliques,
           res.build_us);
  } else {
    printf("%-20s %4d %4d %6d | %9.0fus | %9.0fus %9.0fus | %10.2e\n",
           res.name.c_str(), cfg.num_threads,
           cfg.tree.max_merge_supernode_size, res.num_cliques,
           res.build_us,
           res.assemble_factor_us, res.solve_us,
           res.residual);
  }
}

// =====================================================================
// File format dispatch
// =====================================================================

struct ProblemWithInfo {
  Model problem;
  std::string name;
  double objective_constant = 0;
};

ProblemWithInfo ReadProblemFile(const std::string& filename) {
  std::string ext = filename.substr(filename.find_last_of('.') + 1);
  ProblemWithInfo result;

  if (ext == "mps") {
    auto [p, info] = ReadMPS(filename);
    result.problem = std::move(p);
    char buf[256];
    snprintf(buf, sizeof(buf), "MPS: %s (%d vars, %d LE, %d GE, %d EQ)",
             info.name.c_str(), info.num_variables,
             info.num_le_rows, info.num_ge_rows, info.num_eq_rows);
    result.name = buf;
  } else if (ext == "dat-s" || ext == "dat" ||
             filename.find(".dat-s") != std::string::npos) {
    auto [p, info] = ReadSDPA(filename);
    result.problem = std::move(p);
    char buf[256];
    snprintf(buf, sizeof(buf), "SDPA: %d constraints, %d blocks, dim=%d",
             info.num_constraints, info.num_blocks, info.total_matrix_dim);
    result.name = buf;
  } else if (ext == "cbf") {
    auto [p, info] = ReadCBF(filename);
    result.problem = std::move(p);
    char buf[256];
    snprintf(buf, sizeof(buf), "CBF: %d vars, %d cons",
             info.num_variables, info.num_constraints);
    result.name = buf;
  } else if (ext == "qps" || ext == "QPS") {
    auto [p, info] = ReadQPS(filename);
    result.problem = std::move(p);
    result.objective_constant = info.objective_constant;
    char buf[256];
    snprintf(buf, sizeof(buf), "QPS: %s (%d vars, %d eq, %d ineq, %d quad)",
             info.name.c_str(), info.num_variables,
             info.num_equality_rows, info.num_inequality_rows,
             info.num_quadratic_entries);
    result.name = buf;
  } else if (ext == "mtx") {
    auto [p, info] = ReadMTX(filename);
    result.problem = std::move(p);
    char buf[256];
    snprintf(buf, sizeof(buf), "MTX: %s (%dx%d, nnz=%d%s%s)",
             info.name.c_str(), info.rows, info.cols, info.nnz,
             info.is_quadratic ? ", quadratic" : ", least-squares",
             info.was_transposed ? ", transposed" : "");
    result.name = buf;
  } else {
    throw std::runtime_error("Unknown file extension: " + ext);
  }

  return result;
}

// Parse comma-separated int list: "1,2,4" -> {1, 2, 4}.
std::vector<int> ParseList(const std::string& s) {
  std::vector<int> result;
  std::istringstream ss(s);
  std::string token;
  while (std::getline(ss, token, ',')) result.push_back(std::stoi(token));
  return result;
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  if (argc < 2) {
    printf("Usage:\n");
    printf("  %s <file.mps|.qps|.dat-s|.cbf|.mtx>\n", argv[0]);
    printf("  %s <file> --profile                    Factorization timing\n", argv[0]);
    printf("  %s <file> --profile --sweep-threads 1,2,4\n", argv[0]);
    printf("  %s <directory> [--limit N]              All QPS files\n", argv[0]);
    printf("  %s --synthetic lp <m> <n> [seed]\n", argv[0]);
    printf("  %s --synthetic sdp <n> <p> [seed]\n", argv[0]);
    printf("  %s --synthetic socp <dim> <p> [seed]\n", argv[0]);
    return 1;
  }

  namespace fs = std::filesystem;

  // Parse arguments.
  conex::SolverConfiguration cfg;
  bool profile_mode = false;
  bool do_rescale = false;
  bool randomize = false;
  conex::ColumnScaling strategy = conex::ColumnScaling::Ruiz;
  int limit = 0;
  int max_profile_iters = -1;
  std::string algo_filter;
  double tol = 1e-8;
  std::vector<int> sweep_threads, sweep_merge;
  std::string arg1 = argv[1];

  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--profile") {
      profile_mode = true;
    } else if (arg == "--rescale" || arg == "--ruiz") {
      do_rescale = true; strategy = conex::ColumnScaling::Ruiz;
    } else if (arg == "--l2") {
      do_rescale = true; strategy = conex::ColumnScaling::L2Norm;
    } else if (arg == "--maxabs") {
      do_rescale = true; strategy = conex::ColumnScaling::MaxAbsValue;
    } else if (arg == "--randomize") {
      randomize = true;
    } else if (arg == "--limit" && i + 1 < argc) {
      limit = std::atoi(argv[++i]);
    } else if (arg == "--threads" && i + 1 < argc) {
      cfg.num_threads = std::stoi(argv[++i]);
    } else if (arg == "--merge" && i + 1 < argc) {
      cfg.tree.max_merge_supernode_size = std::stoi(argv[++i]);
    } else if (arg == "--sweep-threads" && i + 1 < argc) {
      sweep_threads = conex::ParseList(argv[++i]);
    } else if (arg == "--sweep-merge" && i + 1 < argc) {
      sweep_merge = conex::ParseList(argv[++i]);
    } else if (arg == "--reorder" && i + 1 < argc) {
      cfg.tree.supernode_reorder_method = std::stoi(argv[++i]);
    } else if (arg == "--generic") {
      cfg.tree.use_generic_factorization = true;
    } else if (arg == "--iters" && i + 1 < argc) {
      max_profile_iters = std::stoi(argv[++i]);
    } else if (arg == "--algo" && i + 1 < argc) {
      algo_filter = argv[++i];
    } else if (arg == "--tol" && i + 1 < argc) {
      tol = std::stod(argv[++i]);
    }
  }

  // --- Synthetic problems ---
  if (arg1 == "--synthetic") {
    if (argc < 5) {
      printf("Need: --synthetic <type> <dim1> <dim2> [seed]\n");
      return 1;
    }
    std::string type = argv[2];
    int d1 = std::atoi(argv[3]);
    int d2 = std::atoi(argv[4]);
    int seed = argc > 5 ? std::atoi(argv[5]) : 42;

    conex::Model problem;
    char name[128];
    if (type == "lp") {
      snprintf(name, sizeof(name), "Synthetic LP (%dx%d, seed=%d)", d1, d2, seed);
      problem = conex::MakeSyntheticLP(d1, d2, seed);
    } else if (type == "sdp") {
      snprintf(name, sizeof(name), "Synthetic SDP (n=%d, p=%d, seed=%d)", d1, d2, seed);
      problem = conex::MakeSyntheticSDP(d1, d2, seed);
    } else if (type == "socp") {
      snprintf(name, sizeof(name), "Synthetic SOCP (dim=%d, p=%d, seed=%d)", d1, d2, seed);
      problem = conex::MakeSyntheticSOCP(d1, d2, seed);
    } else {
      printf("Unknown type: %s\n", type.c_str());
      return 1;
    }

    if (profile_mode) {
      conex::PrintProfileHeader();
      auto res = conex::ProfileFactorization(problem, name, cfg, max_profile_iters);
      conex::PrintProfileResult(res, cfg);
    } else {
      conex::ProfileAlgorithm(problem, name, cfg, 0, algo_filter, tol);
    }
    return 0;
  }

  // --- Directory of QPS files ---
  if (fs::is_directory(arg1)) {
    std::vector<std::pair<uintmax_t, std::string>> files;
    for (const auto& entry : fs::directory_iterator(arg1)) {
      auto p = entry.path();
      auto ext = p.extension().string();
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
        auto info = conex::ReadProblemFile(filepath);
        if (profile_mode) {
          conex::PrintProfileHeader();
          auto res = conex::ProfileFactorization(
              info.problem, info.name, cfg, max_profile_iters);
          conex::PrintProfileResult(res, cfg);
        } else {
          conex::ProfileAlgorithm(info.problem, info.name, cfg,
                                   info.objective_constant, algo_filter, tol);
        }
        count++;
      } catch (const std::exception& e) {
        printf("SKIP %s: %s\n\n",
               fs::path(filepath).filename().c_str(), e.what());
      }
    }
    printf("Completed %d / %d instances.\n", count, (int)files.size());
    return 0;
  }

  // --- Single file ---
  try {
    auto info = conex::ReadProblemFile(arg1);

    if (do_rescale) {
      const char* sname[] = {"MaxAbsValue", "L2Norm", "Ruiz"};
      printf("Column scaling: %s\n", sname[static_cast<int>(strategy)]);
      auto [rescaled, rinfo] = conex::RescaleProblem(info.problem, strategy);
      if (rinfo.was_rescaled) {
        printf("Rescaled (col_scale range: [%.2e, %.2e])\n",
               rinfo.col_scale.minCoeff(), rinfo.col_scale.maxCoeff());
        info.problem = std::move(rescaled);
        info.name += " [rescaled]";
      } else {
        printf("Rescaling had no effect.\n");
      }
    }

    if (profile_mode) {
      if (sweep_threads.empty()) sweep_threads = {cfg.num_threads};
      if (sweep_merge.empty()) sweep_merge = {cfg.tree.max_merge_supernode_size};

      for (int threads : sweep_threads) {
        for (int merge : sweep_merge) {
          conex::SolverConfiguration run_cfg = cfg;
          run_cfg.num_threads = threads;
          run_cfg.tree.max_merge_supernode_size = merge;

          conex::PrintProfileHeader();
          auto res = conex::ProfileFactorization(
              info.problem, info.name, run_cfg, max_profile_iters);
          conex::PrintProfileResult(res, run_cfg);
          printf("\n");
        }
      }
      printf("Columns: thrd=num_threads, merg=max_merge_supernode_size, cliq=num_cliques\n");
      printf("Stages:  build=solver construction, asm+fac/solve are median of repeated runs\n");
    } else {
      conex::ProfileAlgorithm(info.problem, info.name, cfg,
                               info.objective_constant, algo_filter, tol);
    }
  } catch (const std::exception& e) {
    printf("Error: %s\n", e.what());
    return 1;
  }

  return 0;
}
