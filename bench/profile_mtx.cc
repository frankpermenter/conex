// Thin wrapper: profiles MTX files via the unified benchmark infrastructure.
// Kept for convenience (shorter name, MTX-specific defaults).
//
// For full options, use: benchmark_solver <file.mtx> --profile

#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "conex/common/mtx_reader.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <algorithm>
#include <chrono>
#include <numeric>

namespace conex {
namespace {

using Clock = std::chrono::high_resolution_clock;
double us(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::micro>(b - a).count();
}

struct ProfileResult {
  std::string name;
  int rows, cols, nnz;
  int num_cliques;
  double build_us;
  double assemble_factor_us;
  double solve_us;
  double total_setup_us;
  double residual;
};

ProfileResult ProfileMatrix(const Model& problem,
                            const MTXInfo& info,
                            const SolverConfiguration& cfg,
                            int max_iters = -1) {
  ProfileResult res;
  res.name = info.name;
  res.rows = info.rows;
  res.cols = info.cols;
  res.nnz = info.nnz;

  // Build solver (preprocessing is internal).
  auto t0 = Clock::now();
  auto solver = Solver::Build(problem, cfg);
  auto* kkt = solver.kkt();
  auto t1 = Clock::now();
  res.build_us = us(t0, t1);
  res.num_cliques = solver.tree_solver()
      ? solver.tree_solver()->num_subsystems() : 1;
  res.total_setup_us = res.build_us;

  int n_solve = kkt->number_of_variables();

  if (solver.was_reduced()) {
    fprintf(stderr, "  %s: structural rank %d / %d (dropped %d columns)\n",
            info.name.c_str(), n_solve, info.cols, info.cols - n_solve);
  }

  int iters = (max_iters >= 0) ? max_iters
                               : std::max(20, 2000 / std::max(1, n_solve));
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

  // Build RHS from random ground truth.
  Eigen::VectorXd x_true = Eigen::VectorXd::Random(n_solve);
  // For a general Gram system Gx = rhs, we can use kkt->Solve to get
  // a reference. Instead, build rhs = G * x_true by factoring once and
  // measuring ||solve(G * x_true) - x_true||.
  // Since we don't have G explicitly, use a random rhs and measure
  // solve consistency: solve twice and check agreement.
  Eigen::VectorXd rhs = Eigen::VectorXd::Random(n_solve);

  // Solve timing.
  kkt->Solve(rhs);  // warm up
  std::vector<double> s_times(iters);
  Eigen::VectorXd sol;
  for (int i = 0; i < iters; ++i) {
    auto ta = Clock::now();
    sol = kkt->Solve(rhs);
    auto tb = Clock::now();
    s_times[i] = us(ta, tb);
  }
  std::sort(s_times.begin(), s_times.end());
  res.solve_us = s_times[iters / 2];

  // Residual: solve twice, check consistency.
  Eigen::VectorXd sol2 = kkt->Solve(rhs);
  res.residual = (sol - sol2).norm() / (sol.norm() + 1e-15);
  return res;
}

}  // namespace
}  // namespace conex

using namespace conex;

void PrintHeader() {
  printf("%-20s %4s %4s %6s | %10s | %10s %10s | %10s | %10s\n",
         "Matrix", "thrd", "merg", "cliq",
         "build",
         "asm+fac", "solve",
         "setup_tot", "residual");
  printf("%s\n", std::string(110, '-').c_str());
}

void PrintResult(const ProfileResult& res, const SolverConfiguration& cfg) {
  if (res.assemble_factor_us < 0) {
    printf("%-20s %4d %4d %6d | %9.0fus |  FACTOR FAILED          | %9.0fus | rank-def\n",
           res.name.c_str(), cfg.num_threads,
           cfg.tree.max_merge_supernode_size, res.num_cliques,
           res.build_us, res.total_setup_us);
  } else {
    printf("%-20s %4d %4d %6d | %9.0fus | %9.0fus %9.0fus | %9.0fus | %10.2e\n",
           res.name.c_str(), cfg.num_threads,
           cfg.tree.max_merge_supernode_size, res.num_cliques,
           res.build_us,
           res.assemble_factor_us, res.solve_us,
           res.total_setup_us, res.residual);
  }
}

// Parse comma-separated int list: "1,2,4" -> {1, 2, 4}.
std::vector<int> ParseList(const std::string& s) {
  std::vector<int> result;
  std::istringstream ss(s);
  std::string token;
  while (std::getline(ss, token, ',')) result.push_back(std::stoi(token));
  return result;
}

int main(int argc, char* argv[]) {
  SolverConfiguration cfg;
  bool randomize = false;
  int max_iters = -1;
  std::vector<std::string> mtx_paths;
  std::vector<int> sweep_threads, sweep_merge;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--randomize") {
      randomize = true;
    } else if (arg == "--threads" && i + 1 < argc) {
      cfg.num_threads = std::stoi(argv[++i]);
    } else if (arg == "--merge" && i + 1 < argc) {
      cfg.tree.max_merge_supernode_size = std::stoi(argv[++i]);
    } else if (arg == "--sweep-threads" && i + 1 < argc) {
      sweep_threads = ParseList(argv[++i]);
    } else if (arg == "--sweep-merge" && i + 1 < argc) {
      sweep_merge = ParseList(argv[++i]);
    } else if (arg == "--reorder" && i + 1 < argc) {
      cfg.tree.supernode_reorder_method = std::stoi(argv[++i]);
    } else if (arg == "--generic") {
      cfg.tree.use_generic_factorization = true;
    } else if (arg == "--iters" && i + 1 < argc) {
      max_iters = std::stoi(argv[++i]);
    } else if (arg[0] != '-') {
      mtx_paths.push_back(arg);
    } else {
      fprintf(stderr,
        "Usage: %s [options] file.mtx ...\n"
        "  --randomize              Replace nonzeros with random values\n"
        "  --threads <n>            Number of threads (default: 1)\n"
        "  --merge <n>              Max merge supernode size (default: 5)\n"
        "  --reorder <n>            Supernode reorder method (default: 0)\n"
        "  --generic                Use generic (RLDLT) factorization\n"
        "  --iters <n>             Timing iterations (-1=auto, 0=build only)\n"
        "  --sweep-threads <list>   Sweep thread counts (e.g. 1,2,4)\n"
        "  --sweep-merge <list>     Sweep merge thresholds (e.g. 0,5,10,20)\n",
        argv[0]);
      return 1;
    }
  }

  if (mtx_paths.empty()) {
    fprintf(stderr, "No MTX files specified.\n");
    return 1;
  }

  // Load problems via the unified reader.
  struct MatrixEntry {
    Model problem;
    MTXInfo info;
  };
  std::vector<MatrixEntry> matrices;
  for (const auto& path : mtx_paths) {
    try {
      auto [problem, info] = ReadMTX(path, randomize);
      if (info.is_quadratic) {
        fprintf(stderr, "  %s: square %dx%d, using as quadratic cost\n",
                info.name.c_str(), info.rows, info.cols);
      }
      if (info.was_transposed) {
        fprintf(stderr, "  Transposed %s (%dx%d -> %dx%d)\n",
                info.name.c_str(), info.cols, info.rows, info.rows, info.cols);
      }
      matrices.push_back({std::move(problem), std::move(info)});
    } catch (const std::exception& e) {
      fprintf(stderr, "  %s: %s\n", path.c_str(), e.what());
    }
  }

  // Build sweep configurations.
  if (sweep_threads.empty()) sweep_threads = {cfg.num_threads};
  if (sweep_merge.empty()) sweep_merge = {cfg.tree.max_merge_supernode_size};

  for (int threads : sweep_threads) {
    for (int merge : sweep_merge) {
      SolverConfiguration run_cfg = cfg;
      run_cfg.num_threads = threads;
      run_cfg.tree.max_merge_supernode_size = merge;

      PrintHeader();
      for (auto& mf : matrices) {
        try {
          auto res = ProfileMatrix(mf.problem, mf.info, run_cfg, max_iters);
          PrintResult(res, run_cfg);
        } catch (const std::exception& e) {
          fprintf(stderr, "  %s: exception: %s\n", mf.info.name.c_str(), e.what());
        }
      }
      printf("\n");
    }
  }

  printf("Columns: thrd=num_threads, merg=max_merge_supernode_size, cliq=num_cliques\n");
  printf("Stages:  build=solver construction, asm+fac/solve are median of repeated runs\n");

  return 0;
}
