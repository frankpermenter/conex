// Benchmark tool for sparse least-squares on MTX files.
// Uses Problem + Solver API.  Times: build, asm+fac, solve.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "conex/common/problem.h"
#include "conex/common/solver.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
namespace {

using Clock = std::chrono::high_resolution_clock;
double us(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::micro>(b - a).count();
}

Eigen::SparseMatrix<double> ReadMTX(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open: " + path);
  }
  std::string line;
  bool symmetric = false;
  bool pattern_only = false;
  while (std::getline(f, line)) {
    if (line.find("symmetric") != std::string::npos) symmetric = true;
    if (line.find("pattern") != std::string::npos) pattern_only = true;
    if (line[0] != '%') break;
  }
  int nrows, ncols, nnz;
  std::istringstream(line) >> nrows >> ncols >> nnz;
  std::vector<Eigen::Triplet<double>> triplets;
  for (int i = 0; i < nnz; i++) {
    int r, c;
    double v = 1.0;
    if (pattern_only) {
      f >> r >> c;
    } else {
      f >> r >> c >> v;
    }
    r--; c--;
    triplets.emplace_back(r, c, v);
    if (symmetric && r != c) {
      triplets.emplace_back(c, r, v);
    }
  }
  Eigen::SparseMatrix<double> A(nrows, ncols);
  A.setFromTriplets(triplets.begin(), triplets.end());
  return A;
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
  double cond_AtA = 0;
};

ProfileResult ProfileMatrix(const std::string& name,
                            const Eigen::SparseMatrix<double>& A,
                            const SolverConfiguration& cfg) {
  ProfileResult res;
  res.name = name;
  res.rows = A.rows();
  res.cols = A.cols();
  res.nnz = A.nonZeros();

  const int num_vars = A.cols();
  const int m = A.rows();

  // Build Problem.
  std::vector<int> vars(num_vars);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(m), vars);

  // Preprocess.
  auto [reduced, expansion] = Preprocess(problem);
  const int n_solve = reduced.num_variables();

  if (expansion.was_reduced()) {
    fprintf(stderr, "  %s: structural rank %d / %d (dropped %d columns)\n",
            name.c_str(), n_solve, num_vars, num_vars - n_solve);
  }

  // Build solver.
  auto t0 = Clock::now();
  auto solver = Solver::Build(reduced, cfg);
  auto t1 = Clock::now();
  res.build_us = us(t0, t1);
  res.num_cliques = solver.tree_solver()
      ? solver.tree_solver()->num_subsystems() : 1;
  res.total_setup_us = res.build_us;

  // AssembleAndFactor.
  bool ok = solver.AssembleAndFactor();
  if (!ok) {
    res.assemble_factor_us = -1;
    res.solve_us = -1;
    res.residual = -1;
    return res;
  }

  int iters = std::max(20, 2000 / std::max(1, num_vars));
  std::vector<double> af_times(iters);
  for (int i = 0; i < iters; ++i) {
    auto ta = Clock::now();
    solver.AssembleAndFactor();
    auto tb = Clock::now();
    af_times[i] = us(ta, tb);
  }
  std::sort(af_times.begin(), af_times.end());
  res.assemble_factor_us = af_times[iters / 2];

  // Build RHS in reduced space.
  Eigen::VectorXd x_true = Eigen::VectorXd::Random(n_solve);
  Eigen::SparseMatrix<double> A_solve = A;
  if (expansion.was_reduced()) {
    std::vector<Eigen::Triplet<double>> trips;
    std::vector<int> inv(num_vars, -1);
    for (int j = 0; j < n_solve; ++j) inv[expansion.col_map[j]] = j;
    for (int k = 0; k < A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
        int nc = inv[it.col()];
        if (nc >= 0) trips.emplace_back(it.row(), nc, it.value());
      }
    A_solve.resize(m, n_solve);
    A_solve.setFromTriplets(trips.begin(), trips.end());
  }
  Eigen::MatrixXd Ad_solve(A_solve);
  Eigen::VectorXd rhs = Ad_solve.transpose() * (Ad_solve * x_true);

  // Condition number.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Ad_solve);
  auto sv = svd.singularValues();
  res.cond_AtA = (sv(sv.size() - 1) > 0)
      ? (sv(0) / sv(sv.size() - 1)) * (sv(0) / sv(sv.size() - 1))
      : std::numeric_limits<double>::infinity();

  // Solve timing.
  solver.Solve(rhs);  // warm up
  std::vector<double> s_times(iters);
  Eigen::VectorXd sol;
  for (int i = 0; i < iters; ++i) {
    auto ta = Clock::now();
    sol = solver.Solve(rhs);
    auto tb = Clock::now();
    s_times[i] = us(ta, tb);
  }
  std::sort(s_times.begin(), s_times.end());
  res.solve_us = s_times[iters / 2];

  res.residual = (sol - x_true).norm() / x_true.norm();
  return res;
}

}  // namespace
}  // namespace conex

using namespace conex;

void PrintHeader() {
  printf("%-20s %4s %4s %6s | %10s | %10s %10s | %10s | %10s %10s\n",
         "Matrix", "thrd", "merg", "cliq",
         "build",
         "asm+fac", "solve",
         "setup_tot", "residual", "cond(A'A)");
  printf("%s\n", std::string(120, '-').c_str());
}

void PrintResult(const ProfileResult& res, const SolverConfiguration& cfg) {
  if (res.assemble_factor_us < 0) {
    printf("%-20s %4d %4d %6d | %9.0fus |  FACTOR FAILED          | %9.0fus | rank-def\n",
           res.name.c_str(), cfg.num_threads,
           cfg.tree.max_merge_supernode_size, res.num_cliques,
           res.build_us, res.total_setup_us);
  } else {
    printf("%-20s %4d %4d %6d | %9.0fus | %9.0fus %9.0fus | %9.0fus | %10.2e %10.2e\n",
           res.name.c_str(), cfg.num_threads,
           cfg.tree.max_merge_supernode_size, res.num_cliques,
           res.build_us,
           res.assemble_factor_us, res.solve_us,
           res.total_setup_us, res.residual, res.cond_AtA);
  }
}

struct MatrixFile {
  std::string path;
  std::string name;
  Eigen::SparseMatrix<double> A;
};

int main(int argc, char* argv[]) {
  SolverConfiguration cfg;
  bool randomize = false;
  std::vector<std::string> mtx_paths;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--randomize") {
      randomize = true;
    } else if (arg == "--threads" && i + 1 < argc) {
      cfg.num_threads = std::stoi(argv[++i]);
    } else if (arg == "--merge" && i + 1 < argc) {
      cfg.tree.max_merge_supernode_size = std::stoi(argv[++i]);
    } else if (arg[0] != '-') {
      mtx_paths.push_back(arg);
    } else {
      fprintf(stderr, "Usage: %s [--randomize] [--threads <n>] [--merge <n>] file.mtx ...\n",
              argv[0]);
      return 1;
    }
  }

  if (mtx_paths.empty()) {
    fprintf(stderr, "No MTX files specified.\n");
    return 1;
  }

  // Load matrices.
  std::vector<MatrixFile> matrices;
  for (const auto& path : mtx_paths) {
    MatrixFile mf;
    mf.path = path;
    // Extract name from path.
    auto slash = path.rfind('/');
    auto dot = path.rfind('.');
    mf.name = path.substr(slash == std::string::npos ? 0 : slash + 1,
                           dot - (slash == std::string::npos ? 0 : slash + 1));
    try {
      mf.A = ReadMTX(path);
    } catch (const std::exception& e) {
      fprintf(stderr, "  %s: %s\n", mf.name.c_str(), e.what());
      continue;
    }
    // Skip square matrices (not least-squares problems).
    if (mf.A.rows() == mf.A.cols()) {
      fprintf(stderr, "  Skipping %s (square, %dx%d)\n",
              mf.name.c_str(), (int)mf.A.rows(), (int)mf.A.cols());
      continue;
    }
    if (randomize) {
      srand(42);
      for (int k = 0; k < mf.A.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(mf.A, k); it; ++it)
          it.valueRef() = (double)rand() / RAND_MAX - 0.5;
      mf.name += "_rand";
    }
    matrices.push_back(std::move(mf));
  }

  PrintHeader();
  for (auto& mf : matrices) {
    try {
      auto res = ProfileMatrix(mf.name, mf.A, cfg);
      PrintResult(res, cfg);
    } catch (const std::exception& e) {
      fprintf(stderr, "  %s: exception: %s\n", mf.name.c_str(), e.what());
    }
  }

  printf("\nColumns: thrd=num_threads, merg=max_merge_supernode_size, cliq=num_cliques\n");
  printf("Stages:  build=solver construction, asm+fac/solve are median of repeated runs\n");

  return 0;
}
