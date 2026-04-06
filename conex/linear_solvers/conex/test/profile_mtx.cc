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
};

ProfileResult ProfileMatrix(const std::string& name,
                            const Eigen::SparseMatrix<double>& A,
                            bool is_quadratic,
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
  int n_solve = num_vars;

  if (is_quadratic) {
    // Square matrix: treat as Q in min 0.5 x^T Q x.
    // Symmetrize: Q = A + A^T to ensure SPD-like structure.
    Eigen::SparseMatrix<double> Q = A + Eigen::SparseMatrix<double>(A.transpose());
    // Add diagonal to ensure positive definiteness.
    for (int i = 0; i < num_vars; ++i)
      Q.coeffRef(i, i) += num_vars;
    problem.AddQuadraticCost(Q, vars);
  } else {
    problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(m), vars);
  }

  // Preprocess.
  auto [reduced, expansion] = Preprocess(problem);
  n_solve = reduced.num_variables();

  if (expansion.was_reduced()) {
    fprintf(stderr, "  %s: structural rank %d / %d (dropped %d columns)\n",
            name.c_str(), n_solve, num_vars, num_vars - n_solve);
  }

  // Build solver.
  auto t0 = Clock::now();
  auto solver = Solver::Build(reduced, cfg);
  auto* kkt = solver.solver();
  auto t1 = Clock::now();
  res.build_us = us(t0, t1);
  res.num_cliques = solver.tree_solver()
      ? solver.tree_solver()->num_subsystems() : 1;
  res.total_setup_us = res.build_us;

  // AssembleAndFactor.
  bool ok = kkt->AssembleAndFactor();
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
    kkt->AssembleAndFactor();
    auto tb = Clock::now();
    af_times[i] = us(ta, tb);
  }
  std::sort(af_times.begin(), af_times.end());
  res.assemble_factor_us = af_times[iters / 2];

  // Build RHS.
  Eigen::VectorXd x_true = Eigen::VectorXd::Random(n_solve);
  Eigen::VectorXd rhs;

  if (is_quadratic) {
    // Q x = rhs.  Build Q in reduced space.
    Eigen::SparseMatrix<double> Q = A + Eigen::SparseMatrix<double>(A.transpose());
    for (int i = 0; i < num_vars; ++i) Q.coeffRef(i, i) += num_vars;
    if (expansion.was_reduced()) {
      // Subselect rows/cols.
      std::vector<Eigen::Triplet<double>> trips;
      std::vector<int> inv(num_vars, -1);
      for (int j = 0; j < n_solve; ++j) inv[expansion.col_map[j]] = j;
      for (int k = 0; k < Q.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k); it; ++it) {
          int nr = inv[it.row()], nc = inv[it.col()];
          if (nr >= 0 && nc >= 0) trips.emplace_back(nr, nc, it.value());
        }
      Eigen::SparseMatrix<double> Qr(n_solve, n_solve);
      Qr.setFromTriplets(trips.begin(), trips.end());
      rhs = Qr * x_true;
    } else {
      rhs = Q * x_true;
    }
  } else {
    // A^T A x = rhs.
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
    rhs = Ad_solve.transpose() * (Ad_solve * x_true);
  }

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

  res.residual = (sol - x_true).norm() / x_true.norm();
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

struct MatrixFile {
  std::string path;
  std::string name;
  Eigen::SparseMatrix<double> A;
  bool is_quadratic = false;  // true for square matrices (Q cost)
};

// Parse comma-separated int list: "1,2,4" → {1, 2, 4}.
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

  // Load matrices.
  std::vector<MatrixFile> matrices;
  for (const auto& path : mtx_paths) {
    MatrixFile mf;
    mf.path = path;
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
    if (mf.A.rows() == mf.A.cols()) {
      mf.is_quadratic = true;
      fprintf(stderr, "  %s: square %dx%d, using as quadratic cost\n",
              mf.name.c_str(), (int)mf.A.rows(), (int)mf.A.cols());
    }
    // Transpose wide matrices to make them tall (overdetermined).
    if (!mf.is_quadratic && mf.A.rows() < mf.A.cols()) {
      fprintf(stderr, "  Transposing %s (%dx%d -> %dx%d)\n",
              mf.name.c_str(), (int)mf.A.rows(), (int)mf.A.cols(),
              (int)mf.A.cols(), (int)mf.A.rows());
      mf.A = Eigen::SparseMatrix<double>(mf.A.transpose());
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
          auto res = ProfileMatrix(mf.name, mf.A, mf.is_quadratic, run_cfg);
          PrintResult(res, run_cfg);
        } catch (const std::exception& e) {
          fprintf(stderr, "  %s: exception: %s\n", mf.name.c_str(), e.what());
        }
      }
      printf("\n");
    }
  }

  printf("Columns: thrd=num_threads, merg=max_merge_supernode_size, cliq=num_cliques\n");
  printf("Stages:  build=solver construction, asm+fac/solve are median of repeated runs\n");

  return 0;
}
