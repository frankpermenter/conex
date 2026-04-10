// Benchmark solver: runs geodesic IPM on benchmark instances.
//
// Usage:
//   ./benchmark_solver <file.mps|file.dat-s|file.cbf>
//   ./benchmark_solver --synthetic lp <m> <n> [seed]
//   ./benchmark_solver --synthetic sdp <n> <p> [seed]
//   ./benchmark_solver --synthetic socp <dim> <p> [seed]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/cbf_reader.h"
#include "conex/common/eja_ops.h"
#include "conex/common/mps_reader.h"
#include "conex/common/problem.h"
#include "conex/common/sdpa_reader.h"
#include "conex/common/solver.h"

namespace conex {
namespace {

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

Problem MakeSyntheticLP(int m, int n, int seed) {
  srand(seed);
  MatrixXd A_dense = MatrixXd::Random(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A_dense.transpose() * VectorXd::Ones(m);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(ToSparse(A_dense), b, vars);
  problem.SetLinearCost(c);
  return problem;
}

Problem MakeSyntheticSDP(int n, int p, int seed) {
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
  Problem problem;
  problem.AddPSDConstraint(A_list, ToSparse(MatrixXd::Identity(n, n)), vars,
                            false);
  problem.SetLinearCost(c);
  return problem;
}

Problem MakeSyntheticSOCP(int dim, int p, int seed) {
  srand(seed);
  MatrixXd A_dense = MatrixXd::Random(dim, p);
  VectorXd b = VectorXd::Zero(dim);
  b(0) = 1.0;
  VectorXd c = A_dense.row(0).transpose();
  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddSOCConstraint(ToSparse(A_dense), b, vars);
  problem.SetLinearCost(c);
  return problem;
}

void RunBenchmark(const Problem& problem, const std::string& name) {
  printf("=== %s ===\n", name.c_str());
  printf("  Variables: %d, Constraints: %d\n",
         problem.num_variables(), problem.num_constraints());

  auto t0 = std::chrono::high_resolution_clock::now();
  auto solver = Solver::Build(problem);
  auto t1 = std::chrono::high_resolution_clock::now();
  double build_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  printf("  Build: %.1f ms\n", build_ms);

  auto* kkt = solver.solver();

  // Prepare cost RHS.
  auto cost_rhs = kkt->MakeSolverRHS();
  if (problem.has_linear_cost()) {
    cost_rhs = kkt->MakeBlockVariable(problem.linear_cost());
  } else {
    cost_rhs.SetZero();
  }

  // --- Geodesic LP ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 50, 0, 1e-8);
    auto t3 = std::chrono::high_resolution_clock::now();
    double solve_ms =
        std::chrono::duration<double, std::milli>(t3 - t2).count();
    printf("  GeodesicLP: %d fac, %d sol, mu=%.2e, %.1f ms\n",
           result.total_factorizations, result.total_solves,
           result.mu, solve_ms);
    if (result.optimality.dual_residual > 0) {
      printf("    Opt: dual_res=%.2e, compl=%.2e, min_s=%.2e, min_lam=%.2e\n",
             result.optimality.dual_residual,
             result.optimality.complementarity,
             result.optimality.min_slack,
             result.optimality.min_dual);
    }
  }

  // --- Hybrid ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto result = SolveGeodesicHybrid(*kkt, cost_rhs, W, 100, 1e-8);
    auto t3 = std::chrono::high_resolution_clock::now();
    double solve_ms =
        std::chrono::duration<double, std::milli>(t3 - t2).count();
    printf("  Hybrid: %d fac, %d sol, gap=%.2e, %.1f ms\n",
           result.total_factorizations, result.total_solves,
           result.complementarity, solve_ms);
    if (result.optimality.dual_residual > 0) {
      printf("    Opt: dual_res=%.2e, compl=%.2e, min_s=%.2e, min_lam=%.2e\n",
             result.optimality.dual_residual,
             result.optimality.complementarity,
             result.optimality.min_slack,
             result.optimality.min_dual);
    }
  }

  printf("\n");
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  if (argc < 2) {
    printf("Usage:\n");
    printf("  %s <file.mps|file.dat-s|file.cbf>\n", argv[0]);
    printf("  %s --synthetic lp <m> <n> [seed]\n", argv[0]);
    printf("  %s --synthetic sdp <n> <p> [seed]\n", argv[0]);
    printf("  %s --synthetic socp <dim> <p> [seed]\n", argv[0]);
    return 1;
  }

  std::string arg1 = argv[1];

  if (arg1 == "--synthetic") {
    if (argc < 5) {
      printf("Need: --synthetic <type> <dim1> <dim2> [seed]\n");
      return 1;
    }
    std::string type = argv[2];
    int d1 = std::atoi(argv[3]);
    int d2 = std::atoi(argv[4]);
    int seed = argc > 5 ? std::atoi(argv[5]) : 42;

    if (type == "lp") {
      char name[64];
      snprintf(name, sizeof(name), "Synthetic LP (%dx%d, seed=%d)", d1, d2, seed);
      conex::RunBenchmark(conex::MakeSyntheticLP(d1, d2, seed), name);
    } else if (type == "sdp") {
      char name[64];
      snprintf(name, sizeof(name), "Synthetic SDP (n=%d, p=%d, seed=%d)", d1, d2, seed);
      conex::RunBenchmark(conex::MakeSyntheticSDP(d1, d2, seed), name);
    } else if (type == "socp") {
      char name[64];
      snprintf(name, sizeof(name), "Synthetic SOCP (dim=%d, p=%d, seed=%d)", d1, d2, seed);
      conex::RunBenchmark(conex::MakeSyntheticSOCP(d1, d2, seed), name);
    } else {
      printf("Unknown type: %s\n", type.c_str());
      return 1;
    }
  } else {
    // File input.
    std::string filename = arg1;
    std::string ext = filename.substr(filename.find_last_of('.') + 1);

    try {
      if (ext == "mps") {
        auto [problem, info] = conex::ReadMPS(filename);
        char name[256];
        snprintf(name, sizeof(name), "MPS: %s (%d vars, %d LE, %d GE, %d EQ)",
                 info.name.c_str(), info.num_variables,
                 info.num_le_rows, info.num_ge_rows, info.num_eq_rows);
        conex::RunBenchmark(problem, name);
      } else if (ext == "dat-s" || ext == "dat" ||
                 filename.find(".dat-s") != std::string::npos) {
        auto [problem, info] = conex::ReadSDPA(filename);
        char name[256];
        snprintf(name, sizeof(name), "SDPA: %d constraints, %d blocks, dim=%d",
                 info.num_constraints, info.num_blocks, info.total_matrix_dim);
        conex::RunBenchmark(problem, name);
      } else if (ext == "cbf") {
        auto [problem, info] = conex::ReadCBF(filename);
        char name[256];
        snprintf(name, sizeof(name), "CBF: %d vars, %d cons",
                 info.num_variables, info.num_constraints);
        conex::RunBenchmark(problem, name);
      } else {
        printf("Unknown file extension: %s\n", ext.c_str());
        return 1;
      }
    } catch (const std::exception& e) {
      printf("Error: %s\n", e.what());
      return 1;
    }
  }

  return 0;
}
