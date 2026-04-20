#include <gtest/gtest.h>
#include <cstdio>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/self_dual_embedding.h"
#include "conex/common/eja_ops.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/model.h"
#include "conex/common/sdpa_reader.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

Eigen::SparseMatrix<double> toSparse(const MatrixXd& M) {
  return M.sparseView(1e-14, 1);
}

// Small LP: min c^T x s.t. Ax + b >= 0.
TEST(HSD, SmallLP) {
  srand(42);
  const int m = 8, n = 4;
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A_dense.transpose() * VectorXd::Ones(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddLinearConstraint(toSparse(A_dense), b, vars);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveHSD(*kkt, cost_rhs, W, 100, 1e-8, true);
  printf("HSD LP: %d iters, mu=%.2e, tau=%.2e, kappa=%.2e, solved=%d\n",
         result.iterations, result.mu, result.tau, result.kappa, result.solved);
  EXPECT_TRUE(result.solved);
  EXPECT_LT(result.mu, 1e-6);
}

// Small SDP: 3×3 PSD constraint.
TEST(HSD, SmallSDP) {
  const int n = 3, p = 2;
  MatrixXd A1 = MatrixXd::Zero(n, n); A1(0, 0) = 1;
  MatrixXd A2 = MatrixXd::Zero(n, n); A2(1, 1) = 1;
  MatrixXd B = MatrixXd::Identity(n, n);

  std::vector<Eigen::SparseMatrix<double>> A_list = {toSparse(A1), toSparse(A2)};
  std::vector<int> vars = {0, 1};

  VectorXd c(p);
  c(0) = A1.trace();
  c(1) = A2.trace();

  Model problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveHSD(*kkt, cost_rhs, W, 100, 1e-8, true);
  printf("HSD SDP: %d iters, mu=%.2e, tau=%.2e, kappa=%.2e, solved=%d\n",
         result.iterations, result.mu, result.tau, result.kappa, result.solved);
  EXPECT_TRUE(result.solved);
}

// SDPA benchmark: buck3 (PSD + nonneg).
TEST(HSD, Buck3) {
  auto [problem, info] = ReadSDPA(
      "../benchmark_data/buck3.dat-s");
  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  if (problem.has_linear_cost())
    cost_rhs = kkt->MakeBlockVariable(problem.linear_cost());
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveHSD(*kkt, cost_rhs, W, 50, 1e-6, true);
  printf("HSD buck3: %d iters, mu=%.2e, tau=%.2e, kappa=%.2e, solved=%d\n",
         result.iterations, result.mu, result.tau, result.kappa, result.solved);
}

}  // namespace
}  // namespace conex
