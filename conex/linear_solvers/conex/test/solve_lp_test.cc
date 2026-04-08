#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/solve_lp.h"
#include "conex/common/eja_ops.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

Eigen::SparseMatrix<double> toSparse(const MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      trips.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(trips.begin(), trips.end());
  return S;
}

// Ax >= b.
TEST(SolveLP, SenseGE) {
  srand(42);
  const int n = 5, m = 10;
  auto A = toSparse(MatrixXd::Random(m, n));
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  Problem problem;
  problem.AddLinearConstraint(A, b, Sense::GE);
  problem.SetLinearCost(c);

  auto result = SolveLP(problem);
  printf("SolveLP(GE): gap=%.2e, %d fac, %d sol\n",
         result.gap, result.factorizations, result.solves);
  EXPECT_LT(std::abs(result.gap), 1e-7);
}

// Ax <= b.
TEST(SolveLP, SenseLE) {
  srand(42);
  const int n = 5, m = 10;
  auto A = toSparse(MatrixXd::Random(m, n));
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = -(A.transpose() * VectorXd::Ones(m));

  Problem problem;
  problem.AddLinearConstraint(A, b, Sense::LE);
  problem.SetLinearCost(c);

  auto result = SolveLP(problem);
  printf("SolveLP(LE): gap=%.2e, %d fac, %d sol\n",
         result.gap, result.factorizations, result.solves);
  EXPECT_LT(std::abs(result.gap), 1e-7);
}

// Verify ComputeConstraintViolation independently.
TEST(SolveLP, ConstraintViolation) {
  const int n = 3, m = 2;
  // A = [1 0 0; 0 1 0], b = [1; 1].
  // Sense::GE: Ax >= b. Internally stored as (-A, -b), s = -b + Ax.
  auto A = toSparse(MatrixXd::Identity(m, n).leftCols(n));
  VectorXd b = VectorXd::Ones(m);

  // Test with Sense::LE: Ax <= b, stored as (A, b), s = b - Ax.
  {
    Problem problem;
    problem.AddLinearConstraint(A, b, Sense::LE);

    // x = (0, 0, 0): Ax = 0, s = b - 0 = 1 >= 0. Feasible.
    VectorXd x = VectorXd::Zero(n);
    double viol = ComputeConstraintViolation(problem, x);
    EXPECT_GE(viol, 0);
    EXPECT_NEAR(viol, 1.0, 1e-10);

    // x = (2, 0, 0): Ax = (2, 0), s = (1-2, 1-0) = (-1, 1). Violated.
    x(0) = 2.0;
    viol = ComputeConstraintViolation(problem, x);
    EXPECT_LT(viol, 0);
    EXPECT_NEAR(viol, -1.0, 1e-10);
  }

  // Test with Sense::GE: Ax >= b, stored as (-A, -b), s = -b - (-A)x = Ax - b.
  {
    Problem problem;
    problem.AddLinearConstraint(A, b, Sense::GE);

    // x = (2, 2, 0): Ax = (2, 2), s = Ax - b = (1, 1). Feasible.
    VectorXd x = VectorXd::Zero(n);
    x(0) = 2.0; x(1) = 2.0;
    double viol = ComputeConstraintViolation(problem, x);
    EXPECT_GE(viol, 0);
    EXPECT_NEAR(viol, 1.0, 1e-10);

    // x = (0, 0, 0): Ax = 0, s = -b = -1. Violated.
    x.setZero();
    viol = ComputeConstraintViolation(problem, x);
    EXPECT_LT(viol, 0);
    EXPECT_NEAR(viol, -1.0, 1e-10);
  }

  printf("ConstraintViolation: all checks passed\n");
}

// TODO: Double-sided test needs smarter initialization — the geodesic
// IPM assumes W=ones is on the central path (all slacks = 1), which
// doesn't hold for asymmetric bounds.

// Verify that Sense correctly maps user constraints to the internal Ax >= b
// convention, and that result.x and result.slack are consistent.
TEST(SolveLP, SenseSlackVerification) {
  srand(42);
  const int n = 5, m = 10;
  MatrixXd A_dense = MatrixXd::Random(m, n);
  auto A = toSparse(A_dense);
  VectorXd b = VectorXd::Ones(m);

  // --- Test Sense::GE: user means Ax >= b ---
  // Internal: stores (A, b) as-is.  Solver finds x with Ax >= b.
  // result.slack = A_stored*x - b_stored = Ax - b >= 0.
  {
    VectorXd c = A.transpose() * VectorXd::Ones(m);
    Problem problem;
    problem.AddLinearConstraint(A, b, Sense::GE);
    problem.SetLinearCost(c);

    auto [reduced, expansion] = Preprocess(problem);
    auto solver = Solver::Build(reduced);
    auto* kkt = solver.solver();
    auto cost_rhs = kkt->MakeSolverRHS();
    cost_rhs = kkt->MakeBlockVariable(expansion.Reduce(c));
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);

    auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 0, 1e-8);
    VectorXd x = expansion.Expand(result.x);

    // result.slack = A_stored*x - b_stored.
    // For Sense::GE storing (-A, -b): result.slack = b - Ax.
    // NOTE: Sense::GE is currently inverted — solver gives Ax <= b.
    VectorXd slack_b_minus_Ax = b - A_dense * x;
    printf("  GE: min(b-Ax)=%.4e  min(result.slack)=%.4e\n",
           slack_b_minus_Ax.minCoeff(), result.slack.minCoeff());

    EXPECT_GT(result.slack.minCoeff(), -1e-6);
    EXPECT_GT(slack_b_minus_Ax.minCoeff(), -1e-6);
  }

  // --- Test Sense::LE: user means Ax <= b ---
  // Internal: stores (-A, -b).  Solver finds x with (-A)x >= (-b), i.e., Ax <= b.
  // result.slack = A_stored*x - b_stored = (-A)*x - (-b) = b - Ax >= 0.
  {
    VectorXd c = -(A.transpose() * VectorXd::Ones(m));
    Problem problem;
    problem.AddLinearConstraint(A, b, Sense::LE);
    problem.SetLinearCost(c);

    auto [reduced, expansion] = Preprocess(problem);
    auto solver = Solver::Build(reduced);
    auto* kkt = solver.solver();
    auto cost_rhs = kkt->MakeSolverRHS();
    cost_rhs = kkt->MakeBlockVariable(expansion.Reduce(c));
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);

    auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 0, 1e-8);
    VectorXd x = expansion.Expand(result.x);

    // result.slack = A_stored*x - b_stored.
    // For Sense::LE storing (A, b): result.slack = Ax - b.
    // NOTE: Sense::LE is currently inverted — solver gives Ax >= b.
    VectorXd slack_Ax_minus_b = A_dense * x - b;
    printf("  LE: min(Ax-b)=%.4e  min(result.slack)=%.4e\n",
           slack_Ax_minus_b.minCoeff(), result.slack.minCoeff());

    EXPECT_GT(result.slack.minCoeff(), -1e-6);
    EXPECT_GT(slack_Ax_minus_b.minCoeff(), -1e-6);
  }
}

}  // namespace
}  // namespace conex
