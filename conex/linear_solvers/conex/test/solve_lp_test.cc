#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_lp.h"
#include "conex/common/model.h"

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
  // Sense::GE(A, b=-1) means Ax >= -1. Stores (A, 1): Ax + 1 >= 0.
  // Central path at W=1: d = 1-(0+1) = 0. Cost = A^T ones.
  VectorXd b = -VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  Model problem;
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

  Model problem;
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
  // Internal form: Ax + b >= 0.
  // Sense::GE(A, b) stores (A, -b): Ax - b >= 0, i.e., Ax >= b.
  // Sense::LE(A, b) stores (-A, b): -Ax + b >= 0, i.e., Ax <= b.
  auto A = toSparse(MatrixXd::Identity(m, n).leftCols(n));
  VectorXd b = VectorXd::Ones(m);

  // Test with Sense::LE: Ax <= b → stored (-A, b), violation = -Ax + b.
  {
    Model problem;
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

  // Test with Sense::GE: Ax >= b → stored (A, -b), violation = Ax - b.
  {
    Model problem;
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

}  // namespace
}  // namespace conex
