#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_lp.h"
#include "conex/common/problem.h"

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
  // For Ax <= b with slack s = b - Ax, central path cost is -A^T ones.
  VectorXd c = -(A.transpose() * VectorXd::Ones(m));

  Problem problem;
  problem.AddLinearConstraint(A, b, Sense::LE);
  problem.SetLinearCost(c);

  auto result = SolveLP(problem);
  printf("SolveLP(LE): gap=%.2e, %d fac, %d sol\n",
         result.gap, result.factorizations, result.solves);
  EXPECT_LT(std::abs(result.gap), 1e-7);
}

// TODO: Double-sided test needs smarter initialization — the geodesic
// IPM assumes W=ones is on the central path (all slacks = 1), which
// doesn't hold for asymmetric bounds.

}  // namespace
}  // namespace conex
