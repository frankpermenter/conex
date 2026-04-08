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

// Note: the geodesic IPM currently expects the old Ax <= b convention
// internally (GetAffineTerm returns b, slack = b - Ax). Tests use
// manual sign flips until the algorithm is updated to Ax + b >= 0.
TEST(SolveLP, Basic) {
  srand(42);
  const int n = 5, m = 10;
  auto A = toSparse(MatrixXd::Random(m, n));
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  // Model Ax >= b: internally Ax + (-b) >= 0, stored via Sense::GE.
  // But geodesic IPM expects Ax <= b form, so we pass -A, -b directly.
  Eigen::SparseMatrix<double> negA = -A;
  VectorXd neg_b = -b;

  Problem problem;
  problem.AddLinearConstraint(negA, neg_b);
  problem.SetLinearCost(c);

  auto result = SolveLP(problem);
  printf("SolveLP: gap=%.2e, %d fac, %d sol\n",
         result.gap, result.factorizations, result.solves);
  EXPECT_LT(std::abs(result.gap), 1e-7);
}

// TODO: Add tests using Sense::GE, Sense::LE, and double-sided
// after updating the geodesic IPM to the Ax + b >= 0 convention.

}  // namespace
}  // namespace conex
