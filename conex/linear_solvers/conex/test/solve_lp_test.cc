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

TEST(SolveLP, Basic) {
  srand(42);
  const int n = 5, m = 10;

  MatrixXd A_dense = MatrixXd::Random(m, n);
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  // Model Ax >= b via -Ax <= -b.
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

}  // namespace
}  // namespace conex
