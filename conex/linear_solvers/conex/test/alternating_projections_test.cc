#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/alternating_projections.h"
#include "conex/common/affine_projection.h"
#include "conex/common/eja_ops.h"
#include "conex/common/problem.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

TEST(AlternatingProjections, Feasibility) {
  // Find a feasible point for Ax <= b using alternating projections.
  srand(42);
  const int n = 5, m = 10;

  MatrixXd A_dense = MatrixXd::Random(m, n);
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  // b = ones, so x=0 is feasible with s = b = ones > 0.
  VectorXd b = VectorXd::Ones(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, b, vars);
  auto affine = AffineProjection::Build(problem);

  // Start with s that violates the cone (some negative entries).
  RowSpace s = affine.MakeVariable();
  setFromVector(s, VectorXd::Random(m));

  auto result = AlternatingProjections(affine, s, 200, 1e-10, false);

  printf("AlternatingProjections: %d iters, residual=%.2e\n",
         result.iterations, result.residual);
  EXPECT_LT(result.residual, 1e-8);

  // Verify s >= 0.
  for (int i = 0; i < m; ++i)
    EXPECT_GE(s.col()(i), -1e-10);

  // Verify s is in the affine subspace by projecting again — should be no-op.
  RowSpace s_copy = s;
  affine.Project(s_copy);
  double affine_err = std::sqrt(squaredNorm(addScaled(s, s_copy, 1.0, -1.0)));
  EXPECT_LT(affine_err, 1e-8);

  printf("  cone violation: %.2e, affine error: %.2e\n",
         -s.col().minCoeff(), affine_err);
}

TEST(AlternatingProjections, MultipleConstraints) {
  // Two constraint blocks.
  srand(77);
  const int n = 6, m1 = 8, m2 = 5;

  auto toSparse = [](const MatrixXd& M) {
    std::vector<Eigen::Triplet<double>> t;
    for (int i = 0; i < M.rows(); ++i)
      for (int j = 0; j < M.cols(); ++j)
        t.emplace_back(i, j, M(i, j));
    Eigen::SparseMatrix<double> S(M.rows(), M.cols());
    S.setFromTriplets(t.begin(), t.end());
    return S;
  };

  auto A1 = toSparse(MatrixXd::Random(m1, n));
  auto A2 = toSparse(MatrixXd::Random(m2, n));

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A1, VectorXd::Ones(m1), vars);
  problem.AddLinearConstraint(A2, VectorXd::Ones(m2), vars);
  auto affine = AffineProjection::Build(problem);

  RowSpace s = affine.MakeVariable();
  setFromVector(s, VectorXd::Random(m1 + m2));

  auto result = AlternatingProjections(affine, s, 200, 1e-10, false);

  printf("AlternatingProjections (2 constraints): %d iters, residual=%.2e\n",
         result.iterations, result.residual);
  EXPECT_LT(result.residual, 1e-8);

  // All entries non-negative.
  for (int i = 0; i < m1 + m2; ++i)
    EXPECT_GE(s.col()(i), -1e-10);
}

}  // namespace
}  // namespace conex
