#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/alternating_projections.h"
#include "conex/common/affine_projection.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"

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

  Model problem;
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

  Model problem;
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

// SOC constraint: find (t, x) in affine subspace with ||x|| <= t.
TEST(AlternatingProjections, SOC) {
  srand(42);
  const int vec_dim = 4;
  const int n_soc = 1 + vec_dim;  // SOC dimension
  const int p = 3;                // free variables

  auto toSparse = [](const MatrixXd& M) {
    std::vector<Eigen::Triplet<double>> t;
    for (int i = 0; i < M.rows(); ++i)
      for (int j = 0; j < M.cols(); ++j)
        if (std::abs(M(i, j)) > 1e-14)
          t.emplace_back(i, j, M(i, j));
    Eigen::SparseMatrix<double> S(M.rows(), M.cols());
    S.setFromTriplets(t.begin(), t.end());
    return S;
  };

  MatrixXd A_dense = MatrixXd::Random(n_soc, p);
  // b = (3, 1, 1, ..., 1): feasible at x=0 (||b1|| = 2 < 3 = b0).
  VectorXd b = VectorXd::Ones(n_soc);
  b(0) = 3.0;

  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddSOCConstraint(toSparse(A_dense), b, vars);
  auto affine = AffineProjection::Build(problem);

  RowSpace s = affine.MakeVariable();
  // Start from a point outside the cone: large vector part, small scalar.
  {
    VectorXd s0(n_soc);
    s0(0) = 0.1;
    s0.tail(vec_dim) = 3.0 * VectorXd::Random(vec_dim);
    setFromVector(s, s0);
  }

  auto result = AlternatingProjections(affine, s, 500, 1e-8);
  printf("SOC AP: %d iters, residual=%.2e\n",
         result.iterations, result.residual);
  EXPECT_LT(result.residual, 1e-6);

  // Verify result is in the SOC: t >= ||x||.
  double t = s.segment_ptr(0)[0];
  Eigen::Map<VectorXd> x(s.segment_ptr(0) + 1, vec_dim);
  double cone_viol = x.norm() - t;
  printf("  t=%.4f, ||x||=%.4f, violation=%.2e\n", t, x.norm(), cone_viol);
  EXPECT_LT(cone_viol, 1e-6);
}

// Mixed: nonneg + SOC constraints on the same variables.
TEST(AlternatingProjections, MixedNonnegSOC) {
  srand(99);
  const int p = 4;
  const int m_nn = 6;       // nonneg rows
  const int vec_dim = 3;
  const int n_soc = 1 + vec_dim;

  auto toSparse = [](const MatrixXd& M) {
    std::vector<Eigen::Triplet<double>> t;
    for (int i = 0; i < M.rows(); ++i)
      for (int j = 0; j < M.cols(); ++j)
        if (std::abs(M(i, j)) > 1e-14)
          t.emplace_back(i, j, M(i, j));
    Eigen::SparseMatrix<double> S(M.rows(), M.cols());
    S.setFromTriplets(t.begin(), t.end());
    return S;
  };

  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  // Nonneg: Ax + b >= 0, b = ones.
  problem.AddLinearConstraint(
      toSparse(MatrixXd::Random(m_nn, p)), VectorXd::Ones(m_nn), vars);
  // SOC: ||A1 x + b1|| <= A0 x + b0, b = (3, 0, ...).
  VectorXd b_soc = VectorXd::Zero(n_soc);
  b_soc(0) = 3.0;
  problem.AddSOCConstraint(
      toSparse(MatrixXd::Random(n_soc, p)), b_soc, vars);

  auto affine = AffineProjection::Build(problem);
  RowSpace s = affine.MakeVariable();
  EuclideanJordanAlgebra::setOnes(s);

  auto result = AlternatingProjections(affine, s, 500, 1e-8);
  printf("Mixed AP: %d iters, residual=%.2e\n",
         result.iterations, result.residual);
  EXPECT_LT(result.residual, 1e-6);

  // Nonneg segment: all entries >= 0.
  for (int i = 0; i < m_nn; ++i)
    EXPECT_GE(s.segment_ptr(0)[i], -1e-6);

  // SOC segment: t >= ||x||.
  double t = s.segment_ptr(1)[0];
  Eigen::Map<VectorXd> x(s.segment_ptr(1) + 1, vec_dim);
  printf("  SOC: t=%.4f, ||x||=%.4f\n", t, x.norm());
  EXPECT_GE(t + 1e-6, x.norm());
}

}  // namespace
}  // namespace conex
