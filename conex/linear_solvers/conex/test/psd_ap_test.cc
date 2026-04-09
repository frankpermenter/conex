#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>

#include "conex/algorithms/alternating_projections.h"
#include "conex/common/affine_projection.h"
#include "conex/common/eja_ops.h"
#include "conex/common/psd_cone_ops.h"
#include "conex/common/problem.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

Eigen::SparseMatrix<double> toSparse(const MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i, j)) > 1e-14)
        trips.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(trips.begin(), trips.end());
  return S;
}

// Find X ≽ 0 in an affine subspace using alternating projections.
// Problem: X = B + x1*A1 + x2*A2 ≽ 0.
TEST(PSD_AP, Feasibility) {
  srand(42);
  const int n = 3;  // 3x3 matrices
  const int p = 5;  // 5 free variables

  // Random symmetric A_i.
  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars;
  for (int k = 0; k < p; ++k) {
    MatrixXd Ak = MatrixXd::Random(n, n);
    Ak = 0.5 * (Ak + Ak.transpose());
    A_list.push_back(toSparse(Ak));
    vars.push_back(k);
  }

  // B = I (feasible at x=0).
  MatrixXd B = MatrixXd::Identity(n, n);

  Problem problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars);

  auto affine = AffineProjection::Build(problem);
  RowSpace s = affine.MakeVariable();
  // Start from the affine term b (= vec(B) = vec(I)). This should be in
  // the affine subspace, so projecting onto it should be a no-op.
  // Then cone-project onto PSD, then affine-project again.
  {
    RowSpace b = affine.GetAffineTerm();
    printf("  b norm: %.4f\n", std::sqrt(squaredNorm(b)));
    RowSpace s_test = affine.MakeVariable();
    // Set s_test = b.
    for (int i = 0; i < s_test.total_rows(); ++i)
      s_test.segment_ptr(0)[i] = b.segment_ptr(0)[i];
    RowSpace s_before = s_test;
    affine.Project(s_test);
    double proj_err = std::sqrt(squaredNorm(addScaled(s_test, s_before, 1.0, -1.0)));
    printf("  affine proj of b: err=%.4e\n", proj_err);
  }
  Eigen::Map<MatrixXd>(s.segment_ptr(0), n, n) = -2.0 * MatrixXd::Identity(n, n);

  auto result = AlternatingProjections(affine, s, 20, 1e-8, true);
  printf("PSD AP: %d iters, residual=%.2e\n",
         result.iterations, result.residual);
  EXPECT_LT(result.residual, 1e-6);

  // Verify result is PSD: extract n×n matrix and check eigenvalues.
  int n2 = n * n;
  ASSERT_EQ(s.total_rows(), n2);
  Eigen::Map<MatrixXd> X(s.segment_ptr(0), n, n);
  Eigen::SelfAdjointEigenSolver<MatrixXd> eig(0.5 * (X + X.transpose()));
  double min_eig = eig.eigenvalues().minCoeff();
  printf("  min eigenvalue: %.6e\n", min_eig);
  EXPECT_GT(min_eig, -1e-6);
}

// Same problem, but use NonnegOrthantOps instead of PSDConeOps.
// The result should be componentwise nonneg but NOT necessarily PSD.
TEST(PSD_AP, NonnegComparison) {
  srand(42);
  const int n = 3;
  const int p = 2;

  MatrixXd A1 = MatrixXd::Random(n, n);
  A1 = 0.5 * (A1 + A1.transpose());
  MatrixXd A2 = MatrixXd::Random(n, n);
  A2 = 0.5 * (A2 + A2.transpose());
  MatrixXd B = -MatrixXd::Identity(n, n);

  // Manually vectorize and use AddLinearConstraint with default (nonneg) ops.
  int n2 = n * n;
  std::vector<Eigen::Triplet<double>> trips;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      int row = j * n + i;
      if (std::abs(A1(i, j)) > 1e-14) trips.emplace_back(row, 0, A1(i, j));
      if (std::abs(A2(i, j)) > 1e-14) trips.emplace_back(row, 1, A2(i, j));
    }
  Eigen::SparseMatrix<double> A_vec(n2, p);
  A_vec.setFromTriplets(trips.begin(), trips.end());

  VectorXd b_vec(n2);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i)
      b_vec(j * n + i) = B(i, j);

  std::vector<int> vars = {0, 1};
  Problem problem;
  problem.AddLinearConstraint(A_vec, b_vec, vars);  // nonneg orthant

  auto affine = AffineProjection::Build(problem);
  RowSpace s = affine.MakeVariable();
  EuclideanJordanAlgebra::setOnes(s);

  auto result = AlternatingProjections(affine, s, 500, 1e-8);
  printf("Nonneg AP: %d iters, residual=%.2e\n",
         result.iterations, result.residual);
  EXPECT_LT(result.residual, 1e-6);

  // Result is componentwise nonneg.
  for (int i = 0; i < n2; ++i)
    EXPECT_GE(s.segment_ptr(0)[i], -1e-6);

  // But may NOT be PSD.
  Eigen::Map<MatrixXd> X(s.segment_ptr(0), n, n);
  Eigen::SelfAdjointEigenSolver<MatrixXd> eig(0.5 * (X + X.transpose()));
  double min_eig = eig.eigenvalues().minCoeff();
  printf("  min eigenvalue: %.6e (may be negative)\n", min_eig);
  // Don't assert PSD — just report.
}

}  // namespace
}  // namespace conex
