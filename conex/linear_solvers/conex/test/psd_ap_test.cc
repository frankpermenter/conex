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

  // Random A_i (not necessarily symmetric — the PSD constraint
  // is Σ A_i x_i + B ≽ 0, symmetry of X is enforced by the cone projection).
  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars;
  for (int k = 0; k < p; ++k) {
    A_list.push_back(toSparse(MatrixXd::Random(n, n)));
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
  // Check condition of A^T A.
  {
    // Reconstruct A_vec from the stored data.
    const auto& data = std::get<Problem::PSDConstraintData>(problem.constraint(0));
    Eigen::SparseMatrix<double> A_vec;
    Eigen::VectorXd b_vec;
    VectorizePSD(data.A_list, data.B, &A_vec, &b_vec);
    MatrixXd A_dense(A_vec);
    MatrixXd AtA = A_dense.transpose() * A_dense;
    Eigen::JacobiSVD<MatrixXd> svd(AtA);
    printf("  A^T A singular values: ");
    for (int i = 0; i < svd.singularValues().size(); ++i)
      printf("%.2e ", svd.singularValues()(i));
    printf("\n  cond = %.2e\n",
           svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1));
  }

  // Start from b + small perturbation (near feasible).
  {
    RowSpace b = affine.GetAffineTerm();
    for (int i = 0; i < s.total_rows(); ++i)
      s.segment_ptr(0)[i] = b.segment_ptr(0)[i] + 0.5 * ((double)rand() / RAND_MAX - 0.5);
  }

  // Manual first iteration.
  {
    RowSpace s_proj = affine.MakeVariable();
    project(s_proj, s);
    printf("  after cone proj: norm=%.4e\n", std::sqrt(squaredNorm(s_proj)));
    affine.Project(s_proj);
    printf("  after affine proj: norm=%.4e\n", std::sqrt(squaredNorm(s_proj)));
  }

  auto result = AlternatingProjections(affine, s, 500, 1e-8);
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

// Block-diagonal PSD constraint: exercises the chordal decomposition
// in SparsePSDConstraintAssembler.  The aggregate sparsity of A_i and B
// is block-diagonal (two independent blocks), so the assembler should
// split it into two smaller PSD sub-constraints.
//
// Problem: X = B + Σ x_i A_i ≽ 0 where X is 6×6 with 3×3 block structure.
TEST(PSD_AP, BlockDiagonal) {
  srand(99);
  const int n = 6;      // 6×6 matrix
  const int blk = 3;    // two 3×3 blocks
  const int p = 4;      // 4 free variables

  // Build A_i with block-diagonal sparsity: A_0, A_1 touch block (0:2, 0:2),
  // A_2, A_3 touch block (3:5, 3:5).  No cross-block entries.
  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars;

  for (int k = 0; k < 2; ++k) {
    MatrixXd M = MatrixXd::Zero(n, n);
    M.block(0, 0, blk, blk) = MatrixXd::Random(blk, blk);
    A_list.push_back(toSparse(M));
    vars.push_back(k);
  }
  for (int k = 0; k < 2; ++k) {
    MatrixXd M = MatrixXd::Zero(n, n);
    M.block(blk, blk, blk, blk) = MatrixXd::Random(blk, blk);
    A_list.push_back(toSparse(M));
    vars.push_back(2 + k);
  }

  // B = I (feasible at x = 0).
  MatrixXd B = MatrixXd::Identity(n, n);

  Problem problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars);

  auto affine = AffineProjection::Build(problem);
  RowSpace s = affine.MakeVariable();

  // The chordal decomposition should produce two segments (two 3×3 blocks),
  // so the RowSpace should have two segments of size 9 each.
  printf("  BlockDiag: num_constraints=%d, total_rows=%d\n",
         s.num_constraints(), s.total_rows());
  EXPECT_EQ(s.num_constraints(), 2);
  EXPECT_EQ(s.total_rows(), 2 * blk * blk);

  // Start from b + perturbation.
  {
    RowSpace b = affine.GetAffineTerm();
    for (int i = 0; i < s.total_rows(); ++i)
      s.segment_ptr(0)[i] = b.segment_ptr(0)[i] +
          0.3 * ((double)rand() / RAND_MAX - 0.5);
  }

  auto result = AlternatingProjections(affine, s, 500, 1e-8);
  printf("  BlockDiag AP: %d iters, residual=%.2e\n",
         result.iterations, result.residual);
  EXPECT_LT(result.residual, 1e-6);

  // Verify each block is PSD.
  for (int b = 0; b < 2; ++b) {
    Eigen::Map<MatrixXd> Xb(s.segment_ptr(b), blk, blk);
    Eigen::SelfAdjointEigenSolver<MatrixXd> eig(0.5 * (Xb + Xb.transpose()));
    double min_eig = eig.eigenvalues().minCoeff();
    printf("  block %d min eigenvalue: %.6e\n", b, min_eig);
    EXPECT_GT(min_eig, -1e-6);
  }
}

// Verify that chordal decomposition produces identical iterates to the
// non-decomposed (single-block) path on a block-diagonal problem.
TEST(PSD_AP, ChordalMatchesNonChordal) {
  srand(77);
  const int n = 6;
  const int blk = 3;
  const int p = 4;

  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars;
  for (int k = 0; k < 2; ++k) {
    MatrixXd M = MatrixXd::Zero(n, n);
    M.block(0, 0, blk, blk) = MatrixXd::Random(blk, blk);
    A_list.push_back(toSparse(M));
    vars.push_back(k);
  }
  for (int k = 0; k < 2; ++k) {
    MatrixXd M = MatrixXd::Zero(n, n);
    M.block(blk, blk, blk, blk) = MatrixXd::Random(blk, blk);
    A_list.push_back(toSparse(M));
    vars.push_back(2 + k);
  }
  MatrixXd B = MatrixXd::Identity(n, n);

  // --- Chordal (default) ---
  Problem prob_chordal;
  prob_chordal.AddPSDConstraint(A_list, toSparse(B), vars, /*use_chordal=*/true);
  auto affine_c = AffineProjection::Build(prob_chordal);
  RowSpace s_c = affine_c.MakeVariable();
  EuclideanJordanAlgebra::setOnes(s_c);
  s_c *= -1.0;  // -I — block-diagonal, PSD-infeasible, forces iterations
  auto res_c = AlternatingProjections(affine_c, s_c, 10, 1e-14);

  // --- Non-chordal (single block) ---
  Problem prob_full;
  prob_full.AddPSDConstraint(A_list, toSparse(B), vars, /*use_chordal=*/false);
  auto affine_f = AffineProjection::Build(prob_full);
  RowSpace s_f = affine_f.MakeVariable();
  EuclideanJordanAlgebra::setOnes(s_f);
  s_f *= -1.0;  // -I
  auto res_f = AlternatingProjections(affine_f, s_f, 10, 1e-14);

  printf("  chordal:     %d iters, residual=%.2e\n", res_c.iterations, res_c.residual);
  printf("  non-chordal: %d iters, residual=%.2e\n", res_f.iterations, res_f.residual);
  EXPECT_EQ(res_c.iterations, res_f.iterations);

  // Recover optimization variables x from each path via x = (A^T A)^{-1} A^T (s - b).
  // x is unambiguous — no ordering issues.
  auto recover_x = [](const AffineProjection& affine, const RowSpace& s) {
    auto* kkt = affine.solver()->solver();
    RowSpace b = kkt->GetAffineTerm();
    RowSpace r = EuclideanJordanAlgebra::addScaled(s, b, 1.0, -1.0);
    auto rhs = kkt->MakeSolverRHS();
    rhs.SetZero();
    kkt->AccumulateAtranspose(r, rhs);
    kkt->SolveSolverRHS(rhs);
    int nv = kkt->number_of_variables();
    VectorXd x(nv);
    rhs.supernodes->GatherInto(x);
    return x;
  };

  VectorXd x_c = recover_x(affine_c, s_c);
  VectorXd x_f = recover_x(affine_f, s_f);
  double x_diff = (x_c - x_f).lpNorm<Eigen::Infinity>();
  printf("  x diff: %.2e\n", x_diff);
  EXPECT_EQ(x_c.size(), x_f.size());
  EXPECT_LT(x_diff, 1e-12);
}

}  // namespace
}  // namespace conex
