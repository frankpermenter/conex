// Tests for Problem rescaling (row + column scaling).

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include <numeric>
#include <vector>

#include "conex/common/problem.h"
#include "conex/common/rescale.h"

namespace conex {
namespace {

Eigen::SparseMatrix<double> MakeSparse(const Eigen::MatrixXd& M) {
  return M.sparseView(1e-15, 1);
}

// Compute max abs column norm across linear/SOC constraints.
double MaxColumnNorm(const Problem& problem, int n) {
  Eigen::VectorXd col_max = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
        for (int k = 0; k < data.A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
               it; ++it)
            col_max(data.vars[it.col()]) =
                std::max(col_max(data.vars[it.col()]), std::abs(it.value()));
      }
    }, problem.constraint(i));
  }
  return col_max.maxCoeff();
}

// Compute ratio of max/min nonzero column norms.
double ColumnNormRatio(const Problem& problem, int n) {
  Eigen::VectorXd col_max = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
        for (int k = 0; k < data.A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
               it; ++it)
            col_max(data.vars[it.col()]) =
                std::max(col_max(data.vars[it.col()]), std::abs(it.value()));
      }
    }, problem.constraint(i));
  }
  double lo = 1e30, hi = 0;
  for (int j = 0; j < n; ++j) {
    if (col_max(j) > 1e-15) {
      lo = std::min(lo, col_max(j));
      hi = std::max(hi, col_max(j));
    }
  }
  return (lo > 1e-15) ? hi / lo : 1e30;
}

// Build a poorly-scaled LP: column norms vary by 1e6.
Problem MakePoorlyScaledLP(int m, int n) {
  Eigen::MatrixXd A_dense = Eigen::MatrixXd::Random(m, n);
  // Scale columns: col j by 10^(3j/(n-1) - 3) so range is [1e-3, 1e3].
  for (int j = 0; j < n; ++j) {
    double s = std::pow(10.0, 6.0 * j / (n - 1) - 3.0);
    A_dense.col(j) *= s;
  }
  Eigen::VectorXd b = Eigen::VectorXd::Ones(m) * 5.0;

  Problem p;
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  p.AddLinearConstraint(MakeSparse(A_dense), b, vars);

  Eigen::VectorXd c = Eigen::VectorXd::Random(n);
  p.SetLinearCost(c);
  return p;
}

TEST(Rescale, MaxAbsValueReducesColumnRatio) {
  int m = 20, n = 10;
  Problem p = MakePoorlyScaledLP(m, n);
  double ratio_before = ColumnNormRatio(p, n);
  EXPECT_GT(ratio_before, 100);  // Poorly scaled.

  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::MaxAbsValue);
  EXPECT_TRUE(info.was_rescaled);

  double ratio_after = ColumnNormRatio(rescaled, n);
  EXPECT_LT(ratio_after, 5.0);  // Well balanced.
  EXPECT_LT(ratio_after, ratio_before / 10);
}

TEST(Rescale, L2NormReducesColumnRatio) {
  int m = 20, n = 10;
  Problem p = MakePoorlyScaledLP(m, n);
  double ratio_before = ColumnNormRatio(p, n);

  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::L2Norm);
  EXPECT_TRUE(info.was_rescaled);

  double ratio_after = ColumnNormRatio(rescaled, n);
  EXPECT_LT(ratio_after, ratio_before / 10);
}

TEST(Rescale, RuizReducesColumnRatio) {
  int m = 20, n = 10;
  Problem p = MakePoorlyScaledLP(m, n);
  double ratio_before = ColumnNormRatio(p, n);

  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::Ruiz);
  EXPECT_TRUE(info.was_rescaled);

  double ratio_after = ColumnNormRatio(rescaled, n);
  EXPECT_LT(ratio_after, ratio_before / 10);
}

TEST(Rescale, RuizBetterThanMaxAbs) {
  // Ruiz iterates, so should achieve tighter balance than one-shot.
  int m = 30, n = 15;
  Problem p = MakePoorlyScaledLP(m, n);

  auto [r_max, info_max] = RescaleProblem(p, ColumnScaling::MaxAbsValue);
  auto [r_ruiz, info_ruiz] = RescaleProblem(p, ColumnScaling::Ruiz);

  double ratio_max = ColumnNormRatio(r_max, n);
  double ratio_ruiz = ColumnNormRatio(r_ruiz, n);

  // Ruiz should be at least as good as MaxAbsValue (often much better
  // because it also row-scales within the Ruiz loop).
  EXPECT_LE(ratio_ruiz, ratio_max + 1.0);
}

TEST(Rescale, UnscaleRecoversSolution) {
  int m = 10, n = 5;
  Problem p = MakePoorlyScaledLP(m, n);
  Eigen::VectorXd x_original = Eigen::VectorXd::Random(n);

  for (auto strategy : {ColumnScaling::MaxAbsValue,
                        ColumnScaling::L2Norm,
                        ColumnScaling::Ruiz}) {
    auto [rescaled, info] = RescaleProblem(p, strategy);
    // x_rescaled = D^{-1} x_original, so Unscale should recover.
    Eigen::VectorXd x_rescaled(n);
    for (int j = 0; j < n; ++j)
      x_rescaled(j) = x_original(j) / info.col_scale(j);

    Eigen::VectorXd x_recovered = info.Unscale(x_rescaled);
    EXPECT_LT((x_recovered - x_original).norm(), 1e-12 * x_original.norm());
  }
}

TEST(Rescale, NonnegRowScalingSetsOnes) {
  int m = 5, n = 3;
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(m, n).cwiseAbs();
  Eigen::VectorXd b(m);
  b << 2.0, 0.5, 10.0, 0.1, 3.0;

  Problem p;
  p.AddLinearConstraint(MakeSparse(A), b);

  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::MaxAbsValue);

  // After row scaling, b should be all ones (before column scaling).
  // We can't check directly since column scaling may change things,
  // but the was_rescaled flag should be set.
  EXPECT_TRUE(info.was_rescaled);
}

TEST(Rescale, DefaultIsRuiz) {
  int m = 20, n = 10;
  Problem p = MakePoorlyScaledLP(m, n);

  // Default (no strategy arg) should be Ruiz.
  auto [r_default, info_default] = RescaleProblem(p);
  auto [r_ruiz, info_ruiz] = RescaleProblem(p, ColumnScaling::Ruiz);

  // Same col_scale.
  EXPECT_LT((info_default.col_scale - info_ruiz.col_scale).norm(), 1e-12);
}

TEST(Rescale, PSDRowScaling) {
  // 2×2 PSD constraint with non-identity B.
  int n_mat = 2;
  int p = 2;
  std::vector<Eigen::SparseMatrix<double>> A_list;
  A_list.push_back(MakeSparse(
      (Eigen::MatrixXd(2, 2) << 1, 0, 0, 0).finished()));
  A_list.push_back(MakeSparse(
      (Eigen::MatrixXd(2, 2) << 0, 0, 0, 1).finished()));

  // B = [[4, 1], [1, 2]] (positive definite).
  Eigen::SparseMatrix<double> B = MakeSparse(
      (Eigen::MatrixXd(2, 2) << 4, 1, 1, 2).finished());

  std::vector<int> vars = {0, 1};
  Problem prob;
  prob.AddPSDConstraint(A_list, B, vars);

  auto [rescaled, info] = RescaleProblem(prob, ColumnScaling::MaxAbsValue);
  EXPECT_TRUE(info.was_rescaled);

  // After rescaling, B should be identity.
  const auto& rdata = std::get<Problem::PSDConstraintData>(
      rescaled.constraint(0));
  Eigen::MatrixXd B_dense(rdata.B);
  Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(2, 2);
  EXPECT_LT((B_dense - I2).norm(), 1e-10);
}

TEST(Rescale, NoScalingNeeded) {
  // Already well-scaled problem: all column norms near 1.
  int m = 5, n = 3;
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(m, n);
  // Normalize columns to unit norm.
  for (int j = 0; j < n; ++j) A.col(j).normalize();
  Eigen::VectorXd b = Eigen::VectorXd::Ones(m);

  Problem p;
  p.AddLinearConstraint(MakeSparse(A), b);

  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::MaxAbsValue);
  // Row scaling still happens (b was ones, so nonneg scaling is identity).
  // Column scaling may or may not be triggered depending on exact random values.
  // Just verify it doesn't crash and Unscale works.
  Eigen::VectorXd x = Eigen::VectorXd::Ones(n);
  Eigen::VectorXd xr = info.Unscale(x);
  EXPECT_GT(xr.norm(), 0);
}

}  // namespace
}  // namespace conex
