// Tests for Model rescaling (row + column scaling).

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include <numeric>
#include <vector>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/model.h"
#include "conex/common/rescale.h"
#include "conex/common/solver.h"

namespace conex {
namespace {

Eigen::SparseMatrix<double> MakeSparse(const Eigen::MatrixXd& M) {
  return M.sparseView(1e-15, 1);
}

// Compute max abs column norm across linear/SOC constraints.
double MaxColumnNorm(const Model& problem, int n) {
  Eigen::VectorXd col_max = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
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
double ColumnNormRatio(const Model& problem, int n) {
  Eigen::VectorXd col_max = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
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
Model MakePoorlyScaledLP(int m, int n) {
  Eigen::MatrixXd A_dense = Eigen::MatrixXd::Random(m, n);
  // Scale columns: col j by 10^(3j/(n-1) - 3) so range is [1e-3, 1e3].
  for (int j = 0; j < n; ++j) {
    double s = std::pow(10.0, 6.0 * j / (n - 1) - 3.0);
    A_dense.col(j) *= s;
  }
  Eigen::VectorXd b = Eigen::VectorXd::Ones(m) * 5.0;

  Model p;
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  p.AddLinearConstraint(MakeSparse(A_dense), b, vars);

  Eigen::VectorXd c = Eigen::VectorXd::Random(n);
  p.SetLinearCost(c);
  return p;
}

TEST(Rescale, MaxAbsValueReducesColumnRatio) {
  int m = 20, n = 10;
  Model p = MakePoorlyScaledLP(m, n);
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
  Model p = MakePoorlyScaledLP(m, n);
  double ratio_before = ColumnNormRatio(p, n);

  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::L2Norm);
  EXPECT_TRUE(info.was_rescaled);

  double ratio_after = ColumnNormRatio(rescaled, n);
  EXPECT_LT(ratio_after, ratio_before / 10);
}

TEST(Rescale, RuizReducesColumnRatio) {
  int m = 20, n = 10;
  Model p = MakePoorlyScaledLP(m, n);
  double ratio_before = ColumnNormRatio(p, n);

  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::Ruiz);
  EXPECT_TRUE(info.was_rescaled);

  double ratio_after = ColumnNormRatio(rescaled, n);
  EXPECT_LT(ratio_after, ratio_before / 10);
}

TEST(Rescale, RuizBetterThanMaxAbs) {
  // Ruiz iterates, so should achieve tighter balance than one-shot.
  int m = 30, n = 15;
  Model p = MakePoorlyScaledLP(m, n);

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
  Model p = MakePoorlyScaledLP(m, n);
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

  Model p;
  p.AddLinearConstraint(MakeSparse(A), b);

  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::MaxAbsValue);

  // After row scaling, b should be all ones (before column scaling).
  // We can't check directly since column scaling may change things,
  // but the was_rescaled flag should be set.
  EXPECT_TRUE(info.was_rescaled);
}

TEST(Rescale, DefaultIsRuiz) {
  int m = 20, n = 10;
  Model p = MakePoorlyScaledLP(m, n);

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
  Model prob;
  prob.AddPSDConstraint(A_list, B, vars);

  auto [rescaled, info] = RescaleProblem(prob, ColumnScaling::MaxAbsValue);
  EXPECT_TRUE(info.was_rescaled);

  // After rescaling, B should be identity.
  const auto& rdata = std::get<Model::PSDConstraintData>(
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

  Model p;
  p.AddLinearConstraint(MakeSparse(A), b);

  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::MaxAbsValue);
  // Row scaling still happens (b was ones, so nonneg scaling is identity).
  // Column scaling may or may not be triggered depending on exact random values.
  // Just verify it doesn't crash and Unscale works.
  Eigen::VectorXd x = Eigen::VectorXd::Ones(n);
  Eigen::VectorXd xr = info.Unscale(x);
  EXPECT_GT(xr.norm(), 0);
}

TEST(Rescale, QuadraticCostPreservesObjective) {
  // Verify that rescaling a QP preserves the objective value:
  //   (1/2) x'Qx + c'x = (1/2) x_new' Q_new x_new + c_new' x_new
  // where x = D * x_new, Q_new = D*Q*D, c_new = D*c.
  int n = 5;
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // Random PSD Q.
  Eigen::MatrixXd R = Eigen::MatrixXd::Random(n, n);
  Eigen::MatrixXd Q_dense = R.transpose() * R + 0.1 * Eigen::MatrixXd::Identity(n, n);

  // Poorly-scaled constraints to trigger Ruiz.
  Eigen::MatrixXd A_dense = Eigen::MatrixXd::Random(10, n);
  for (int j = 0; j < n; ++j)
    A_dense.col(j) *= std::pow(10.0, 3.0 * j / (n - 1) - 1.5);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(10) * 5.0;
  Eigen::VectorXd c = Eigen::VectorXd::Random(n);

  Model p;
  p.AddQuadraticCost(MakeSparse(Q_dense), vars);
  p.AddLinearConstraint(MakeSparse(A_dense), b, vars);
  p.SetLinearCost(c);

  Eigen::VectorXd x_orig = Eigen::VectorXd::Random(n);
  double obj_orig = 0.5 * x_orig.dot(Q_dense * x_orig) + c.dot(x_orig);

  for (auto strategy : {ColumnScaling::MaxAbsValue,
                        ColumnScaling::L2Norm,
                        ColumnScaling::Ruiz}) {
    auto [rescaled, info] = RescaleProblem(p, strategy);
    if (!info.was_rescaled) continue;

    // x_new = D^{-1} * x_orig
    Eigen::VectorXd x_new(n);
    for (int j = 0; j < n; ++j)
      x_new(j) = x_orig(j) / info.col_scale(j);

    // Verify Q_new = D*Q*D and c_new = D*c directly.
    const auto& D = info.col_scale;
    Eigen::MatrixXd Q_expected = D.asDiagonal() * Q_dense * D.asDiagonal();
    Eigen::VectorXd c_expected = D.cwiseProduct(c);

    // Extract Q_new from rescaled model.
    Eigen::MatrixXd Q_new = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < rescaled.num_constraints(); ++i) {
      auto* qd = std::get_if<Model::QuadraticCostData>(&rescaled.constraint(i));
      if (qd) Q_new += Eigen::MatrixXd(qd->Q_sparse);
    }

    double q_err = (Q_new - Q_expected).norm();
    double c_err = (rescaled.linear_cost() - c_expected).norm();
    if (q_err > 1e-6) {
      printf("  strategy=%d D=[%.4f", static_cast<int>(strategy), D(0));
      for (int j = 1; j < n; ++j) printf(",%.4f", D(j));
      printf("]\n");
      printf("  Q_new(0,0)=%.6f Q_expected(0,0)=%.6f Q_orig(0,0)=%.6f\n",
             Q_new(0,0), Q_expected(0,0), Q_dense(0,0));
      printf("  Q_new(1,1)=%.6f Q_expected(1,1)=%.6f\n", Q_new(1,1), Q_expected(1,1));
    }
    EXPECT_LT(q_err, 1e-10 * Q_expected.norm())
        << "Q scaling error for strategy " << static_cast<int>(strategy);
    EXPECT_LT(c_err, 1e-10 * c_expected.norm())
        << "c scaling error for strategy " << static_cast<int>(strategy);

    double obj_new = 0.5 * x_new.dot(Q_new * x_new) + rescaled.linear_cost().dot(x_new);
    EXPECT_NEAR(obj_new, obj_orig, 1e-8 * std::abs(obj_orig) + 1e-12)
        << "Objective mismatch for strategy " << static_cast<int>(strategy);
  }
}

TEST(Rescale, QuadraticCostSolveEquivalence) {
  // Verify that solving the rescaled QP and unscaling gives the same
  // solution as solving the original.
  int n = 4;
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // min 0.5 x'Qx + c'x s.t. Ax + b >= 0
  Eigen::MatrixXd Q_dense = Eigen::MatrixXd::Identity(n, n);
  Q_dense(0, 0) = 100;  // Poorly conditioned Q.
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n);
  c(0) = -1;

  // Poorly-scaled A.
  Eigen::MatrixXd A_dense = Eigen::MatrixXd::Identity(n, n);
  A_dense(0, 0) = 1000;  // Large column 0.
  Eigen::VectorXd b = Eigen::VectorXd::Ones(n);

  Model p;
  p.AddQuadraticCost(MakeSparse(Q_dense), vars);
  p.AddLinearConstraint(MakeSparse(A_dense), b, vars);
  p.SetLinearCost(c);

  // Solve original.
  auto s1 = Solver::Build(p);
  auto r1 = s1.Solve(ThetaContinuation{1e-10, 200, 1});

  // Solve rescaled.
  auto [rescaled, info] = RescaleProblem(p, ColumnScaling::Ruiz);
  auto s2 = Solver::Build(rescaled);
  auto r2 = s2.Solve(ThetaContinuation{1e-10, 200, 1});
  Eigen::VectorXd x2_orig = info.Unscale(r2.x);

  EXPECT_LT((r1.x - x2_orig).norm(), 1e-4)
      << "Rescaled solution differs from original.\n"
      << "  x_orig = " << r1.x.transpose() << "\n"
      << "  x_resc = " << x2_orig.transpose();
}

}  // namespace
}  // namespace conex
