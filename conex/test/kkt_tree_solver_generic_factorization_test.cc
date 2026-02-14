#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include "conex/kkt_tree_solver.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

class DenseLLTSubsystem final : public KKTSubsystem {
 public:
  DenseLLTSubsystem(const MatrixXd& supernode_block, const MatrixXd& separator_rows,
                    const MatrixXd& separator_schur)
      : supernode_block_(supernode_block),
        separator_rows_block_(separator_rows),
        separator_schur_block_(separator_schur) {}

 private:
  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    supernode_submatrix() = supernode_block_;
    separator_rows() = separator_rows_block_;
    separator_schur_complement() = separator_schur_block_;
  }

  bool DoEliminateSupernodeColumns() override {
    llt_.compute(supernode_submatrix().selfadjointView<Eigen::Lower>());
    return llt_.info() == Eigen::Success;
  }

  void DoComputeSeparatorSchurComplement() override {
    if (separator_rows().rows() == 0) {
      return;
    }
    MatrixXd temp = separator_rows().transpose();
    llt_.matrixL().solveInPlace(temp);
    separator_schur_complement().noalias() -= temp.transpose() * temp;
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    llt_.matrixL().solveInPlace(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    llt_.matrixL().transpose().solveInPlace(y);
  }

  MatrixXd supernode_block_;
  MatrixXd separator_rows_block_;
  MatrixXd separator_schur_block_;
  Eigen::LLT<MatrixXd> llt_;
};

class DiagonalLowRankSubsystem final : public KKTSubsystem {
 public:
  DiagonalLowRankSubsystem(const VectorXd& diag, const MatrixXd& low_rank_factor,
                           const MatrixXd& separator_rows,
                           const MatrixXd& separator_schur)
      : diag_(diag),
        low_rank_factor_(low_rank_factor),
        separator_rows_block_(separator_rows),
        separator_schur_block_(separator_schur) {}

 private:
  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    supernode_submatrix().setZero();
    supernode_submatrix().diagonal() = diag_;
    supernode_submatrix().selfadjointView<Eigen::Lower>().rankUpdate(
        low_rank_factor_);
    separator_rows() = separator_rows_block_;
    separator_schur_complement() = separator_schur_block_;
  }

  bool DoEliminateSupernodeColumns() override {
    dinv_ = diag_.cwiseInverse();
    const MatrixXd dinv_u = dinv_.asDiagonal() * low_rank_factor_;
    const MatrixXd middle =
        MatrixXd::Identity(low_rank_factor_.cols(), low_rank_factor_.cols()) +
        low_rank_factor_.transpose() * dinv_u;
    middle_inv_ = middle.ldlt().solve(
        MatrixXd::Identity(middle.rows(), middle.cols()));
    return true;
  }

  void DoComputeSeparatorSchurComplement() override {
    if (separator_rows().rows() == 0) {
      return;
    }
    MatrixXd temp = separator_rows().transpose();
    ApplyAInverseInPlace(temp);
    separator_schur_complement().noalias() -= separator_rows() * temp;
  }

  // Generic block factorization path: A = I * A.
  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> /*y*/) const override {}

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    ApplyAInverseInPlace(y);
  }

  void ApplyAInverseInPlace(Eigen::Ref<MatrixXd> y) const {
    // Woodbury inverse for A = D + U U^T:
    // A^{-1} = D^{-1} - D^{-1} U (I + U^T D^{-1} U)^{-1} U^T D^{-1}
    MatrixXd z = dinv_.asDiagonal() * y;
    const MatrixXd tmp = low_rank_factor_.transpose() * z;
    y.noalias() = z - (dinv_.asDiagonal() * low_rank_factor_) * (middle_inv_ * tmp);
  }

  VectorXd diag_;
  MatrixXd low_rank_factor_;
  MatrixXd separator_rows_block_;
  MatrixXd separator_schur_block_;
  VectorXd dinv_;
  MatrixXd middle_inv_;
};

struct BlockCase {
  VectorXd d;
  MatrixXd u;
  MatrixXd a11;
  MatrixXd a21;
  MatrixXd a22;
  MatrixXd kkt;
};

BlockCase MakeBlockCase(int n1, int n2, int rank, uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> normal(0.0, 1.0);

  BlockCase c;
  c.d.resize(n1);
  for (int i = 0; i < n1; ++i) {
    c.d(i) = 2.0 + 0.2 * static_cast<double>(i % 7);
  }

  c.u.resize(n1, rank);
  for (int i = 0; i < n1; ++i) {
    for (int j = 0; j < rank; ++j) {
      c.u(i, j) = 0.08 * normal(rng);
    }
  }

  c.a11 = c.d.asDiagonal();
  c.a11.selfadjointView<Eigen::Lower>().rankUpdate(c.u);

  c.a21.resize(n2, n1);
  for (int i = 0; i < n2; ++i) {
    for (int j = 0; j < n1; ++j) {
      c.a21(i, j) = 0.05 * normal(rng);
    }
  }

  MatrixXd r = MatrixXd::NullaryExpr(n2, n2, [&]() { return 0.1 * normal(rng); });
  c.a22 = 4.0 * MatrixXd::Identity(n2, n2) + r * r.transpose();

  c.kkt.resize(n1 + n2, n1 + n2);
  c.kkt << c.a11, c.a21.transpose(), c.a21, c.a22;
  return c;
}

TEST(KKTTreeSolver, GenericBlockFactorizationDiagonalLowRankFirstBlock) {
  const int n1 = 2;
  const int n2 = 2;

  VectorXd d(n1);
  d << 2.0, 3.0;
  MatrixXd u(n1, 1);
  u << 0.8, -0.3;

  MatrixXd a11 = d.asDiagonal();
  a11.selfadjointView<Eigen::Lower>().rankUpdate(u);

  MatrixXd a21(n2, n1);
  a21 << 0.4, -0.2, 0.1, 0.3;

  MatrixXd a22(n2, n2);
  a22 << 4.0, 0.2, 0.2, 5.0;

  MatrixXd kkt(n1 + n2, n1 + n2);
  kkt << a11, a21.transpose(), a21, a22;
  const Eigen::LDLT<MatrixXd> kkt_ldlt(kkt.selfadjointView<Eigen::Lower>());
  ASSERT_EQ(kkt_ldlt.info(), Eigen::Success);

  auto child = std::make_unique<DiagonalLowRankSubsystem>(
      d, u, a21, MatrixXd::Zero(n2, n2));
  child->SetSupernodes({0, 1});
  child->SetSeparators({2, 3});

  auto root = std::make_unique<DenseLLTSubsystem>(a22, MatrixXd(0, n2),
                                                  MatrixXd(0, 0));
  root->SetSupernodes({2, 3});
  root->SetSeparators({});

  SymmetricLinearSystemTreeSolver solver;
  solver.SetNumThreads(1);
  solver.AddSubsystem(child.get());
  solver.AddSubsystem(root.get());

  CliqueTree tree;
  tree.node_to_parent = {1, -1};
  tree.supernodes = {{0, 1}, {2, 3}};
  tree.separators = {{2, 3}, {}};
  solver.Finalize(tree);

  ASSERT_TRUE(solver.AssembleAndFactor());

  MatrixXd rhs(n1 + n2, 3);
  rhs << 1.0, -0.3, 0.2, -1.0, 0.1, 0.7, 0.4, 1.2, -0.5, -0.6, 0.8, 0.9;
  const MatrixXd ref = kkt_ldlt.solve(rhs);

  MatrixXd sol = rhs;
  solver.SolveInPlace(sol, false);
  EXPECT_NEAR((sol - ref).norm(), 0.0, 1e-9);
}

TEST(KKTTreeSolver, GenericBlockFactorizationDiagonalLowRankBenchmarkLarge) {
  const int n1 = 96;
  const int n2 = 96;
  const int rank = 6;
  const int rhs_cols = 8;
  const int repeats = 25;

  const BlockCase c = MakeBlockCase(n1, n2, rank, 12345);

  MatrixXd rhs = MatrixXd::Zero(n1 + n2, rhs_cols);
  for (int i = 0; i < rhs.rows(); ++i) {
    for (int j = 0; j < rhs.cols(); ++j) {
      rhs(i, j) = 0.1 * static_cast<double>((i + 3 * j) % 11 - 5);
    }
  }

  using Clock = std::chrono::steady_clock;
  double tree_ms = 0.0;
  double dense_ms = 0.0;
  double max_rel_err = 0.0;

  for (int k = 0; k < repeats; ++k) {
    std::vector<int> child_supernodes(n1);
    for (int i = 0; i < n1; ++i) {
      child_supernodes[i] = i;
    }
    std::vector<int> child_separators(n2);
    for (int i = 0; i < n2; ++i) {
      child_separators[i] = n1 + i;
    }
    auto child = std::make_unique<DiagonalLowRankSubsystem>(
        c.d, c.u, c.a21, MatrixXd::Zero(n2, n2));
    child->SetSupernodes(child_supernodes);
    child->SetSeparators(child_separators);

    auto root =
        std::make_unique<DenseLLTSubsystem>(c.a22, MatrixXd(0, n2), MatrixXd(0, 0));
    std::vector<int> root_supernodes(n2);
    for (int i = 0; i < n2; ++i) {
      root_supernodes[i] = n1 + i;
    }
    root->SetSupernodes(root_supernodes);
    root->SetSeparators({});

    SymmetricLinearSystemTreeSolver solver;
    solver.SetNumThreads(1);
    solver.AddSubsystem(child.get());
    solver.AddSubsystem(root.get());
    CliqueTree tree;
    tree.node_to_parent = {1, -1};
    tree.supernodes = {child_supernodes, root_supernodes};
    tree.separators = {child_separators, {}};
    solver.Finalize(tree);

    auto t0 = Clock::now();
    ASSERT_TRUE(solver.AssembleAndFactor());
    MatrixXd x_tree = rhs;
    solver.SolveInPlace(x_tree, false);
    auto t1 = Clock::now();
    tree_ms +=
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0)
            .count();

    auto d0 = Clock::now();
    Eigen::LDLT<MatrixXd> ldlt(c.kkt.selfadjointView<Eigen::Lower>());
    ASSERT_EQ(ldlt.info(), Eigen::Success);
    MatrixXd x_ref = ldlt.solve(rhs);
    auto d1 = Clock::now();
    dense_ms +=
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(d1 - d0)
            .count();

    const double rel_err =
        (x_tree - x_ref).norm() / std::max(1e-12, x_ref.norm());
    max_rel_err = std::max(max_rel_err, rel_err);
  }

  std::cout << "GenericFactorizationLarge n1=" << n1 << " n2=" << n2
            << " rank=" << rank << " repeats=" << repeats
            << " tree_ms_avg=" << (tree_ms / repeats)
            << " dense_ldlt_ms_avg=" << (dense_ms / repeats)
            << " speedup_dense_over_tree=" << (tree_ms / std::max(1e-12, dense_ms))
            << " max_rel_err=" << max_rel_err << std::endl;

  EXPECT_LT(max_rel_err, 1e-9);
}

}  // namespace
}  // namespace conex
