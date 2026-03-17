#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include "conex/cholesky_solvers.h"
#include "conex/kkt_tree_solver.h"
#include "conex/static_subsystem.h"
#include "conex/workspace.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

class DenseLLTSubsystem final : public KKTSubsystem {
 public:
  DenseLLTSubsystem(const MatrixXd& supernode_block,
                    const MatrixXd& separator_rows,
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
  DiagonalLowRankSubsystem(const VectorXd& diag,
                           const MatrixXd& low_rank_factor,
                           const MatrixXd& separator_rows,
                           const MatrixXd& separator_schur)
      : diag_(diag),
        low_rank_factor_(low_rank_factor),
        separator_rows_block_(separator_rows),
        separator_schur_block_(separator_schur) {}

  bool used_generic_fallback() const { return use_generic_fallback_; }

 private:
  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    supernode_submatrix().setZero();
    supernode_submatrix().diagonal() = diag_;
    supernode_submatrix().selfadjointView<Eigen::Lower>().rankUpdate(
        low_rank_factor_);
    separator_rows() = separator_rows_block_;
    separator_schur_complement() = separator_schur_block_;
    use_generic_fallback_ = false;
  }

  bool DoEliminateSupernodeColumns() override {
    MatrixXd expected = diag_.asDiagonal();
    expected.selfadjointView<Eigen::Lower>().rankUpdate(low_rank_factor_);
    const double residual = (supernode_submatrix() - expected).norm() /
                            std::max(1e-12, supernode_submatrix().norm());
    use_generic_fallback_ = residual > 1e-12;
    if (use_generic_fallback_) {
      generic_ldlt_.compute(
          supernode_submatrix().selfadjointView<Eigen::Lower>());
      return generic_ldlt_.info() == Eigen::Success;
    }

    dinv_ = diag_.cwiseInverse();
    const MatrixXd dinv_u = dinv_.asDiagonal() * low_rank_factor_;
    const MatrixXd middle =
        MatrixXd::Identity(low_rank_factor_.cols(), low_rank_factor_.cols()) +
        low_rank_factor_.transpose() * dinv_u;
    middle_inv_ =
        middle.ldlt().solve(MatrixXd::Identity(middle.rows(), middle.cols()));
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
    if (use_generic_fallback_) {
      y = generic_ldlt_.solve(y);
      return;
    }
    // Woodbury inverse for A = D + U U^T:
    // A^{-1} = D^{-1} - D^{-1} U (I + U^T D^{-1} U)^{-1} U^T D^{-1}
    MatrixXd z = dinv_.asDiagonal() * y;
    const MatrixXd tmp = low_rank_factor_.transpose() * z;
    y.noalias() =
        z - (dinv_.asDiagonal() * low_rank_factor_) * (middle_inv_ * tmp);
  }

  VectorXd diag_;
  MatrixXd low_rank_factor_;
  MatrixXd separator_rows_block_;
  MatrixXd separator_schur_block_;
  VectorXd dinv_;
  MatrixXd middle_inv_;
  bool use_generic_fallback_ = false;
  Eigen::LDLT<MatrixXd> generic_ldlt_;
};

class InjectedSeparatorUpdateSubsystem final : public KKTSubsystem {
 public:
  explicit InjectedSeparatorUpdateSubsystem(const MatrixXd& separator_schur)
      : separator_schur_(separator_schur) {}

 private:
  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    supernode_submatrix().setZero();
    separator_rows().setZero();
    separator_schur_complement() = separator_schur_;
  }

  bool DoEliminateSupernodeColumns() override { return true; }

  void DoComputeSeparatorSchurComplement() override {}

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> /*y*/) const override {}

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> /*y*/) const override {}

  MatrixXd separator_schur_;
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

  MatrixXd r =
      MatrixXd::NullaryExpr(n2, n2, [&]() { return 0.1 * normal(rng); });
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

  auto root =
      std::make_unique<DenseLLTSubsystem>(a22, MatrixXd(0, n2), MatrixXd(0, 0));
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

    auto root = std::make_unique<DenseLLTSubsystem>(c.a22, MatrixXd(0, n2),
                                                    MatrixXd(0, 0));
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
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t1 - t0)
            .count();

    auto d0 = Clock::now();
    Eigen::LDLT<MatrixXd> ldlt(c.kkt.selfadjointView<Eigen::Lower>());
    ASSERT_EQ(ldlt.info(), Eigen::Success);
    MatrixXd x_ref = ldlt.solve(rhs);
    auto d1 = Clock::now();
    dense_ms +=
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            d1 - d0)
            .count();

    const double rel_err =
        (x_tree - x_ref).norm() / std::max(1e-12, x_ref.norm());
    max_rel_err = std::max(max_rel_err, rel_err);
  }

  std::cout << "GenericFactorizationLarge n1=" << n1 << " n2=" << n2
            << " rank=" << rank << " repeats=" << repeats
            << " tree_ms_avg=" << (tree_ms / repeats)
            << " dense_ldlt_ms_avg=" << (dense_ms / repeats)
            << " speedup_dense_over_tree="
            << (tree_ms / std::max(1e-12, dense_ms))
            << " max_rel_err=" << max_rel_err << std::endl;

  EXPECT_LT(max_rel_err, 1e-9);
}

TEST(KKTTreeSolver, GenericBlockFactorizationFallsBackWhenStructureDestroyed) {
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

  const MatrixXd injected =
      (MatrixXd(2, 2) << 0.25, 0.07, 0.07, 0.18).finished();

  MatrixXd kkt(n1 + n2, n1 + n2);
  kkt << (a11 + injected), a21.transpose(), a21, a22;
  Eigen::LDLT<MatrixXd> ref_ldlt(kkt.selfadjointView<Eigen::Lower>());
  ASSERT_EQ(ref_ldlt.info(), Eigen::Success);

  auto injected_child =
      std::make_unique<InjectedSeparatorUpdateSubsystem>(injected);
  injected_child->SetSupernodes({});
  injected_child->SetSeparators({0, 1});

  auto lowrank = std::make_unique<DiagonalLowRankSubsystem>(
      d, u, a21, MatrixXd::Zero(n2, n2));
  lowrank->SetSupernodes({0, 1});
  lowrank->SetSeparators({2, 3});

  auto root =
      std::make_unique<DenseLLTSubsystem>(a22, MatrixXd(0, n2), MatrixXd(0, 0));
  root->SetSupernodes({2, 3});
  root->SetSeparators({});

  SymmetricLinearSystemTreeSolver solver;
  solver.SetNumThreads(1);
  solver.AddSubsystem(injected_child.get());
  solver.AddSubsystem(lowrank.get());
  solver.AddSubsystem(root.get());

  CliqueTree tree;
  tree.node_to_parent = {1, 2, -1};
  tree.supernodes = {{}, {0, 1}, {2, 3}};
  tree.separators = {{0, 1}, {2, 3}, {}};
  solver.Finalize(tree);

  ASSERT_TRUE(solver.AssembleAndFactor());
  EXPECT_TRUE(lowrank->used_generic_fallback());

  MatrixXd rhs(4, 2);
  rhs << 1.0, -0.1, -0.2, 0.4, 0.7, -1.3, -0.5, 0.8;
  const MatrixXd ref = ref_ldlt.solve(rhs);
  MatrixXd sol = rhs;
  solver.SolveInPlace(sol, false);
  EXPECT_NEAR((sol - ref).norm(), 0.0, 1e-10);
}

TEST(KKTTreeSolver, SubmatrixContributorBasic) {
  // Build a 4x4 SPD matrix partitioned as:
  //   child: supernodes {0,1}, separators {2,3}
  //   root:  supernodes {2,3}, separators {}
  const int n1 = 2, n2 = 2;

  MatrixXd a11(n1, n1);
  a11 << 4.0, 0.5, 0.5, 3.0;
  MatrixXd a21(n2, n1);
  a21 << 0.3, -0.1, 0.2, 0.4;
  MatrixXd a22(n2, n2);
  a22 << 5.0, 0.1, 0.1, 6.0;

  MatrixXd kkt(4, 4);
  kkt << a11, a21.transpose(), a21, a22;
  Eigen::LDLT<MatrixXd> ref_ldlt(kkt.selfadjointView<Eigen::Lower>());
  ASSERT_EQ(ref_ldlt.info(), Eigen::Success);

  // Create plain KKTSubsystem nodes (no custom factorization).
  auto child = std::make_unique<DenseLLTSubsystem>(
      a11, a21, MatrixXd::Zero(n2, n2));
  child->SetSupernodes({0, 1});
  child->SetSeparators({2, 3});

  auto root = std::make_unique<DenseLLTSubsystem>(
      a22, MatrixXd(0, n2), MatrixXd(0, 0));
  root->SetSupernodes({2, 3});
  root->SetSeparators({});

  SymmetricLinearSystemTreeSolver solver;
  solver.AddSubsystem(child.get());
  solver.AddSubsystem(root.get());

  CliqueTree tree;
  tree.node_to_parent = {1, -1};
  tree.supernodes = {{0, 1}, {2, 3}};
  tree.separators = {{2, 3}, {}};
  solver.Finalize(tree);

  // Get a contributor for the child subsystem.
  auto contrib = solver.MakeContributor({0, 1, 2, 3});
  EXPECT_EQ(contrib.supernode_start(), 0);
  EXPECT_EQ(contrib.supernode_count(), 2);
  EXPECT_EQ(contrib.separator_indices().size(), 2u);
  EXPECT_EQ(contrib.separator_indices()[0], 2);
  EXPECT_EQ(contrib.separator_indices()[1], 3);

  // Verify we can write through the contributor and the data reaches the
  // subsystem storage (contributor refs alias the same memory).
  contrib.supernode_submatrix() = a11;
  contrib.separator_rows() = a21;
  contrib.separator_schur_complement().setZero();

  // Get a contributor for just the root's supernodes.
  auto root_contrib = solver.MakeContributor({2, 3});
  EXPECT_EQ(root_contrib.supernode_start(), 2);
  EXPECT_EQ(root_contrib.supernode_count(), 2);
  EXPECT_TRUE(root_contrib.separator_indices().empty());
  root_contrib.supernode_submatrix() = a22;

  ASSERT_TRUE(solver.AssembleAndFactor());

  MatrixXd rhs(4, 1);
  rhs << 1.0, -0.5, 0.3, 0.8;
  MatrixXd sol = rhs;
  solver.SolveInPlace(sol, false);
  MatrixXd ref = ref_ldlt.solve(rhs);
  EXPECT_NEAR((sol - ref).norm(), 0.0, 1e-10);
}

TEST(KKTTreeSolver, SubmatrixContributorWriteSymmetric) {
  // Replicate what StaticSubsystem does: provide Q in original variable order
  // and let the contributor permute it into the internal storage blocks.
  //
  // Tree: child has supernodes {0,1}, separators {2,3}
  //       root  has supernodes {2,3}, separators {}
  //
  // Q covers all 4 variables.
  const int n = 4;
  MatrixXd Q(n, n);
  Q << 4.0, 0.5, 0.3, 0.2,
       0.5, 3.0, -0.1, 0.4,
       0.3, -0.1, 5.0, 0.1,
       0.2, 0.4, 0.1, 6.0;

  MatrixXd kkt = Q.selfadjointView<Eigen::Lower>();
  Eigen::LDLT<MatrixXd> ref_ldlt(kkt);
  ASSERT_EQ(ref_ldlt.info(), Eigen::Success);

  // Use LLTSolver (arena-backed) — DoInitialize is a no-op for data,
  // so contributor-written data survives across AssembleAndFactor.
  auto child = std::make_unique<LLTSolver>();
  child->SetSupernodes({0, 1});
  child->SetSeparators({2, 3});

  auto root = std::make_unique<LLTSolver>();
  root->SetSupernodes({2, 3});
  root->SetSeparators({});

  SymmetricLinearSystemTreeSolver solver;
  solver.AddSubsystem(child.get());
  solver.AddSubsystem(root.get());

  CliqueTree tree;
  tree.node_to_parent = {1, -1};
  tree.supernodes = {{0, 1}, {2, 3}};
  tree.separators = {{2, 3}, {}};
  solver.Finalize(tree);

  // Zero all storage first (arena memory is not zeroed by allocator).
  auto root_contrib = solver.MakeContributor({2, 3});
  root_contrib.supernode_submatrix().setZero();

  // Write Q into the child subsystem via contributor.
  // The child's sep_schur starts with Q_{22}, then factorization subtracts
  // S * A^{-1} * S^T.  The result is scattered to the root's supernode.
  auto contrib = solver.MakeContributor({0, 1, 2, 3});
  contrib.supernode_submatrix().setZero();
  contrib.separator_rows().setZero();
  contrib.separator_schur_complement().setZero();
  contrib.WriteSymmetric(Q, {0, 1, 2, 3});

  ASSERT_TRUE(solver.AssembleAndFactor());

  VectorXd rhs(4);
  rhs << 1.0, -0.5, 0.3, 0.8;
  MatrixXd sol = rhs;
  solver.SolveInPlace(sol, false);
  VectorXd ref = ref_ldlt.solve(rhs);
  EXPECT_NEAR((sol - ref).norm(), 0.0, 1e-10);
}

TEST(KKTTreeSolver, SubmatrixContributorThrowsOnInvalidIndices) {
  auto child = std::make_unique<DenseLLTSubsystem>(
      MatrixXd::Identity(2, 2), MatrixXd::Zero(1, 2), MatrixXd::Zero(1, 1));
  child->SetSupernodes({0, 1});
  child->SetSeparators({2});

  auto root = std::make_unique<DenseLLTSubsystem>(
      MatrixXd::Identity(1, 1), MatrixXd(0, 1), MatrixXd(0, 0));
  root->SetSupernodes({2});
  root->SetSeparators({});

  SymmetricLinearSystemTreeSolver solver;
  solver.AddSubsystem(child.get());
  solver.AddSubsystem(root.get());

  CliqueTree tree;
  tree.node_to_parent = {1, -1};
  tree.supernodes = {{0, 1}, {2}};
  tree.separators = {{2}, {}};
  solver.Finalize(tree);

  // Index 3 doesn't exist in any subsystem.
  EXPECT_THROW(solver.MakeContributor({0, 3}), std::runtime_error);

  // Indices {0, 2} span child's supernodes and root's supernodes — child
  // contains both (0 as supernode, 2 as separator), so this should succeed.
  EXPECT_NO_THROW(solver.MakeContributor({0, 2}));
}

// Build an 8x8 block-tridiagonal SPD matrix with 2x2 blocks.
// The sparsity pattern matches a chain elimination tree:
//   {0,1} - {2,3} - {4,5} - {6,7}
// This matrix can be factored by any chain tree that progressively
// merges adjacent blocks.
MatrixXd MakeBlockTridiagonalSPD(uint64_t seed) {
  const int N = 8;
  const int bs = 2;
  const int nb = N / bs;
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  auto randn = [&](int r, int c) {
    MatrixXd M(r, c);
    for (int i = 0; i < r; ++i)
      for (int j = 0; j < c; ++j) M(i, j) = normal(rng);
    return M;
  };

  MatrixXd K = MatrixXd::Zero(N, N);
  for (int b = 0; b < nb; ++b) {
    int s = b * bs;
    MatrixXd R = 0.3 * randn(bs, bs);
    K.block(s, s, bs, bs) = 4.0 * MatrixXd::Identity(bs, bs) + R * R.transpose();
  }
  for (int b = 0; b + 1 < nb; ++b) {
    int s1 = b * bs, s2 = (b + 1) * bs;
    MatrixXd B = 0.2 * randn(bs, bs);
    K.block(s1, s2, bs, bs) = B;
    K.block(s2, s1, bs, bs) = B.transpose();
  }
  return K;
}

// Describes a chain elimination tree on N=8 variables with 2-variable blocks.
struct ChainTreeSpec {
  std::vector<std::vector<int>> supernodes;
  std::vector<std::vector<int>> separators;
  std::vector<int> parent;
};

// Build the chain tree specs for progressive merging from 4 nodes to 1.
std::vector<ChainTreeSpec> MakeProgressiveChainTrees() {
  return {
      // 4 nodes: {0,1} → {2,3} → {4,5} → {6,7}
      {{{0, 1}, {2, 3}, {4, 5}, {6, 7}},
       {{2, 3}, {4, 5}, {6, 7}, {}},
       {1, 2, 3, -1}},
      // 3 nodes: merge first two
      {{{0, 1, 2, 3}, {4, 5}, {6, 7}},
       {{4, 5}, {6, 7}, {}},
       {1, 2, -1}},
      // 2 nodes: merge first three
      {{{0, 1, 2, 3, 4, 5}, {6, 7}}, {{6, 7}, {}}, {1, -1}},
      // 1 node: fully dense
      {{{0, 1, 2, 3, 4, 5, 6, 7}}, {{}}, {-1}},
  };
}

// Write the block-tridiagonal matrix K into a tree solver via contributors.
//
// Convention for a chain tree:
//   Node 0 is always the leaf (no children feed into it). It gets the full
//   K[clique, clique] submatrix via WriteSymmetric.
//
//   Nodes 1..n-1 each receive a Schur complement from their child into their
//   supernode block, so we write zero for the supernode-supernode part and
//   K entries for the separator-related parts.
//
// If contribution_type is specified, each contributor is tagged with that type
// so the solver can select the appropriate factorization (LLT vs LU).
void WriteBlockTridiagonalToTree(
    const MatrixXd& K, const ChainTreeSpec& spec,
    SymmetricLinearSystemTreeSolver& solver,
    ContributionType type = ContributionType::kPositiveDefinite) {
  const int num_nodes = static_cast<int>(spec.supernodes.size());

  // Zero all blocks (arena memory is uninitialized).
  for (int ni = 0; ni < num_nodes; ++ni) {
    std::vector<int> clique;
    clique.insert(clique.end(), spec.supernodes[ni].begin(),
                  spec.supernodes[ni].end());
    clique.insert(clique.end(), spec.separators[ni].begin(),
                  spec.separators[ni].end());
    if (clique.empty()) clique = spec.supernodes[ni];
    auto c = solver.MakeContributor(clique);
    c.supernode_submatrix().setZero();
    if (!c.separator_indices().empty()) {
      c.separator_rows().setZero();
      c.separator_schur_complement().setZero();
    }
  }

  // Node 0 (leaf): write full K[clique, clique].
  {
    std::vector<int> clique;
    clique.insert(clique.end(), spec.supernodes[0].begin(),
                  spec.supernodes[0].end());
    clique.insert(clique.end(), spec.separators[0].begin(),
                  spec.separators[0].end());
    int cs = static_cast<int>(clique.size());
    MatrixXd Q(cs, cs);
    for (int i = 0; i < cs; ++i)
      for (int j = 0; j < cs; ++j) Q(i, j) = K(clique[i], clique[j]);
    auto contrib = solver.MakeContributor(clique);
    contrib.set_type(type);
    contrib.WriteSymmetric(Q, clique);
  }

  // Nodes 1..n-1: zero in supernode-supernode block, K entries elsewhere.
  for (int ni = 1; ni < num_nodes; ++ni) {
    std::vector<int> clique;
    clique.insert(clique.end(), spec.supernodes[ni].begin(),
                  spec.supernodes[ni].end());
    clique.insert(clique.end(), spec.separators[ni].begin(),
                  spec.separators[ni].end());
    if (clique.empty()) continue;

    int cs = static_cast<int>(clique.size());
    int sn_size = static_cast<int>(spec.supernodes[ni].size());
    MatrixXd Q = MatrixXd::Zero(cs, cs);
    for (int i = 0; i < cs; ++i)
      for (int j = 0; j < cs; ++j) {
        if (i < sn_size && j < sn_size) continue;
        Q(i, j) = K(clique[i], clique[j]);
      }
    auto contrib = solver.MakeContributor(clique);
    contrib.set_type(type);
    contrib.WriteSymmetric(Q, clique);
  }
}

TEST(KKTTreeSolver, ContributorProgressiveMerge) {
  const int N = 8;
  MatrixXd K = MakeBlockTridiagonalSPD(/*seed=*/42);

  // Reference solution.
  Eigen::LDLT<MatrixXd> ref_ldlt(K.selfadjointView<Eigen::Lower>());
  ASSERT_EQ(ref_ldlt.info(), Eigen::Success);

  MatrixXd rhs(N, 3);
  std::mt19937_64 rng(99);
  std::normal_distribution<double> normal(0.0, 1.0);
  for (int i = 0; i < rhs.rows(); ++i)
    for (int j = 0; j < rhs.cols(); ++j) rhs(i, j) = normal(rng);
  MatrixXd ref_sol = ref_ldlt.solve(rhs);

  auto trees = MakeProgressiveChainTrees();

  // Test both contribution types: PD selects LLT, indefinite selects LU.
  // The matrix is SPD so both paths should give the same answer.
  for (ContributionType ctype :
       {ContributionType::kPositiveDefinite, ContributionType::kIndefinite}) {
    for (size_t t = 0; t < trees.size(); ++t) {
      const auto& spec = trees[t];
      int num_nodes = static_cast<int>(spec.supernodes.size());

      // No explicit subsystem creation — solver auto-creates DynamicSubsystems.
      SymmetricLinearSystemTreeSolver solver;

      CliqueTree tree;
      tree.supernodes = spec.supernodes;
      tree.separators = spec.separators;
      tree.node_to_parent = spec.parent;
      solver.Finalize(tree);

      WriteBlockTridiagonalToTree(K, spec, solver, ctype);

      ASSERT_TRUE(solver.AssembleAndFactor())
          << "Factor failed for tree with " << num_nodes << " nodes"
          << " (type=" << static_cast<int>(ctype) << ")";

      MatrixXd sol = rhs;
      solver.SolveInPlace(sol, false);

      double rel_err = (sol - ref_sol).norm() / ref_sol.norm();
      EXPECT_LT(rel_err, 1e-10)
          << "Tree with " << num_nodes << " nodes"
          << " (type=" << static_cast<int>(ctype) << "): rel_err=" << rel_err;
    }
  }
}

// A simple assembler that holds a fixed symmetric matrix for its variables.
class StaticMatrixAssembler final : public SupernodalAssemblerBase {
 public:
  StaticMatrixAssembler(const std::vector<int>& variables,
                        const Eigen::MatrixXd& local_matrix)
      : SupernodalAssemblerBase(variables), local_matrix_(local_matrix) {
    CONEX_DEMAND(local_matrix_.rows() == local_matrix_.cols(),
                 "Local matrix must be square.");
    CONEX_DEMAND(local_matrix_.rows() == static_cast<int>(variables.size()),
                 "Local matrix size must match variables.");
    Workspace workspace(&submatrix_data_);
    memory_.resize(SizeOf(workspace));
    Initialize(&workspace, memory_.data());
  }

  void SetDenseData() override { submatrix_data_.G = local_matrix_; }

 private:
  Eigen::MatrixXd local_matrix_;
  Eigen::VectorXd memory_;
};

// Test that KKTAssemblerToSubsystemAdapter works with the contributor contract:
// the adapter does not call create_subsystem or AddSubsystem; instead it sets a
// contribution type and the tree solver auto-creates DynamicSubsystems and binds
// contributors after Finalize.
TEST(KKTTreeSolver, AdapterContributorContract) {
  // Two independent 2x2 SPD blocks: variables {0,1} and {2,3}.
  MatrixXd local_01(2, 2);
  local_01 << 4.0, 1.0, 1.0, 3.0;
  MatrixXd local_23(2, 2);
  local_23 << 5.0, 2.0, 2.0, 6.0;

  // Full 4x4 reference matrix.
  MatrixXd K = MatrixXd::Zero(4, 4);
  K.topLeftCorner(2, 2) = local_01;
  K.bottomRightCorner(2, 2) = local_23;

  Eigen::LDLT<MatrixXd> ref_ldlt(K);
  ASSERT_EQ(ref_ldlt.info(), Eigen::Success);

  VectorXd rhs(4);
  rhs << 1.0, -2.0, 3.0, 4.0;
  VectorXd ref_sol = ref_ldlt.solve(rhs);

  for (ContributionType ctype :
       {ContributionType::kPositiveDefinite, ContributionType::kIndefinite}) {
    std::vector<std::unique_ptr<StaticMatrixAssembler>> assemblers;
    SymmetricLinearSystemTreeSolver solver;
    solver.EnableAutoUpdateAtAssemble(true);

    // Create adapters using the contributor contract (no create_subsystem).
    assemblers.push_back(
        std::make_unique<StaticMatrixAssembler>(std::vector<int>{0, 1}, local_01));
    {
      auto adapter = std::make_unique<KKTAssemblerToSubsystemAdapter>(
          assemblers.back().get());
      adapter->set_contribution_type(ctype);
      solver.push_back(std::move(adapter));
    }

    assemblers.push_back(
        std::make_unique<StaticMatrixAssembler>(std::vector<int>{2, 3}, local_23));
    {
      auto adapter = std::make_unique<KKTAssemblerToSubsystemAdapter>(
          assemblers.back().get());
      adapter->set_contribution_type(ctype);
      solver.push_back(std::move(adapter));
    }

    CliqueTree tree;
    tree.supernodes = {{0, 1}, {2, 3}};
    tree.separators = {{}, {}};
    tree.node_to_parent = {-1, -1};
    solver.Finalize(tree);

    solver.Assemble();
    ASSERT_TRUE(solver.Factor())
        << "Factor failed (type=" << static_cast<int>(ctype) << ")";

    MatrixXd rhs_mat(4, 1);
    rhs_mat.col(0) = rhs;
    MatrixXd sol = solver.Solve(rhs_mat, true);

    double rel_err = (sol.col(0) - ref_sol).norm() / ref_sol.norm();
    EXPECT_LT(rel_err, 1e-10)
        << "type=" << static_cast<int>(ctype) << " rel_err=" << rel_err;
  }
}

// Test the adapter contributor contract with overlapping cliques (chain tree).
// Each clique's local matrix is an additive contribution; their sum is K.
TEST(KKTTreeSolver, AdapterContributorChainTree) {
  const int N = 8;
  std::mt19937_64 rng(77);
  std::normal_distribution<double> normal(0.0, 1.0);

  // 3 overlapping cliques: {0,1,2,3}, {2,3,4,5}, {4,5,6,7}.
  std::vector<std::vector<int>> cliques = {
      {0, 1, 2, 3}, {2, 3, 4, 5}, {4, 5, 6, 7}};

  // Build random SPD local matrices for each clique; K = sum of contributions.
  std::vector<MatrixXd> locals;
  MatrixXd K = MatrixXd::Zero(N, N);
  for (const auto& clique : cliques) {
    int cs = static_cast<int>(clique.size());
    MatrixXd R(cs, cs);
    for (int i = 0; i < cs; ++i)
      for (int j = 0; j < cs; ++j) R(i, j) = 0.3 * normal(rng);
    MatrixXd Q = 4.0 * MatrixXd::Identity(cs, cs) + R * R.transpose();
    locals.push_back(Q);
    for (int i = 0; i < cs; ++i)
      for (int j = 0; j < cs; ++j) K(clique[i], clique[j]) += Q(i, j);
  }

  Eigen::LDLT<MatrixXd> ref_ldlt(K);
  ASSERT_EQ(ref_ldlt.info(), Eigen::Success);

  MatrixXd rhs(N, 2);
  for (int i = 0; i < rhs.rows(); ++i)
    for (int j = 0; j < rhs.cols(); ++j) rhs(i, j) = normal(rng);
  MatrixXd ref_sol = ref_ldlt.solve(rhs);

  // Tree: node 0 sn={0,1} sep={2,3}, node 1 sn={2,3} sep={4,5},
  //       node 2 sn={4,5} sep={6,7}, node 3 sn={6,7} sep={}.
  CliqueTree tree;
  tree.supernodes = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};
  tree.separators = {{2, 3}, {4, 5}, {6, 7}, {}};
  tree.node_to_parent = {1, 2, 3, -1};

  for (ContributionType ctype :
       {ContributionType::kPositiveDefinite, ContributionType::kIndefinite}) {
    std::vector<std::unique_ptr<StaticMatrixAssembler>> assemblers;
    SymmetricLinearSystemTreeSolver solver;
    solver.EnableAutoUpdateAtAssemble(true);

    for (size_t i = 0; i < cliques.size(); ++i) {
      assemblers.push_back(
          std::make_unique<StaticMatrixAssembler>(cliques[i], locals[i]));
      auto adapter = std::make_unique<KKTAssemblerToSubsystemAdapter>(
          assemblers.back().get());
      adapter->set_contribution_type(ctype);
      solver.push_back(std::move(adapter));
    }

    solver.Finalize(tree);

    solver.Assemble();
    ASSERT_TRUE(solver.Factor())
        << "Factor failed (type=" << static_cast<int>(ctype) << ")";

    MatrixXd sol = solver.Solve(rhs, true);

    double rel_err = (sol - ref_sol).norm() / ref_sol.norm();
    EXPECT_LT(rel_err, 1e-10)
        << "type=" << static_cast<int>(ctype) << " rel_err=" << rel_err;
  }
}

// Lazy evaluator for A^T A.  Holds columns of A and computes sub-blocks of
// the Gram matrix on demand.
class LazyGramMatrix {
 public:
  explicit LazyGramMatrix(const Eigen::MatrixXd& A) : A_(A) {}

  void set_order(const std::vector<int>& perm) {
    Eigen::MatrixXd Ap(A_.rows(), A_.cols());
    for (int i = 0; i < static_cast<int>(perm.size()); ++i) {
      Ap.col(i) = A_.col(perm[i]);
    }
    A_ = std::move(Ap);
    set_order_called_ = true;
    applied_perm_ = perm;
  }

  void add_block(int row, int col, int rows, int cols,
                 Eigen::Ref<Eigen::MatrixXd> dest) const {
    dest.noalias() +=
        A_.middleCols(row, rows).transpose() * A_.middleCols(col, cols);
  }

  void add_block_lower(int pos, int size,
                       Eigen::Ref<Eigen::MatrixXd> dest) const {
    dest.selfadjointView<Eigen::Lower>().rankUpdate(
        A_.middleCols(pos, size).transpose());
  }

  int rows() const { return static_cast<int>(A_.cols()); }
  int cols() const { return static_cast<int>(A_.cols()); }

  bool set_order_called() const { return set_order_called_; }
  const std::vector<int>& applied_perm() const { return applied_perm_; }

 private:
  Eigen::MatrixXd A_;
  bool set_order_called_ = false;
  std::vector<int> applied_perm_;
};

// Test WriteSymmetricLazy with a LazyGramMatrix to solve A^T A x = b.
//
// 10 groups of columns with sizes 3,4,5,6,3,4,5,6,3,4 (total n=43).
// Each group has non-overlapping row supports, so P^T A^T A P is block
// diagonal.  Columns are round-robin interleaved across groups so that the
// natural column order mixes supernodes and separators.
//
// Groups are paired into 5 two-node subtrees: (0,1), (2,3), ..., (8,9).
// Each leaf node's contributor covers both groups in its pair, with the
// first group as supernodes and the second as separators.  The interleaved
// column order forces set_order to apply a non-trivial permutation that
// gathers each group's columns into contiguous runs.
TEST(KKTTreeSolver, LazyGramContributorSolve) {
  std::mt19937_64 rng(42);
  std::normal_distribution<double> normal(0.0, 1.0);

  // 10 groups with sizes 3..6 cycling.
  const std::vector<int> group_sizes = {3, 4, 5, 6, 3, 4, 5, 6, 3, 4};
  const int num_groups = static_cast<int>(group_sizes.size());
  const int num_pairs = num_groups / 2;
  int n = 0;
  for (int gs : group_sizes) n += gs;
  // Each group gets its own block of rows (non-overlapping supports).
  const int rows_per_group = 8;
  const int m = num_groups * rows_per_group;

  // Assign each variable to a group.  Round-robin: variable k belongs to
  // group (k % num_groups).  Track which columns belong to each group.
  std::vector<std::vector<int>> group_cols(num_groups);
  {
    std::vector<int> group_fill(num_groups, 0);
    int col = 0;
    while (col < n) {
      for (int g = 0; g < num_groups && col < n; ++g) {
        if (group_fill[g] < group_sizes[g]) {
          group_cols[g].push_back(col);
          group_fill[g]++;
          col++;
        }
      }
    }
  }

  // Build A (m x n): each group's columns are nonzero only in that group's
  // row block.
  MatrixXd A = MatrixXd::Zero(m, n);
  for (int g = 0; g < num_groups; ++g) {
    int row_start = g * rows_per_group;
    for (int c : group_cols[g]) {
      for (int r = row_start; r < row_start + rows_per_group; ++r) {
        A(r, c) = normal(rng);
      }
    }
  }
  MatrixXd AtA = A.transpose() * A;

  // Verify block-diagonal zero structure.
  for (int g1 = 0; g1 < num_groups; ++g1) {
    for (int g2 = g1 + 1; g2 < num_groups; ++g2) {
      for (int c1 : group_cols[g1]) {
        for (int c2 : group_cols[g2]) {
          ASSERT_NEAR(AtA(c1, c2), 0.0, 1e-14)
              << "Expected zero between group " << g1 << " col " << c1
              << " and group " << g2 << " col " << c2;
        }
      }
    }
  }

  Eigen::LDLT<MatrixXd> ref_ldlt(AtA);
  ASSERT_EQ(ref_ldlt.info(), Eigen::Success);
  VectorXd rhs(n);
  for (int i = 0; i < n; ++i) rhs(i) = normal(rng);
  VectorXd ref_sol = ref_ldlt.solve(rhs);

  // Build a forest of 5 two-node subtrees.  Each pair (2k, 2k+1) gives:
  //   Node 2k (leaf): sn = group 2k elim positions, sep = group 2k+1 elim positions
  //   Node 2k+1 (root): sn = group 2k+1 elim positions, sep = {}
  // Elimination positions are contiguous per group.
  CliqueTree tree;
  tree.supernodes.resize(num_groups);
  tree.separators.resize(num_groups);
  tree.node_to_parent.resize(num_groups);

  int elim_start = 0;
  for (int g = 0; g < num_groups; ++g) {
    for (int j = 0; j < group_sizes[g]; ++j) {
      tree.supernodes[g].push_back(elim_start + j);
    }
    elim_start += group_sizes[g];
  }
  for (int p = 0; p < num_pairs; ++p) {
    int leaf = 2 * p;
    int root = 2 * p + 1;
    // Leaf's separator = root's supernodes.
    tree.separators[leaf] = tree.supernodes[root];
    tree.separators[root] = {};  // root has no separator
    tree.node_to_parent[leaf] = root;
    tree.node_to_parent[root] = -1;
  }

  // Build the mapping from original column order to elimination position.
  std::vector<int> col_to_elim(n);
  {
    std::vector<int> next_elim(num_groups);
    int es = 0;
    for (int g = 0; g < num_groups; ++g) {
      next_elim[g] = es;
      es += group_sizes[g];
    }
    for (int g = 0; g < num_groups; ++g) {
      for (int c : group_cols[g]) {
        col_to_elim[c] = next_elim[g]++;
      }
    }
  }

  // Permuted A^T A for verification.
  std::vector<int> elim_to_col(n);
  for (int c = 0; c < n; ++c) elim_to_col[col_to_elim[c]] = c;
  MatrixXd AtA_perm(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      AtA_perm(i, j) = AtA(elim_to_col[i], elim_to_col[j]);

  SymmetricLinearSystemTreeSolver solver;
  solver.Finalize(tree);

  // Zero all storage.
  for (int g = 0; g < num_groups; ++g) {
    std::vector<int> clique;
    clique.insert(clique.end(), tree.supernodes[g].begin(),
                  tree.supernodes[g].end());
    clique.insert(clique.end(), tree.separators[g].begin(),
                  tree.separators[g].end());
    auto c = solver.MakeContributor(clique);
    c.supernode_submatrix().setZero();
    if (c.separator_rows().size() > 0) c.separator_rows().setZero();
    if (c.separator_schur_complement().size() > 0)
      c.separator_schur_complement().setZero();
  }

  // Write each pair's A^T A through the leaf node using WriteSymmetricLazy.
  // The lazy matrix covers both groups in the pair; columns are interleaved,
  // so set_order must reorder them (supernodes first, then separators).
  int set_order_count = 0;
  for (int p = 0; p < num_pairs; ++p) {
    int g0 = 2 * p;
    int g1 = 2 * p + 1;
    int leaf = g0;

    // Merge both groups' columns in interleaved original order.
    std::vector<int> pair_cols;
    pair_cols.insert(pair_cols.end(), group_cols[g0].begin(),
                     group_cols[g0].end());
    pair_cols.insert(pair_cols.end(), group_cols[g1].begin(),
                     group_cols[g1].end());
    // Sort by original column index to get the interleaved order.
    std::sort(pair_cols.begin(), pair_cols.end());

    int pair_n = static_cast<int>(pair_cols.size());
    MatrixXd A_pair(m, pair_n);
    std::vector<int> elim_pos(pair_n);
    for (int j = 0; j < pair_n; ++j) {
      A_pair.col(j) = A.col(pair_cols[j]);
      elim_pos[j] = col_to_elim[pair_cols[j]];
    }

    LazyGramMatrix lazy(A_pair);
    std::vector<int> clique;
    clique.insert(clique.end(), tree.supernodes[leaf].begin(),
                  tree.supernodes[leaf].end());
    clique.insert(clique.end(), tree.separators[leaf].begin(),
                  tree.separators[leaf].end());
    auto contrib = solver.MakeContributor(clique);
    contrib.WriteSymmetricLazy(lazy, elim_pos);

    if (lazy.set_order_called()) set_order_count++;

    // Verify supernode block for group g0.
    int es0 = 0;
    for (int k = 0; k < g0; ++k) es0 += group_sizes[k];
    int gs0 = group_sizes[g0];
    MatrixXd expected_sn =
        AtA_perm.block(es0, es0, gs0, gs0);
    MatrixXd actual_sn = contrib.supernode_submatrix();
    for (int i = 0; i < gs0; ++i)
      for (int j = i + 1; j < gs0; ++j) actual_sn(i, j) = actual_sn(j, i);
    EXPECT_NEAR((actual_sn - expected_sn).norm(), 0.0, 1e-10)
        << "Supernode block mismatch for pair " << p;

    // Verify separator Schur complement for group g1.
    int es1 = es0 + gs0;
    int gs1 = group_sizes[g1];
    MatrixXd expected_sep_sc =
        AtA_perm.block(es1, es1, gs1, gs1);
    MatrixXd actual_sep_sc = contrib.separator_schur_complement();
    for (int i = 0; i < gs1; ++i)
      for (int j = i + 1; j < gs1; ++j)
        actual_sep_sc(i, j) = actual_sep_sc(j, i);
    EXPECT_NEAR((actual_sep_sc - expected_sep_sc).norm(), 0.0, 1e-10)
        << "Separator Schur complement mismatch for pair " << p;

    // Separator rows should be zero (block diagonal, no cross-group terms).
    if (contrib.separator_rows().size() > 0) {
      EXPECT_NEAR(contrib.separator_rows().norm(), 0.0, 1e-10)
          << "Separator rows should be zero for pair " << p;
    }
  }

  // All 5 pairs should trigger set_order (interleaved columns).
  EXPECT_EQ(set_order_count, num_pairs);

  // Solve and verify.  The solver's variable i corresponds to original
  // column elim_to_col[i], so permute the RHS into solver order.
  ASSERT_TRUE(solver.AssembleAndFactor());
  VectorXd rhs_perm(n);
  for (int i = 0; i < n; ++i) rhs_perm(i) = rhs(elim_to_col[i]);
  MatrixXd sol(n, 1);
  sol.col(0) = rhs_perm;
  solver.SolveInPlace(sol, false);
  // Permute solution back to original order.
  VectorXd x_orig(n);
  for (int i = 0; i < n; ++i) x_orig(elim_to_col[i]) = sol(i, 0);
  double rel_err = (x_orig - ref_sol).norm() / ref_sol.norm();
  EXPECT_LT(rel_err, 1e-10) << "rel_err=" << rel_err;
}

}  // namespace
}  // namespace conex
