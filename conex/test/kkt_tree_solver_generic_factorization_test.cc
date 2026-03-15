#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include "conex/cholesky_solvers.h"
#include "conex/kkt_tree_solver.h"
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

}  // namespace
}  // namespace conex
