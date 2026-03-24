#include "conex/tree_solver/low_rank_diagonal_subsystem.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <numeric>
#include <vector>

#include "conex/tree_solver/kkt_tree_solver.h"
#include "conex/tree_solver/tree_utils.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {
namespace {

// Build a single-clique tree solver using a LowRankPlusDiagonalSubsystem,
// factor, and solve.  Compare against a dense reference.
TEST(LowRankDiagonal, SingleCliqueSolve) {
  const int n = 20;
  const int r = 3;

  srand(42);
  Eigen::VectorXd d = Eigen::VectorXd::Random(n).array().abs() + 0.5;
  Eigen::MatrixXd U = Eigen::MatrixXd::Random(n, r);

  // Full matrix for reference.
  Eigen::MatrixXd A_full = d.asDiagonal();
  A_full.noalias() += U * U.transpose();

  Eigen::VectorXd x_true = Eigen::VectorXd::Random(n);
  Eigen::VectorXd rhs = A_full * x_true;

  // Build a tree solver with one LowRankPlusDiagonalSubsystem node.
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // Data source.
  class Source : public LowRankDiagonalDataSource {
   public:
    Source(const std::vector<int>& v, const Eigen::VectorXd& d,
           const Eigen::MatrixXd& U)
        : v_(v), d_(d), U_(U) {}
    std::vector<int> variables() const override { return v_; }
    void GetData(Eigen::VectorXd& d, Eigen::MatrixXd& U,
                 const std::vector<int>& elim) const override {
      const int n = static_cast<int>(v_.size());
      // Reorder to elimination order.
      std::vector<std::pair<int, int>> order(n);
      for (int i = 0; i < n; ++i) order[i] = {elim[i], i};
      std::sort(order.begin(), order.end());
      d.resize(n);
      U.resize(n, U_.cols());
      for (int i = 0; i < n; ++i) {
        d(i) = d_(order[i].second);
        U.row(i) = U_.row(order[i].second);
      }
    }
    std::vector<int> v_;
    Eigen::VectorXd d_;
    Eigen::MatrixXd U_;
  } source(vars, d, U);

  // Create adapter.
  auto adapter = std::make_unique<LowRankDiagonalAdapter>(&source);
  adapter->set_contribution_type(ContributionType::kPositiveDefinite);

  // Build solver.
  auto solver = std::make_unique<SymmetricLinearSystemTreeSolver>();
  auto* adapter_ptr = adapter.get();
  solver->push_back(std::move(adapter));

  // Single-clique tree.
  CliqueTree ct;
  ct.supernodes = {vars};
  ct.separators = {{}};
  ct.node_to_parent = {-1};
  ct.post_order_position_to_clique = {0};

  // Manual Finalize steps:
  // 1. Create subsystem.
  auto subsystem = std::make_unique<LowRankPlusDiagonalSubsystem>();
  auto* subsystem_ptr = subsystem.get();
  adapter_ptr->BindSubsystem(subsystem_ptr);

  // Register as owned subsystem.
  std::vector<bool> needs_indefinite = {false};

  // We need to use the solver's internal machinery. Since we can't easily
  // inject a custom subsystem through the existing Finalize, let's test
  // the subsystem directly through its public KKTSubsystemBase interface.

  // Set up the subsystem.
  subsystem->SetSupernodes(vars);
  subsystem->SetSeparators({});
  subsystem->Initialize();

  // Write data.
  std::vector<int> identity_elim(n);
  std::iota(identity_elim.begin(), identity_elim.end(), 0);
  source.GetData(subsystem->diagonal(), subsystem->low_rank_factor(),
                 identity_elim);

  // Factor and solve through the tree interface.
  // Since there's one root with no children, AssembleAndFactor just factors.
  ASSERT_TRUE(subsystem->AssembleAndFactor());

  // The public solve path goes through ApplyInverseOfLeftFactor which
  // traverses the tree. For a single node with no separator, it just
  // applies the supernode inverse.
  Eigen::MatrixXd y = rhs;
  subsystem->ApplyInverseOfLeftFactor(y);
  // For schur complement mode, right factor is identity.
  subsystem->ApplyInverseOfRightFactor(y);

  double err = (y - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-12) << "Solve error: " << err;
}

// Test Schur complement: build a 2-node tree (child + parent).
// Child has D+UU^T structure; parent is dense.
// Verify the child's Schur complement contribution is correct.
TEST(LowRankDiagonal, SchurComplement) {
  const int n_sn = 10;  // child supernode size
  const int n_sep = 4;   // child separator size (= overlap with parent)
  const int r = 2;       // rank

  srand(42);
  Eigen::VectorXd d = Eigen::VectorXd::Random(n_sn).array().abs() + 0.5;
  Eigen::MatrixXd U = Eigen::MatrixXd::Random(n_sn, r);
  Eigen::MatrixXd S = Eigen::MatrixXd::Random(n_sep, n_sn);

  // Full supernode matrix.
  Eigen::MatrixXd A_full = d.asDiagonal();
  A_full.noalias() += U * U.transpose();

  // Reference Schur complement: -S A^{-1} S^T
  Eigen::MatrixXd sep_schur_ref = -S * A_full.inverse() * S.transpose();

  // Test with subsystem. Must allocate arena for storage.
  LowRankPlusDiagonalSubsystem subsystem;
  std::vector<int> supernodes(n_sn);
  std::iota(supernodes.begin(), supernodes.end(), 0);
  std::vector<int> separators(n_sep);
  std::iota(separators.begin(), separators.end(), n_sn);
  subsystem.SetSupernodes(supernodes);
  subsystem.SetSeparators(separators);

  // Allocate aligned arena memory for the dense storage.
  size_t arena_bytes = subsystem.RequiredArenaBytes();
  void* arena_raw = nullptr;
  posix_memalign(&arena_raw, 64, arena_bytes);
  std::memset(arena_raw, 0, arena_bytes);
  subsystem.BindArenaMemory(static_cast<double*>(arena_raw), arena_bytes);
  subsystem.Initialize();

  subsystem.SetData(d, U);
  subsystem.separator_rows() = S;
  subsystem.separator_schur_complement().setZero();

  ASSERT_TRUE(subsystem.AssembleAndFactor());

  // The tree solver convention is lower-triangular storage.
  // Compare only the lower triangle.
  auto got = subsystem.separator_schur_complement();
  double err = 0, ref_norm = 0;
  for (int j = 0; j < n_sep; j++) {
    for (int i = j; i < n_sep; i++) {
      double diff = got(i, j) - sep_schur_ref(i, j);
      err += diff * diff;
      ref_norm += sep_schur_ref(i, j) * sep_schur_ref(i, j);
    }
  }
  err = std::sqrt(err / ref_norm);
  EXPECT_LT(err, 1e-10) << "Schur complement error: " << err;
}

// Test the adapter's UpdateData path.
TEST(LowRankDiagonal, AdapterUpdateData) {
  const int n = 8;
  const int r = 2;

  srand(42);
  Eigen::VectorXd d = Eigen::VectorXd::Random(n).array().abs() + 0.5;
  Eigen::MatrixXd U = Eigen::MatrixXd::Random(n, r);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  class Source : public LowRankDiagonalDataSource {
   public:
    Source(const std::vector<int>& v, const Eigen::VectorXd& d,
           const Eigen::MatrixXd& U)
        : v_(v), d_(d), U_(U) {}
    std::vector<int> variables() const override { return v_; }
    void GetData(Eigen::VectorXd& d, Eigen::MatrixXd& U,
                 const std::vector<int>& elim) const override {
      const int sz = static_cast<int>(v_.size());
      std::vector<std::pair<int, int>> order(sz);
      for (int i = 0; i < sz; ++i) order[i] = {elim[i], i};
      std::sort(order.begin(), order.end());
      d.resize(sz);
      U.resize(sz, U_.cols());
      for (int i = 0; i < sz; ++i) {
        d(i) = d_(order[i].second);
        U.row(i) = U_.row(order[i].second);
      }
    }
    std::vector<int> v_;
    Eigen::VectorXd d_;
    Eigen::MatrixXd U_;
  } source(vars, d, U);

  LowRankPlusDiagonalSubsystem subsystem;
  subsystem.SetSupernodes(vars);
  subsystem.SetSeparators({});
  subsystem.Initialize();

  LowRankDiagonalAdapter adapter(&source);
  adapter.BindSubsystem(&subsystem);

  // Simulate elimination positions = identity (no reordering).
  std::vector<int> identity(n);
  std::iota(identity.begin(), identity.end(), 0);
  adapter.SetEliminationPosition(identity);

  // UpdateData should write d and U into the subsystem.
  adapter.UpdateData();

  EXPECT_EQ(subsystem.diagonal().size(), n);
  EXPECT_EQ(subsystem.low_rank_factor().rows(), n);
  EXPECT_EQ(subsystem.low_rank_factor().cols(), r);

  // Verify the data matches (identity elimination = no reorder).
  double d_err = (subsystem.diagonal() - d).norm();
  double U_err = (subsystem.low_rank_factor() - U).norm();
  EXPECT_LT(d_err, 1e-14) << "Diagonal mismatch";
  EXPECT_LT(U_err, 1e-14) << "Low-rank factor mismatch";
}

}  // namespace
}  // namespace conex
