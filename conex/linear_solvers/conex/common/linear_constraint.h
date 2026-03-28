#pragma once
#include <unordered_map>

#include "conex/common/arena_allocatable.h"
#include "conex/common/constraint.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/supernodal_assembler_base.h"
#include "conex/common/linear_workspace.h"

namespace conex {

class GramEvaluator : public BlockAssembler {
 public:
  GramEvaluator() = default;
  void bind(WorkspaceLinear* ws, const Eigen::MatrixXd* A) {
    ws_ = ws;
    A_ = A;
  }

  void set_precompute_gram(bool v) { precompute_gram_ = v; }

  // Called once: permute columns of A.  Does NOT compute WA_perm_ —
  // that is deferred to ensure_weights_fresh() which runs at assembly
  // time with the current weights.
  void set_order(const std::vector<int>& perm) override {
    if (order_set_) return;
    const int n = A_->rows();
    const int m = static_cast<int>(perm.size());
    A_perm_.resize(n, m);
    for (int i = 0; i < m; ++i) {
      A_perm_.col(i) = A_->col(perm[i]);
    }
    WA_perm_.resize(n, m);
    weights_dirty_ = true;
    order_set_ = true;
  }

  // Recompute WA_perm_ = diag(W) * A_perm_ (and G_ if precomputing).
  // Called by SetWeights on the LinearConstraint, or by ensure_weights_fresh.
  void update_weights() {
    WA_perm_.noalias() = ws_->W.asDiagonal() * A_perm_;
    if (precompute_gram_) {
      G_.noalias() = WA_perm_.transpose() * WA_perm_;
    }
    weights_dirty_ = false;
  }

  void add_block(int row, int col, int rows, int cols,
                 Eigen::Ref<Eigen::MatrixXd> dest) const override {
    const_cast<GramEvaluator*>(this)->ensure_weights_fresh();
    if (precompute_gram_) {
      dest.noalias() += G_.block(row, col, rows, cols);
    } else {
      dest.noalias() +=
          WA_perm_.middleCols(row, rows).transpose() *
          WA_perm_.middleCols(col, cols);
    }
  }

  void add_block_lower(int pos, int size,
                       Eigen::Ref<Eigen::MatrixXd> dest) const override {
    const_cast<GramEvaluator*>(this)->ensure_weights_fresh();
    if (precompute_gram_) {
      const auto src = G_.block(pos, pos, size, size);
      for (int j = 0; j < size; ++j) {
        dest.col(j).tail(size - j) += src.col(j).tail(size - j);
      }
    } else {
      dest.selfadjointView<Eigen::Lower>().rankUpdate(
          WA_perm_.middleCols(pos, size).transpose());
    }
  }

  int rows() const override { return ws_->num_vars_; }
  int cols() const override { return ws_->num_vars_; }

  bool is_active() const { return order_set_; }
  void invalidate_order() { order_set_ = false; }

  // Number of supernode columns in A_perm_ (first sn_count_ cols).
  int sn_count() const { return sn_count_; }
  void set_sn_count(int c) override { sn_count_ = c; }

  // Compute r = A_perm_(:, 0:sn) * x_sn + A_perm_(:, sn:end) * x_sep - b.
  Eigen::VectorXd ComputeBlockResidual(
      Eigen::Ref<const Eigen::MatrixXd> x_sn,
      Eigen::Ref<const Eigen::MatrixXd> x_sep,
      const Eigen::MatrixXd& b) const {
    const int m = A_perm_.rows();
    const int ns = sn_count_;
    Eigen::VectorXd r(m);
    r.noalias() = A_perm_.leftCols(ns) * x_sn;
    if (A_perm_.cols() > ns) {
      r.noalias() += A_perm_.rightCols(A_perm_.cols() - ns) * x_sep;
    }
    r -= b;
    return r;
  }

  // --- Two-phase protocol ---

  bool RegisterContributions(
      int clique_id, const std::vector<int>& perm,
      const std::vector<BlockContribution>& blocks) override {
    if (!order_set_) set_order(perm);
    registered_blocks_[clique_id] = blocks;
    return true;
  }

  void ContributeBlocks(int clique_id) override {
    ensure_weights_fresh();

    auto it = registered_blocks_.find(clique_id);
    if (it == registered_blocks_.end()) return;

    for (const auto& bc : it->second) {
      using StrideType = Eigen::Stride<Eigen::Dynamic, 1>;
      Eigen::Map<Eigen::MatrixXd, 0, StrideType> dest_strided(
          bc.dest, bc.rows, bc.cols, StrideType(bc.dest_ld, 1));

      if (bc.lower_only) {
        dest_strided.selfadjointView<Eigen::Lower>().rankUpdate(
            WA_perm_.middleCols(bc.q_row, bc.rows).transpose());
      } else {
        dest_strided.noalias() +=
            WA_perm_.middleCols(bc.q_row, bc.rows).transpose() *
            WA_perm_.middleCols(bc.q_col, bc.cols);
      }

    }
  }

 private:
  void ensure_weights_fresh() {
    if (weights_dirty_) update_weights();
  }

  WorkspaceLinear* ws_ = nullptr;
  const Eigen::MatrixXd* A_ = nullptr;
  Eigen::MatrixXd A_perm_;
  Eigen::MatrixXd WA_perm_;
  Eigen::MatrixXd G_;
  bool order_set_ = false;
  bool weights_dirty_ = true;
  bool precompute_gram_ = false;
  int sn_count_ = 0;
  std::unordered_map<int, std::vector<BlockContribution>> registered_blocks_;
};

class LinearConstraint : public Constraint, public ArenaAllocatable {
 public:
  LinearConstraint(const Eigen::MatrixXd& constraint_matrix,
                   const Eigen::MatrixXd& constraint_affine);

  WorkspaceLinear* workspace() { return &workspace_; }

  int number_of_variables() const override { return constraint_matrix_.cols(); }
  BlockAssembler* GetBlockAssembler() override {
    gram_evaluator_.bind(&workspace_, &constraint_matrix_);
    return &gram_evaluator_;
  }
  void set_precompute_gram(bool v) { gram_evaluator_.set_precompute_gram(v); }
  Eigen::MatrixXd constraint_matrix() const { return constraint_matrix_; }
  Eigen::MatrixXd affine_term() const { return constraint_affine_; }

  // Set per-row weights and update the Gram evaluator.
  // weights must have size == number of rows (constraint_matrix_.rows()).
  // The Gram evaluator computes (WA)^T(WA) = A^T W^2 A, so W stores
  // sqrt(weight).  This method takes the actual weights and applies sqrt.
  void SetWeights(const Eigen::VectorXd& weights) {
    CONEX_DEMAND(weights.size() == constraint_matrix_.rows(),
                 "Weight vector size must match number of constraint rows.");
    workspace_.W = weights.array().sqrt().matrix();
    gram_evaluator_.update_weights();
  }

  // Set a single row weight (takes actual weight, applies sqrt internally).
  void SetWeight(int row, double w) {
    workspace_.W(row) = std::sqrt(w);
    gram_evaluator_.update_weights();
  }

  int num_rows() const { return constraint_matrix_.rows(); }

  // Compute residual r = A * x_local - b for this constraint's variables.
  // x_local must have size == number_of_variables().
  Eigen::VectorXd ComputeResidual(
      Eigen::Ref<const Eigen::VectorXd> x_local) const {
    return constraint_matrix_ * x_local - constraint_affine_;
  }

  // Compute residual using the elimination-ordered A_perm_ directly.
  // x_sn = supernode block values (contiguous, size = sn_count)
  // x_sep = separator block values (contiguous, size = num_vars - sn_count)
  // Requires set_order() to have been called (via the lazy evaluator path).
  Eigen::VectorXd ComputeBlockResidual(
      Eigen::Ref<const Eigen::MatrixXd> x_sn,
      Eigen::Ref<const Eigen::MatrixXd> x_sep) const {
    return gram_evaluator_.ComputeBlockResidual(x_sn, x_sep, constraint_affine_);
  }

  // ArenaAllocatable interface.
  size_t RequiredArenaBytes() const override {
    return SizeOf(workspace_) * sizeof(double);
  }
  void BindArenaMemory(double* ptr, size_t /*bytes*/) override {
    Initialize(&workspace_, ptr);
  }

  bool workspace_bound() const { return workspace_.W.data() != nullptr; }

 private:
  Workspace do_get_workspace() override { return Workspace(workspace()); }

  int do_number_of_variables() const override { return number_of_variables(); }

  WorkspaceLinear workspace_;
  GramEvaluator gram_evaluator_;
  Eigen::MatrixXd constraint_matrix_;
  Eigen::MatrixXd constraint_affine_;
};

}  // namespace conex
