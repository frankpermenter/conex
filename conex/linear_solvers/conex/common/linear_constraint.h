#pragma once
#include <unordered_map>

#include "conex/common/arena_allocatable.h"
#include "conex/common/block_partition.h"
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

  // Permute columns of A.  Weight computation is deferred.
  void set_order(const std::vector<int>& perm) override {
    if (order_set_) return;
    const int n = A_->rows();
    const int m = static_cast<int>(perm.size());
    A_perm_.resize(n, m);
    for (int i = 0; i < m; ++i) A_perm_.col(i) = A_->col(perm[i]);
    WA_perm_.resize(n, m);
    weights_dirty_ = true;
    order_set_ = true;
  }

  // Recompute WA_perm_ = diag(W) * A_perm_.
  // Called by SetWeights on the LinearConstraint, or by ensure_weights_fresh.
  void update_weights() {
    WA_perm_.noalias() = ws_->W.asDiagonal() * A_perm_;
    weights_dirty_ = false;
  }

  int rows() const override { return ws_->num_vars_; }
  int cols() const override { return ws_->num_vars_; }

  int sn_count() const { return sn_count_; }
  void set_sn_count(int c) override { sn_count_ = c; }

  // Compute A_perm_ * [x_sn; x_sep].
  Eigen::VectorXd MultiplyBlock(
      Eigen::Ref<const Eigen::MatrixXd> x_sn,
      Eigen::Ref<const Eigen::MatrixXd> x_sep) const {
    const int m = A_perm_.rows();
    const int ns = sn_count_;
    Eigen::VectorXd r(m);
    r.noalias() = A_perm_.leftCols(ns) * x_sn;
    if (A_perm_.cols() > ns)
      r.noalias() += A_perm_.rightCols(A_perm_.cols() - ns) * x_sep;
    return r;
  }

  // Compute r = A_perm_ * [x_sn; x_sep] - b.
  Eigen::VectorXd ComputeBlockResidual(
      Eigen::Ref<const Eigen::MatrixXd> x_sn,
      Eigen::Ref<const Eigen::MatrixXd> x_sep,
      const Eigen::MatrixXd& b) const {
    Eigen::VectorXd r = MultiplyBlock(x_sn, x_sep);
    r -= b;
    return r;
  }

  // Accumulate A_perm_^T * v into [x_sn; x_sep].
  void MultiplyBlockTransposeAdd(
      const Eigen::VectorXd& v,
      Eigen::Ref<Eigen::MatrixXd> x_sn,
      Eigen::Ref<Eigen::MatrixXd> x_sep) const {
    const int ns = sn_count_;
    x_sn.noalias() += A_perm_.leftCols(ns).transpose() * v;
    if (A_perm_.cols() > ns)
      x_sep.noalias() += A_perm_.rightCols(A_perm_.cols() - ns).transpose() * v;
  }

  bool RegisterContributions(
      int clique_id, const std::vector<int>& perm,
      const std::vector<BlockContribution>& blocks) override {
    if (!order_set_) set_order(perm);
    registered_blocks_[clique_id] = blocks;
    return true;
  }

  void RegisterVectorContributions(
      const std::vector<VectorBlockContribution>& blocks) override {
    vector_blocks_ = blocks;
  }


  // Accumulate A_perm_^T * V into supernode blocks and separator scratch.
  // V is (m x batch).  Writes directly to parent destinations.
  template <typename SepAccessor>
  void ContributeAtranspose(
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      BlockPartition& supernodes, SepAccessor& sep, int nc) const {
    for (const auto& vbc : vector_blocks_) {
      auto atv = A_perm_.middleCols(vbc.q_start, vbc.length).transpose() * V;
      if (vbc.dest_is_sn) {
        auto blk = supernodes.block(vbc.dest_block);
        CONEX_DEMAND(vbc.dest_offset + vbc.length <= blk.rows(),
                     "VBC sn write out of bounds");
        blk.middleRows(vbc.dest_offset, vbc.length) += atv;
      } else {
        auto blk = sep.block(vbc.dest_block, nc);
        CONEX_DEMAND(vbc.dest_offset + vbc.length <= blk.rows(),
                     "VBC sep write out of bounds");
        blk.middleRows(vbc.dest_offset, vbc.length) += atv;
      }
    }
  }

  // Compute A_perm_ * x by reading from supernode blocks and separator scratch.
  template <typename SepAccessor>
  Eigen::MatrixXd MultiplyA(
      const BlockPartition& supernodes, const SepAccessor& sep, int nc) const {
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(A_perm_.rows(), nc);
    for (const auto& vbc : vector_blocks_) {
      auto cols = A_perm_.middleCols(vbc.q_start, vbc.length);
      if (vbc.dest_is_sn) {
        auto blk = supernodes.block(vbc.dest_block);
        CONEX_DEMAND(vbc.dest_offset + vbc.length <= blk.rows(),
                     "VBC sn read out of bounds");
        result.noalias() += cols * blk.middleRows(vbc.dest_offset, vbc.length);
      } else {
        auto blk = sep.block(vbc.dest_block, nc);
        CONEX_DEMAND(vbc.dest_offset + vbc.length <= blk.rows(),
                     "VBC sep read out of bounds");
        result.noalias() += cols * blk.middleRows(vbc.dest_offset, vbc.length);
      }
    }
    return result;
  }

  void ContributeBlocks(int clique_id) override {
    ensure_weights_fresh();
    auto it = registered_blocks_.find(clique_id);
    if (it == registered_blocks_.end()) return;
    for (const auto& bc : it->second) {
      using StrideType = Eigen::Stride<Eigen::Dynamic, 1>;
      Eigen::Map<Eigen::MatrixXd, 0, StrideType> dest(
          bc.dest, bc.rows, bc.cols, StrideType(bc.dest_ld, 1));
      if (bc.lower_only) {
        dest.selfadjointView<Eigen::Lower>().rankUpdate(
            WA_perm_.middleCols(bc.q_row, bc.rows).transpose());
      } else {
        dest.noalias() +=
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
  bool order_set_ = false;
  bool weights_dirty_ = true;
  int sn_count_ = 0;
  std::unordered_map<int, std::vector<BlockContribution>> registered_blocks_;
  std::vector<VectorBlockContribution> vector_blocks_;
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

  // A_perm_ * [x_sn; x_sep]
  Eigen::VectorXd MultiplyBlock(
      Eigen::Ref<const Eigen::MatrixXd> x_sn,
      Eigen::Ref<const Eigen::MatrixXd> x_sep) const {
    return gram_evaluator_.MultiplyBlock(x_sn, x_sep);
  }

  // Accumulate A_perm_^T * v into [x_sn; x_sep].
  void MultiplyBlockTransposeAdd(
      const Eigen::VectorXd& v,
      Eigen::Ref<Eigen::MatrixXd> x_sn,
      Eigen::Ref<Eigen::MatrixXd> x_sep) const {
    gram_evaluator_.MultiplyBlockTransposeAdd(v, x_sn, x_sep);
  }

  int sn_count() const { return gram_evaluator_.sn_count(); }

  // Access the GramEvaluator for vector block operations.
  const GramEvaluator& gram() const { return gram_evaluator_; }

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
