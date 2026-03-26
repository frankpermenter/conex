#pragma once
#include "conex/common/constraint.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/supernodal_assembler_base.h"
#include "conex/common/linear_workspace.h"

namespace conex {

class GramEvaluator : public LazySymmetricMatrix {
 public:
  GramEvaluator() = default;
  void bind(WorkspaceLinear* ws, const Eigen::MatrixXd* A) {
    ws_ = ws;
    A_ = A;
  }

  void set_precompute_gram(bool v) { precompute_gram_ = v; }

  // Called once: permute columns of A and compute initial WA_perm_.
  void set_order(const std::vector<int>& perm) override {
    if (order_set_) return;
    const int n = A_->rows();
    const int m = static_cast<int>(perm.size());
    A_perm_.resize(n, m);
    for (int i = 0; i < m; ++i) {
      A_perm_.col(i) = A_->col(perm[i]);
    }
    WA_perm_.resize(n, m);
    update_weights();
    order_set_ = true;
  }

  // Recompute WA_perm_ = diag(W) * A_perm_ (and G_ if precomputing).
  void update_weights() {
    WA_perm_.noalias() = ws_->W.asDiagonal() * A_perm_;
    if (precompute_gram_) {
      G_.noalias() = WA_perm_.transpose() * WA_perm_;
    }
  }

  void add_block(int row, int col, int rows, int cols,
                 Eigen::Ref<Eigen::MatrixXd> dest) const override {
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

 private:
  WorkspaceLinear* ws_ = nullptr;
  const Eigen::MatrixXd* A_ = nullptr;
  Eigen::MatrixXd A_perm_;
  Eigen::MatrixXd WA_perm_;
  Eigen::MatrixXd G_;
  bool order_set_ = false;
  bool precompute_gram_ = false;
};

class LinearConstraint : public Constraint {
 public:
  LinearConstraint(const Eigen::MatrixXd& constraint_matrix,
                   const Eigen::MatrixXd& constraint_affine);

  WorkspaceLinear* workspace() { return &workspace_; }

  int number_of_variables() const override { return constraint_matrix_.cols(); }
  LazySymmetricMatrix* GetLazyEvaluator() override {
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

 private:
  Workspace do_get_workspace() override { return Workspace(workspace()); }

  int do_number_of_variables() const override { return number_of_variables(); }

  WorkspaceLinear workspace_;
  GramEvaluator gram_evaluator_;
  Eigen::MatrixXd constraint_matrix_;
  Eigen::MatrixXd constraint_affine_;
};

}  // namespace conex
