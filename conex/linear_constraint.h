#pragma once
#include "conex/constraint.h"
#include "conex/error_checking_macros.h"
#include "conex/newton_step.h"
#include "conex/supernodal_assembler_base.h"
#include "linear_workspace.h"

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

void PreprocessLinearInequality(const Eigen::MatrixXd& A,
                                const Eigen::MatrixXd& lb,
                                const Eigen::MatrixXd& ub,
                                Eigen::MatrixXd* Aineq, Eigen::MatrixXd* bineq,
                                Eigen::MatrixXd* Aeq, Eigen::MatrixXd* beq,
                                double rescale = true);
// TODO(FrankPermenter) Rename to LinearInequality
class LinearConstraint : public Constraint {
  using StorageType = DenseMatrix;

 public:
  void accept(Visitor* v) const override { v->visit(*this); }
  bool supports_line_search() const override { return true; }
  LinearConstraint(const Eigen::MatrixXd& constraint_matrix,
                   const Eigen::MatrixXd& constraint_affine);

  WorkspaceLinear* workspace() { return &workspace_; }

  int number_of_variables() const override { return constraint_matrix_.cols(); }
  LazySymmetricMatrix* GetLazyEvaluator() override {
    gram_evaluator_.bind(&workspace_, &constraint_matrix_);
    return &gram_evaluator_;
  }
  void set_precompute_gram(bool v) { gram_evaluator_.set_precompute_gram(v); }
  DenseMatrix constraint_matrix() const { return constraint_matrix_; }
  DenseMatrix affine_term() const { return constraint_affine_; }

 private:
  void do_schur_complement(bool initialize,
                           SchurComplementSystem* sys) override {
    ConstructSchurComplementSystemImpl(initialize, sys);
  }

  void do_set_identity() override { SetIdentityImpl(); }

  void do_weighted_slack_eigenvalues(const Ref& y, double c_weight,
                                     WeightedSlackEigenvalues* p) override {
    GetWeightedSlackEigenvaluesImpl(y, c_weight, p);
  }

  Workspace do_get_workspace() override { return Workspace(workspace()); }

  void do_prepare_step(const StepOptions& opt, const Ref& y,
                       StepInfo* info) override {
    PrepareStepImpl(opt, y, info);
  }

  void do_get_dual_variable(double* var) override {
    CopyDualVariableFromWorkspace(workspace(), var);
  }

  bool do_take_step(const StepOptions& opts) override {
    return TakeStepImpl(opts);
  }

  int do_dual_variable_size() override {
    return DualVariableSizeFromWorkspace(workspace());
  }

  int do_number_of_variables() const override { return number_of_variables(); }

  void do_apply_rescaling(Eigen::Ref<Eigen::MatrixXd> ArW,
                          double* inner_product_of_c_and_rW) override {
    ApplyRescalingImpl(ArW, inner_product_of_c_and_rW);
  }

  CONEX_STATUS do_update_linear_operator(double val, int var, int row, int col,
                                         int hyper_complex_dim) override {
    return UpdateLinearOperatorImpl(val, var, row, col, hyper_complex_dim);
  }

  CONEX_STATUS do_update_affine_term(double val, int row, int col,
                                     int hyper_complex_dim) override {
    return UpdateAffineTermImpl(val, row, col, hyper_complex_dim);
  }

  bool do_perform_line_search(const LineSearchParameters& params,
                              const Eigen::Ref<const Eigen::MatrixXd>& y0,
                              const Eigen::Ref<const Eigen::MatrixXd>& y1,
                              LineSearchOutput* output) override {
    return PerformLineSearchImpl(params, y0, y1, output);
  }

  int do_rank() const override { return workspace_.n_; }

  void SetIdentityImpl();
  void PrepareStepImpl(const StepOptions& opt,
                       const Eigen::Ref<const Eigen::MatrixXd>& y0,
                       StepInfo* data);
  bool PerformLineSearchImpl(const LineSearchParameters& params,
                             const Eigen::Ref<const Eigen::MatrixXd>& y0,
                             const Eigen::Ref<const Eigen::MatrixXd>& y1,
                             LineSearchOutput* output);
  void GetWeightedSlackEigenvaluesImpl(const Ref& y, double c_weight,
                                       WeightedSlackEigenvalues* p);
  void ConstructSchurComplementSystemImpl(bool initialize,
                                          SchurComplementSystem* sys);
  void ApplyRescalingImpl(Eigen::Ref<Eigen::MatrixXd>, double*);
  bool TakeStepImpl(const StepOptions&);
  CONEX_STATUS UpdateLinearOperatorImpl(double val, int var, int r, int c,
                                        int dim);
  CONEX_STATUS UpdateAffineTermImpl(double val, int r, int c, int dim);

  void ComputeNegativeSlack(double inv_sqrt_mu,
                            const Eigen::Ref<const Eigen::MatrixXd>& y,
                            Eigen::Ref<Eigen::MatrixXd> minus_s);
  void AffineUpdate(const Eigen::Ref<const Eigen::MatrixXd>& y, int step_type);

  WorkspaceLinear workspace_;
  GramEvaluator gram_evaluator_;
  DenseMatrix constraint_matrix_;
  DenseMatrix constraint_affine_;
};

class LowerBound : public LinearConstraint {
 public:
  LowerBound(const Eigen::VectorXd& lower_bounds)
      : LinearConstraint(-Eigen::MatrixXd::Identity(lower_bounds.rows(),
                                                    lower_bounds.rows()),
                         -lower_bounds) {}
};

class UpperBound : public LinearConstraint {
 public:
  UpperBound(const Eigen::VectorXd& upper_bounds)
      : LinearConstraint(
            Eigen::MatrixXd::Identity(upper_bounds.rows(), upper_bounds.rows()),
            upper_bounds) {}
};

}  // namespace conex
