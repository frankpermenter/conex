#pragma once
#include "conex/constraint.h"
#include "conex/error_checking_macros.h"
#include "conex/newton_step.h"
#include "linear_workspace.h"

namespace conex {

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
  DenseMatrix constraint_matrix() const { return constraint_matrix_; }
  DenseMatrix affine_term() const { return constraint_affine_; }

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
    memcpy(static_cast<void*>(var), static_cast<void*>(workspace()->W.data()),
           sizeof(double) * do_dual_variable_size());
  }

  bool do_take_step(const StepOptions& opts) override { return TakeStepImpl(opts); }

  int do_dual_variable_size() override {
    return workspace()->W.rows() * workspace()->W.cols();
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

 private:
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
