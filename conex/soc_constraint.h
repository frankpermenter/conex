#include "conex/constraint.h"
#include "conex/error_codes.h"
#include "conex/newton_step.h"
#include "conex/workspace_soc.h"

namespace conex {

using RefType = Eigen::Ref<const Eigen::MatrixXd>;
using NonConstRefType = Eigen::Ref<Eigen::MatrixXd>;
class SOCConstraint : public Constraint {
  using StorageType = DenseMatrix;

 public:
  SOCConstraint(const Eigen::MatrixXd& constraint_matrix,
                const Eigen::MatrixXd& constraint_affine);

  void accept(Visitor* v) const override { v->visit(*this); }
  bool supports_line_search() const override { return true; }
  // Lorentz cone a subset of R^(n+1).
  SOCConstraint(int n, int num_vars) : workspace_(n), n_(n) {
    constraint_matrix_.resize(n + 1, num_vars);
    constraint_affine_.resize(n + 1, 1);
    constraint_matrix_.setZero();
    constraint_affine_.setZero();
  }

  WorkspaceSOC* workspace() { return &workspace_; }

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

  bool do_take_step(const StepOptions& opts) override {
    return TakeStepImpl(opts);
  }

  int do_dual_variable_size() override {
    return workspace()->W.rows() * workspace()->W.cols();
  }

  int do_number_of_variables() const override { return number_of_variables(); }

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

  int do_rank() const override { return 2; }

 private:
  void SetIdentityImpl();
  void PrepareStepImpl(const StepOptions& opt, const RefType& y,
                       StepInfo* data);
  bool TakeStepImpl(const StepOptions& opt);
  void GetWeightedSlackEigenvaluesImpl(const RefType& y, double c_weight,
                                       WeightedSlackEigenvalues* p);
  void ConstructSchurComplementSystemImpl(bool initialize,
                                          SchurComplementSystem* sys);
  CONEX_STATUS UpdateLinearOperatorImpl(double val, int var, int r, int c,
                                        int dim);
  CONEX_STATUS UpdateAffineTermImpl(double val, int r, int c, int dim);
  bool PerformLineSearchImpl(const LineSearchParameters& params,
                             const RefType& y0, const RefType& y1,
                             LineSearchOutput* output);

  void ComputeNegativeSlack(double inv_sqrt_mu, const RefType& y,
                            NonConstRefType minus_s);
  void GeodesicUpdate(const RefType& S, StepInfo* data);
  void AffineUpdate(const RefType& S);
  Eigen::VectorXd BuildNewtonDirection(const StepOptions& options,
                                       const RefType& y);

  WorkspaceSOC workspace_;
  DenseMatrix constraint_matrix_;
  DenseMatrix constraint_affine_;
  int n_;
};

}  // namespace conex
