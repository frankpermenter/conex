#pragma once
#include "conex/constraint.h"
#include "newton_step.h"
#include "workspace_soc.h"

namespace conex {

using RefType = Eigen::Ref<const Eigen::MatrixXd>;
using NonConstRefType = Eigen::Ref<Eigen::MatrixXd>;
class QuadraticConstraintBase : public Constraint {
  using StorageType = DenseMatrix;

 public:
  template <typename T>
  QuadraticConstraintBase(const DenseMatrix& Q, const T& constraint_matrix,
                          const T& constraint_affine)
      : Q_(Q),
        n_(constraint_matrix.rows() - 1),
        workspace_(n_),
        A0_(constraint_matrix.row(0)),
        A1_(constraint_matrix.bottomRows(n_)),
        C0_(constraint_affine(0, 0)),
        C1_(constraint_affine.bottomRows(n_).leftCols(1)) {
    assert(constraint_matrix.rows() == constraint_affine.rows());
    assert(constraint_matrix.rows() == n_ + 1);
    assert(Q_.rows() == n_ || /*We assume Q = I*/ Q_.rows() == 0);
    assert(constraint_affine.cols() == 1);
    Initialize();
  }

  void accept(Visitor* v) const override { v->visit(*this); }
  bool supports_line_search() const override { return true; }

  template <typename T>
  QuadraticConstraintBase(const T& constraint_matrix,
                          const T& constraint_affine)
      : QuadraticConstraintBase(DenseMatrix(), constraint_matrix,
                                constraint_affine) {}
  virtual ~QuadraticConstraintBase(){};

  WorkspaceSOC* workspace() { return &workspace_; }

  int number_of_variables() const override { return A1_.cols(); }

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

  bool do_perform_line_search(const LineSearchParameters& params,
                              const Eigen::Ref<const Eigen::MatrixXd>& y0,
                              const Eigen::Ref<const Eigen::MatrixXd>& y1,
                              LineSearchOutput* output) override {
    return PerformLineSearchImpl(params, y0, y1, output);
  }

  int do_rank() const override { return 2; }

 protected:
  virtual void Initialize();
  virtual DenseMatrix EvalAtQX(const DenseMatrix& X, DenseMatrix* QX);
  virtual DenseMatrix EvalAtQX(const DenseMatrix& X, NonConstRefType QX);
  double EvalCQX(const DenseMatrix& X, NonConstRefType QX);

  const DenseMatrix Q_;

 private:
  void SetIdentityImpl();
  void PrepareStepImpl(const StepOptions& opt, const RefType& y,
                       StepInfo* data);
  bool PerformLineSearchImpl(const LineSearchParameters& params,
                             const RefType& y0, const RefType& y1,
                             LineSearchOutput* output);
  bool TakeStepImpl(const StepOptions& opt);
  void GetWeightedSlackEigenvaluesImpl(const RefType& y, double c_weight,
                                       WeightedSlackEigenvalues* p);
  void ConstructSchurComplementSystemImpl(bool initialize,
                                          SchurComplementSystem* sys);

  void ComputeNewtonDirection(const StepOptions opts, const RefType& y,
                              double* d0, Eigen::Ref<Eigen::MatrixXd> d1);
  void ComputeNegativeSlack(double inv_sqrt_mu, const RefType& y,
                            double* minus_s_0, NonConstRefType minus_s_1);
  void GeodesicUpdate(const RefType& S, StepInfo* data);
  void AffineUpdate(const RefType& S);

  const int n_ = 0;
  WorkspaceSOC workspace_;

  const Eigen::VectorXd A0_;
  const Eigen::MatrixXd A1_;
  const double C0_;
  const Eigen::VectorXd C1_;

  // TODO(FrankPermenter): Move to workspace.
  Eigen::MatrixXd A_gram_;
  Eigen::MatrixXd A_dot_x_;
};

using QuadraticConstraint = QuadraticConstraintBase;

class QuadraticEpigraph : public QuadraticConstraintBase {
 public:
  QuadraticEpigraph(const DenseMatrix& Qi);

 private:
  void Initialize() override;
  DenseMatrix EvalAtQX(const DenseMatrix& X, DenseMatrix* QX) override;
  DenseMatrix EvalAtQX(const DenseMatrix& X, NonConstRefType QX) override;
};

}  // namespace conex
