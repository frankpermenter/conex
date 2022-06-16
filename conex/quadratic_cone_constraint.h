#pragma once
#include "conex/constraint_interface.h"
#include "newton_step.h"
#include "workspace_soc.h"

namespace conex {

using RefType = Eigen::Ref<const Eigen::MatrixXd>;
using NonConstRefType = Eigen::Ref<Eigen::MatrixXd>;
class QuadraticConstraintBase : public ConstraintBase {
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

  void accept(Visitor* v) override { v->visit(*this); }

  template <typename T>
  QuadraticConstraintBase(const T& constraint_matrix,
                          const T& constraint_affine)
      : QuadraticConstraintBase(DenseMatrix(), constraint_matrix,
                                constraint_affine) {}

  WorkspaceSOC* workspace() { return &workspace_; }

  int number_of_variables() const override { return A1_.cols(); }
  friend int Rank(const QuadraticConstraintBase&) { return 2; };
  friend void SetIdentity(QuadraticConstraintBase* o);
  friend void PrepareStep(QuadraticConstraintBase* o, const StepOptions& opt,
                          const RefType& y, StepInfo* data);

  friend bool PerformLineSearch(QuadraticConstraintBase* o,
                                const LineSearchParameters& params,
                                const RefType& y0, const RefType& y1,
                                LineSearchOutput* output);
  friend bool TakeStep(QuadraticConstraintBase* o, const StepOptions& opt);
  friend void GetWeightedSlackEigenvalues(QuadraticConstraintBase* o,
                                          const RefType& y, double c_weight,
                                          WeightedSlackEigenvalues* p);
  friend void ConstructSchurComplementSystem(QuadraticConstraintBase* o,
                                             bool initialize,
                                             SchurComplementSystem* sys);

  virtual ~QuadraticConstraintBase(){};

 protected:
  virtual void Initialize();
  virtual DenseMatrix EvalAtQX(const DenseMatrix& X, DenseMatrix* QX);
  virtual DenseMatrix EvalAtQX(const DenseMatrix& X, NonConstRefType QX);
  double EvalCQX(const DenseMatrix& X, NonConstRefType QX);

  const DenseMatrix Q_;

 private:
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
