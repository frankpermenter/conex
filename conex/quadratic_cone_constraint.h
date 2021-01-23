#include "newton_step.h"
#include "workspace_soc.h"

namespace conex {

class QuadraticConstraint {
  using StorageType = DenseMatrix;

 public:
  template <typename T>
  QuadraticConstraint(const DenseMatrix& Q, const T& constraint_matrix,
                      const T& constraint_affine)
      : n_(constraint_matrix.rows() - 1),
        workspace_(n_),
        Q_(Q),
        A0_(constraint_matrix.row(0)),
        A1_(constraint_matrix.bottomRows(n_)),
        C0_(constraint_affine(0, 0)),
        C1_(constraint_affine.bottomRows(n_).leftCols(1)) {
    assert(constraint_matrix.rows() == constraint_affine.rows());
    assert(constraint_matrix.rows() == n_ + 1);
    assert(Q_.rows() == n_ || /*We assume Q = I*/ Q_.rows() == 0);
    assert(constraint_affine.cols() == 1);
    // if (Q_.rows() == n_) {
    //  Q_ += 100*Eigen::MatrixXd::Identity(n_, n_);
    //}
    Initialize();
  }

  template <typename T>
  QuadraticConstraint(const T& constraint_matrix, const T& constraint_affine)
      : QuadraticConstraint(DenseMatrix(), constraint_matrix,
                            constraint_affine) {}

  WorkspaceSOC* workspace() { return &workspace_; }

  int number_of_variables() { return A1_.cols(); }
  friend int Rank(const QuadraticConstraint&) { return 2; };
  friend void SetIdentity(QuadraticConstraint* o, double scale) {
    o->workspace_.W0 = scale;
    o->workspace_.W1.setZero();
  }
  friend void TakeStep(QuadraticConstraint* o, const StepOptions& opt,
                       const Ref& y, StepInfo* data);
  friend void GetMuSelectionParameters(QuadraticConstraint* o, const Ref& y,
                                       MuSelectionParameters* p);
  friend void ConstructSchurComplementSystem(QuadraticConstraint* o,
                                             bool initialize,
                                             SchurComplementSystem* sys);

  friend void GetScalingData(QuadraticConstraint* o, Ref* a_fro_norm,
                             double* c_fro_norm) {
    // (a + b)^2 + (a-b)^2 = 2 (a^2 + b^2)
    *a_fro_norm = 2 * (o->A0_.cwiseProduct(o->A0_) + o->A_gram_.diagonal());
    (*a_fro_norm) = a_fro_norm->array().sqrt();
    if (o->Q_.size() > 0) {
      (*c_fro_norm) = std::sqrt(
          2 * (o->C0_ * o->C0_ + o->C1_.transpose() * o->Q_ * o->C1_));
    } else {
      (*c_fro_norm) =
          std::sqrt(2 * (o->C0_ * o->C0_ + o->C1_.transpose() * o->C1_));
    }
  }

 private:
  void Initialize();
  void ComputeNegativeSlack(double inv_sqrt_mu, const Ref& y, double* minus_s_0,
                            Ref* minus_s_1);
  void GeodesicUpdate(const Ref& S, StepInfo* data);
  void AffineUpdate(const Ref& S);

  const int n_ = 0;
  WorkspaceSOC workspace_;
  DenseMatrix Q_;

  const Eigen::VectorXd A0_;
  const Eigen::MatrixXd A1_;
  const double C0_;
  const Eigen::VectorXd C1_;
  Eigen::MatrixXd A_gram_;
};

}  // namespace conex
