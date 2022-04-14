#include "conex/constraint_interface.h"
#include "conex/error_checking_macros.h"
#include "conex/newton_step.h"
#include "conex/workspace_soc.h"

namespace conex {

class SOCConstraint : public ConstraintBase {
  using StorageType = DenseMatrix;

 public:
  template <typename T1, typename T2>
  SOCConstraint(const T1& constraint_matrix, const T2& constraint_affine)
      : workspace_(constraint_matrix.rows() - 1),
        constraint_matrix_(constraint_matrix),
        constraint_affine_(constraint_affine) {
    CONEX_DEMAND(constraint_matrix_.rows() == constraint_affine_.rows(),
                 "Invalid SOC problem data.");
  }

  void accept(Visitor* v) override { v->visit(*this); }
  // Lorentz cone a subset of R^(n+1).
  SOCConstraint(int n) : workspace_(n), n_(n) {}

  WorkspaceSOC* workspace() { return &workspace_; }

  int number_of_variables() { return constraint_matrix_.cols(); }
  friend int Rank(const SOCConstraint&) { return 2; };
  friend void SetIdentity(SOCConstraint* o) {
    *o->workspace_.W0 = 1;
    o->workspace_.W1.setZero();
  }
  friend void PrepareStep(SOCConstraint* o, const StepOptions& opt,
                          const Ref& y, StepInfo* data);

  friend bool TakeStep(SOCConstraint* o, const StepOptions& opt);

  friend void GetWeightedSlackEigenvalues(SOCConstraint* o, const Ref& y,
                                          double c_weight,
                                          WeightedSlackEigenvalues* p);
  friend void ConstructSchurComplementSystem(SOCConstraint* o, bool initialize,
                                             SchurComplementSystem* sys);

  friend CONEX_STATUS UpdateLinearOperator(SOCConstraint* o, double val,
                                           int var, int r, int c, int dim);
  friend CONEX_STATUS UpdateAffineTerm(SOCConstraint* o, double val, int r,
                                       int c, int dim);

  friend bool PerformLineSearch(SOCConstraint* o,
                                const LineSearchParameters& params,
                                const Ref& y0, const Ref& y1,
                                LineSearchOutput* output);

  DenseMatrix constraint_matrix() const { return constraint_matrix_; }
  DenseMatrix affine_term() const { return constraint_affine_; }

 private:
  void ComputeNegativeSlack(double inv_sqrt_mu, const Ref& y, Ref* minus_s);
  void GeodesicUpdate(const Ref& S, StepInfo* data);
  void AffineUpdate(const Ref& S);
  Eigen::VectorXd BuildNewtonDirection(double c_weight, const Ref& y);

  WorkspaceSOC workspace_;
  DenseMatrix constraint_matrix_;
  DenseMatrix constraint_affine_;
  int n_;
};

}  // namespace conex
