#include "conex/constraint_interface.h"
#include "conex/error_codes.h"
#include "conex/newton_step.h"
#include "conex/workspace_soc.h"

namespace conex {

using RefType = Eigen::Ref<const Eigen::MatrixXd>;
using NonConstRefType = Eigen::Ref<Eigen::MatrixXd>;
class SOCConstraint : public ConstraintBase {
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
  friend int Rank(const SOCConstraint&) { return 2; };
  friend void SetIdentity(SOCConstraint* o) {
    *o->workspace_.W0 = 1;
    o->workspace_.W1.setZero();
  }
  friend void PrepareStep(SOCConstraint* o, const StepOptions& opt,
                          const RefType& y, StepInfo* data);

  friend bool TakeStep(SOCConstraint* o, const StepOptions& opt);

  friend void GetWeightedSlackEigenvalues(SOCConstraint* o, const RefType& y,
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
                                const RefType& y0, const RefType& y1,
                                LineSearchOutput* output);

  DenseMatrix constraint_matrix() const { return constraint_matrix_; }
  DenseMatrix affine_term() const { return constraint_affine_; }

 private:
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
