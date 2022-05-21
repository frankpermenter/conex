#pragma once

#include <vector>

#include "conex/constraint_interface.h"
#include "conex/newton_step.h"
#include <Eigen/Dense>

namespace conex {

struct WorkspaceEqualityConstraints {
  using DenseMatrix = Eigen::MatrixXd;

  friend int SizeOf(const WorkspaceEqualityConstraints&) { return 0; }

  friend void Initialize(WorkspaceEqualityConstraints*, double*) {}

  friend void print(const WorkspaceEqualityConstraints&) {}
  Eigen::Map<DenseMatrix, Eigen::Aligned> W{NULL, 0, 0};
};

class EqualityConstraints : public ConstraintBase {
 public:
  void accept(Visitor* v) override { v->visit(*this); }
  EqualityConstraints(){};
  EqualityConstraints(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b);

  int SizeOfDualVariable() { return A_.rows(); }

  Eigen::MatrixXd constraint_matrix() const { return A_; }
  Eigen::MatrixXd affine_term() const { return b_; }
  Eigen::MatrixXd A_;
  Eigen::MatrixXd b_;

  friend int Rank(const EqualityConstraints&) { return 0; };
  friend void SetIdentity(EqualityConstraints*){};
  friend void PrepareStep(EqualityConstraints* o, const StepOptions&,
                          const Ref& y, StepInfo*);

  friend bool TakeStep(EqualityConstraints*, const StepOptions&) {
    return true;
  };

  friend void GetWeightedSlackEigenvalues(EqualityConstraints*, const Ref&,
                                          double, WeightedSlackEigenvalues*){};

  friend void ConstructSchurComplementSystem(EqualityConstraints* o,
                                             bool initialize,
                                             SchurComplementSystem* sys_);

  int number_of_variables() { return 0; }
  WorkspaceEqualityConstraints workspace_;
  WorkspaceEqualityConstraints* workspace() { return &workspace_; }
  friend bool PerformLineSearch(EqualityConstraints*,
                                const LineSearchParameters&, const Ref&,
                                const Ref&, LineSearchOutput*) {
    bool failure = false;
    return failure;
  }
};

}  // namespace conex
