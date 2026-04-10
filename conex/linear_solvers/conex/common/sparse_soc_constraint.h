// Sparse assembler for SOC constraints.
// Each SOC constraint reports one clique: the union of nonzero columns of A.
// Decompose creates one SOCLinearConstraint per constraint.

#pragma once
#include "conex/common/soc_cone_ops.h"
#include "conex/common/soc_linear_constraint.h"
#include "conex/common/sparse_linear_constraint.h"

namespace conex {

class SparseSOCConstraintAssembler : public SparseLinearConstraintAssembler {
 public:
  using SparseLinearConstraintAssembler::SparseLinearConstraintAssembler;

 protected:
  std::unique_ptr<LinearConstraint> MakeConstraint(
      const Eigen::MatrixXd& A, const Eigen::VectorXd& b) override {
    auto c = std::make_unique<SOCLinearConstraint>(A, b);
    c->cone_ops_ = &EuclideanJordanAlgebra::socConeOps();
    return c;
  }
};

}  // namespace conex
