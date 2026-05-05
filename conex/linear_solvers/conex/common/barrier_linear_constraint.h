// Linear constraint with a general barrier (non-symmetric cone).
//
// Like LinearConstraint, but the Gram assembly uses chol(H(z)) * A
// instead of diag(W) * A.  Works with any cone whose z-space
// SymmetricConeOperations methods are implemented.

#pragma once

#include "conex/common/linear_constraint.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/symmetric_cone_operations.h"

namespace conex {

class BarrierGramEvaluator : public GramEvaluator {
 public:
  void set_ops(const EuclideanJordanAlgebra::SymmetricConeOperations* ops) {
    ops_ = ops;
  }

  void update_weights() override {
    const int n = static_cast<int>(ws_->W.size());

    // Compute full Hessian H(z) where z is stored in ws_->W.
    Eigen::MatrixXd H(n, n);
    ops_->hessian(H.data(), ws_->W.data(), n);

    // Cholesky: H = L L^T.
    Eigen::LLT<Eigen::MatrixXd> llt(H);

    // WA_perm_ = L^T * A_perm_.  Gram = (WA)^T(WA) = A^T L L^T A = A^T H A.
    WA_perm_.noalias() = llt.matrixU() * A_perm_;

    weights_dirty_ = false;
  }

 private:
  const EuclideanJordanAlgebra::SymmetricConeOperations* ops_ = nullptr;
};

// Inherits from LinearConstraint, overriding the GramEvaluator.
class BarrierLinearConstraint : public LinearConstraint {
 public:
  BarrierLinearConstraint(
      const Eigen::MatrixXd& constraint_matrix,
      const Eigen::MatrixXd& constraint_affine,
      const EuclideanJordanAlgebra::SymmetricConeOperations* ops)
      : LinearConstraint(constraint_matrix, constraint_affine) {
    cone_ops_ = ops;
    barrier_gram_.set_ops(ops);
  }

  BlockAssembler* GetBlockAssembler() override {
    barrier_gram_.bind(&workspace_, &constraint_matrix_);
    return &barrier_gram_;
  }

  const GramEvaluator& gram() const override { return barrier_gram_; }

  void SetScaling(const Eigen::VectorXd& scaling) override {
    workspace_.W = scaling;
    barrier_gram_.update_weights();
  }

  void SetWeights(const Eigen::VectorXd& weights) override {
    workspace_.W = weights;
    barrier_gram_.update_weights();
  }

 private:
  BarrierGramEvaluator barrier_gram_;
};

// Assembler that creates BarrierLinearConstraint instead of LinearConstraint.
class SparseBarrierConstraintAssembler
    : public SparseLinearConstraintAssembler {
 public:
  SparseBarrierConstraintAssembler(
      std::unique_ptr<SparseLinearConstraint> slc,
      const std::vector<int>& all_variables,
      const EuclideanJordanAlgebra::SymmetricConeOperations* ops)
      : SparseLinearConstraintAssembler(std::move(slc), all_variables),
        ops_(ops) {}

 protected:
  std::unique_ptr<LinearConstraint> MakeConstraint(
      const Eigen::MatrixXd& A, const Eigen::VectorXd& b) override {
    return std::make_unique<BarrierLinearConstraint>(A, b, ops_);
  }

 private:
  const EuclideanJordanAlgebra::SymmetricConeOperations* ops_;
};

}  // namespace conex
