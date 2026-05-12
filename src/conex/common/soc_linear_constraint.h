// SOC-cone specialization of LinearConstraint.
// Gram = A^T P(W) A = -2*det(w)*A^T*R*A + 4*(A^Tw)(A^Tw)^T
// where R = diag(1,-1,...,-1) and det(w) = w0^2 - ||w1||^2.

#pragma once
#include <Eigen/Core>
#include "conex/common/linear_constraint.h"
#include "conex/common/soc_cone_ops.h"

namespace conex {

// Gram evaluator for SOC segments.
// Precomputes M = A0*A0^T - A1^T*A1, then at each update_weights:
//   G = -2*det_w*M + 4*v*v^T  where v = A_perm^T * w.
class SOCGramEvaluator : public GramEvaluator {
 public:
  void update_weights() override;

  void ContributeBlocks(int clique_id) override;

  // Precompute M_perm = A0*A0^T - A1^T*A1 after permutation.
  // Also set the trace inner product scale (2 for SOC).
  void set_order(const std::vector<int>& perm) override;

 private:
  // Access registered_blocks_ from base class — made protected.
  using GramEvaluator::registered_blocks_;

  Eigen::MatrixXd M_perm_;  // A0*A0^T - A1^T*A1 (precomputed)
  Eigen::VectorXd v_;       // A_perm^T * w (per-iteration)
  double det_w_ = 0;
  bool m_computed_ = false;
};

// LinearConstraint for SOC segments.
class SOCLinearConstraint : public LinearConstraint {
 public:
  SOCLinearConstraint(const Eigen::MatrixXd& constraint_matrix,
                      const Eigen::MatrixXd& constraint_affine)
      : LinearConstraint(constraint_matrix, constraint_affine) {}

  BlockAssembler* GetBlockAssembler() override {
    soc_gram_.bind(&workspace_, &constraint_matrix_);
    return &soc_gram_;
  }

  const GramEvaluator& gram() const override { return soc_gram_; }

  // SetScaling: w is the SOC weight (t, x₁, ..., xₙ).
  // Store directly — the SOCGramEvaluator reads from workspace_.W.
  void SetScaling(const Eigen::VectorXd& scaling) override;

  // SetWeights: receives W² (same convention as nonneg).
  // For SOC: W² in Jordan algebra is (t²+||x||², 2tx).
  // We need to recover W from W². Use spectral sqrt.
  void SetWeights(const Eigen::VectorXd& weights) override;

 private:
  SOCGramEvaluator soc_gram_;
};

}  // namespace conex
