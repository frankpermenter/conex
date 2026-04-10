// SOC-cone specialization of LinearConstraint.
// Gram = A^T P(W) A = -2*det(w)*A^T*R*A + 4*(A^Tw)(A^Tw)^T
// where R = diag(1,-1,...,-1) and det(w) = w0^2 - ||w1||^2.

#pragma once
#include <Eigen/Dense>
#include "conex/common/linear_constraint.h"
#include "conex/common/soc_cone_ops.h"

namespace conex {

// Gram evaluator for SOC segments.
// Precomputes M = A0*A0^T - A1^T*A1, then at each update_weights:
//   G = -2*det_w*M + 4*v*v^T  where v = A_perm^T * w.
class SOCGramEvaluator : public GramEvaluator {
 public:
  void update_weights() override {
    const int n = A_perm_.rows();   // SOC dimension (1 + vector dim)
    const int p = A_perm_.cols();   // number of permuted variables

    // Extract w0 and w1 from ws_->W.
    double w0 = ws_->W(0);
    Eigen::Map<const Eigen::VectorXd> w1(ws_->W.data() + 1, n - 1);
    det_w_ = w0 * w0 - w1.squaredNorm();

    // v = A_perm^T * w.
    Eigen::Map<const Eigen::VectorXd> w_full(ws_->W.data(), n);
    v_ = A_perm_.transpose() * w_full;

    // WA_perm_ is not used for the Gram (we override ContributeBlocks),
    // but it's needed for ContributeAtranspose and MultiplyA which use
    // A_perm_ directly (inherited from GramEvaluator).
    // Just mark weights as fresh.
    weights_dirty_ = false;
  }

  void ContributeBlocks(int clique_id) override {
    ensure_weights_fresh();
    auto it = registered_blocks_.find(clique_id);
    if (it == registered_blocks_.end()) return;

    for (const auto& bc : it->second) {
      using StrideType = Eigen::Stride<Eigen::Dynamic, 1>;
      Eigen::Map<Eigen::MatrixXd, 0, StrideType> dest(
          bc.dest, bc.rows, bc.cols, StrideType(bc.dest_ld, 1));

      // M block: -2*det_w * M_perm[block].
      auto M_block = M_perm_.block(bc.q_row, bc.q_col, bc.rows, bc.cols);
      // v block: 4 * v[rows] * v[cols]^T.
      auto v_rows = v_.segment(bc.q_row, bc.rows);
      auto v_cols = v_.segment(bc.q_col, bc.cols);

      if (bc.lower_only) {
        // Diagonal block: lower triangle only.
        for (int j = 0; j < bc.cols; ++j) {
          for (int i = j; i < bc.rows; ++i) {
            dest(i, j) += -2.0 * det_w_ * M_block(i, j)
                          + 4.0 * v_rows(i) * v_cols(j);
          }
        }
      } else {
        dest.noalias() += -2.0 * det_w_ * M_block
                          + 4.0 * v_rows * v_cols.transpose();
      }
    }
  }

  // Precompute M_perm = A0*A0^T - A1^T*A1 after permutation.
  void set_order(const std::vector<int>& perm) override {
    GramEvaluator::set_order(perm);
    if (!m_computed_) {
      const int n = A_perm_.rows();
      const int p = A_perm_.cols();
      Eigen::VectorXd A0 = A_perm_.row(0).transpose();
      Eigen::MatrixXd A1 = A_perm_.bottomRows(n - 1);
      M_perm_ = A0 * A0.transpose() - A1.transpose() * A1;
      m_computed_ = true;
    }
  }

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
  void SetScaling(const Eigen::VectorXd& scaling) override {
    CONEX_DEMAND(scaling.size() == constraint_matrix_.rows(),
                 "Scaling vector size must match number of constraint rows.");
    workspace_.W = scaling;
    soc_gram_.update_weights();
  }

  // SetWeights: receives W² (same convention as nonneg).
  // For SOC: W² in Jordan algebra is (t²+||x||², 2tx).
  // We need to recover W from W². Use spectral sqrt.
  void SetWeights(const Eigen::VectorXd& weights) override {
    CONEX_DEMAND(weights.size() == constraint_matrix_.rows(),
                 "Weight vector size must match number of constraint rows.");
    int size = weights.size();
    Eigen::VectorXd w(size);
    EuclideanJordanAlgebra::socConeOps().sqrt(
        w.data(), weights.data(), size);
    workspace_.W = w;
    soc_gram_.update_weights();
  }

 private:
  SOCGramEvaluator soc_gram_;
};

}  // namespace conex
