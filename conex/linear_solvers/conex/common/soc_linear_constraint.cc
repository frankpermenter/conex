#include "conex/common/soc_linear_constraint.h"

namespace conex {

// --- SOCGramEvaluator ---

void SOCGramEvaluator::update_weights() {
  const int n = A_perm_.rows();   // SOC dimension (1 + vector dim)

  // Extract w0 and w1 from ws_->W.
  double w0 = ws_->W(0);
  Eigen::Map<const Eigen::VectorXd> w1(ws_->W.data() + 1, n - 1);
  det_w_ = w0 * w0 - w1.squaredNorm();

  // v = A_perm^T * w.
  Eigen::Map<const Eigen::VectorXd> w_full(ws_->W.data(), n);
  v_ = A_perm_.transpose() * w_full;

  // Just mark weights as fresh.
  weights_dirty_ = false;
}

void SOCGramEvaluator::ContributeBlocks(int clique_id) {
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

void SOCGramEvaluator::set_order(const std::vector<int>& perm) {
  GramEvaluator::set_order(perm);
  set_atranspose_scale(2.0);
  if (!m_computed_) {
    const int n = A_perm_.rows();
    Eigen::VectorXd A0 = A_perm_.row(0).transpose();
    Eigen::MatrixXd A1 = A_perm_.bottomRows(n - 1);
    M_perm_ = A0 * A0.transpose() - A1.transpose() * A1;
    m_computed_ = true;
  }
}

// --- SOCLinearConstraint ---

void SOCLinearConstraint::SetScaling(const Eigen::VectorXd& scaling) {
  CONEX_DEMAND(scaling.size() == constraint_matrix_.rows(),
               "Scaling vector size must match number of constraint rows.");
  workspace_.W = scaling;
  soc_gram_.update_weights();
}

void SOCLinearConstraint::SetWeights(const Eigen::VectorXd& weights) {
  CONEX_DEMAND(weights.size() == constraint_matrix_.rows(),
               "Weight vector size must match number of constraint rows.");
  int size = weights.size();
  Eigen::VectorXd w(size);
  EuclideanJordanAlgebra::socConeOps().sqrt(
      w.data(), weights.data(), size);
  workspace_.W = w;
  soc_gram_.update_weights();
}

}  // namespace conex
