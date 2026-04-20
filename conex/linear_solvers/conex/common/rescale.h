// Model rescaling: transform so that b → identity element.
// This places W=I near the central path for the geodesic IPM.
//
// Row scaling (per constraint):
//   Nonneg: divide row i by b_i → b = ones.
//   PSD: Cholesky B = LL^T → A_k = L^{-1} A_k L^{-T}, B = I.
//   SOC: scale by 1/b_0, apply rotation to zero b_1 component.
//
// Column scaling (global):
//   MaxAbsValue: D_j = max_i |A_{ij}|.  One-shot, simple.
//   L2Norm:      D_j = ||A[:,j]||_2.  Equalizes diag(A^T A).
//   Ruiz:        Iterative symmetric equilibration (row + column).
//                Converges to doubly-stochastic-like scaling.
//
// Returns the rescaled problem + scaling info for solution recovery.

#pragma once
#include <Eigen/Dense>
#include "conex/common/model.h"

namespace conex {

enum class ColumnScaling {
  MaxAbsValue,  // D_j = max_i |A_{ij}| (original)
  L2Norm,       // D_j = ||A[:,j]||_2 (Gram diagonal equalization)
  Ruiz,         // Iterative Ruiz equilibration (default)
};

struct RescaleInfo {
  // Column scaling: x_original = col_scale .* x_rescaled.
  Eigen::VectorXd col_scale;
  // Row scaling per constraint (for Ruiz).
  std::vector<Eigen::VectorXd> row_scale;
  int original_n = 0;
  bool was_rescaled = false;

  Eigen::VectorXd Unscale(const Eigen::VectorXd& x_rescaled) const {
    if (!was_rescaled) return x_rescaled;
    return col_scale.cwiseProduct(x_rescaled);
  }
};

// Rescale a problem so b ≈ identity element for each constraint.
// Modifies the cost accordingly.
std::pair<Model, RescaleInfo> RescaleProblem(
    const Model& problem,
    ColumnScaling strategy = ColumnScaling::Ruiz);

}  // namespace conex
