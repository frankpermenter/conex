#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Core>

namespace conex {

// Optimality summary computed via KKT system operations.
// Always correct regardless of tree decomposition.
struct OptimalitySummary {
  double dual_residual = 0;      // ||A'λ - Qx - c||
  double complementarity = 0;    // <s, λ>
  double min_slack = 0;          // min eigenvalue of s (primal feasibility)
  double min_dual = 0;           // min eigenvalue of λ (dual feasibility)
};

// Per-constraint dual information in Model (original) space.
//
// For linear constraint i (Ax + b >= 0):
//   slack[i]  = A_i * x[vars_i] + b_i   (should be >= 0)
//   lambda[i] = dual multiplier          (should be >= 0)
//   complementarity: lambda[i].dot(slack[i]) ≈ 0
//
// For SOC constraint i (||A₁x + b₁|| ≤ A₀x + b₀):
//   slack[i]  = A_i * x + b_i            (should be in SOC: s₀ >= ||s₁||)
//   lambda[i] = dual multiplier          (should be in SOC: λ₀ >= ||λ₁||)
//   complementarity: lambda[i].dot(slack[i]) ≈ 0
//
// For PSD constraint j (Σ A_k x_k + B ≽ 0):
//   psd_slack[j]  = Σ A_k x_k + B       (should be ≽ 0)
//   psd_lambda[j] = dual matrix          (should be ≽ 0)
//   complementarity: tr(psd_slack[j] · psd_lambda[j]) ≈ 0
//   stationarity contribution: c_k = tr(A_k · psd_lambda[j])
//
// For equality constraint k (Cx = d):
//   nu[k] = equality dual multiplier (unconstrained sign)
struct ConstraintDuals {
  std::vector<Eigen::VectorXd> lambda;       // per linear/SOC constraint
  std::vector<Eigen::VectorXd> slack;         // per linear/SOC constraint
  std::vector<Eigen::MatrixXd> psd_lambda;   // per PSD constraint
  std::vector<Eigen::MatrixXd> psd_slack;    // per PSD constraint
  std::vector<Eigen::VectorXd> nu;            // per equality constraint
  std::vector<Eigen::VectorXd> eq_residual;   // per equality: Cx - d

  // Stationarity gradient: c + Qx - A'λ - C'ν (should be ≈ 0).
  Eigen::VectorXd stationarity_gradient;
};

struct SolveResult {
  Eigen::VectorXd x;          // primal solution in Model (original) space
  double objective = 0;        // c'x + (1/2)x'Qx
  double mu = 0;               // barrier parameter at termination
  double gap = 0;              // algorithm's internal gap: |r|²-|r·d|²
  double d_inf = 0;            // ||d||_inf (feasible when <= 1)
  double tau = 1.0;            // homogenization parameter at termination
  double kappa = 0;            // theta/tau (infeasibility certificate when large)
  int iterations = 0;
  int factorizations = 0;
  bool converged = false;

  OptimalitySummary optimality;
  ConstraintDuals duals;
};

// DIMACS-like normalized errors for convergence checking.
//
// Errors are normalized by problem data norms so that a single tolerance
// (e.g. 1e-6) is meaningful across differently-scaled problems.
//
//   dual_err   = ||grad|| / max(1, ||c|| + ||Q|| * ||x||)
//   eq_err     = max_k ||C_k x - d_k|| / max(1, ||d_k||)
//   compl_err  = |<s, λ>| / max(1, |objective|)
//   prim_err   = max(0, -min_slack)  (unnormalized, zero means feasible)
//
struct DimacsErrors {
  double dual_err = 0;
  double eq_err = 0;
  double compl_err = 0;
  double prim_err = 0;

  double max_err() const {
    return std::max({dual_err, eq_err, compl_err, prim_err});
  }

  bool converged(double tol = 1e-6) const { return max_err() < tol; }
};

inline DimacsErrors ComputeDimacsErrors(const SolveResult& result) {
  DimacsErrors e;

  double x_norm = result.x.norm();

  // Dual error: ||A'λ + C'ν - Qx - c|| normalized by max(1, ||x||).
  // Use optimality.dual_residual which is computed in reduced solver space
  // and is consistent across equality-eliminated and non-eliminated problems.
  e.dual_err = result.optimality.dual_residual / std::max(1.0, x_norm);

  // Equality error: max_k ||C_k x - d_k|| normalized by max(1, ||x||).
  for (const auto& r : result.duals.eq_residual) {
    e.eq_err = std::max(e.eq_err, r.norm());
  }
  e.eq_err /= std::max(1.0, x_norm);

  // Complementarity error: |<s, λ>| normalized by max(1, |objective|).
  e.compl_err = std::abs(result.optimality.complementarity)
                / std::max(1.0, std::abs(result.objective));

  // Primal infeasibility: how negative is the most-violated slack.
  e.prim_err = std::max(0.0, -result.optimality.min_slack);

  return e;
}

}  // namespace conex
