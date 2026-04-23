#pragma once
#include <vector>
#include <Eigen/Dense>

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

}  // namespace conex
