#pragma once
#include <vector>
#include <Eigen/Dense>

namespace conex {

// Per-constraint dual information in Model (original) space.
// For linear constraint i (Ax + b >= 0):
//   slack[i]  = A_i * x[vars_i] + b_i   (should be >= 0)
//   lambda[i] = dual multiplier          (should be >= 0)
//   complementarity: lambda[i].dot(slack[i]) should be ≈ 0
//
// For equality constraint j (Cx = d):
//   nu[j] = equality dual multiplier (unconstrained sign)
struct ConstraintDuals {
  std::vector<Eigen::VectorXd> lambda;  // one per cone constraint
  std::vector<Eigen::VectorXd> slack;   // one per cone constraint
  std::vector<Eigen::VectorXd> nu;      // one per equality constraint
};

struct SolveResult {
  Eigen::VectorXd x;          // primal solution in Model (original) space
  double objective = 0;        // c'x + (1/2)x'Qx
  double mu = 0;               // barrier parameter at termination
  double dual_residual = 0;
  double complementarity = 0;
  int iterations = 0;
  int factorizations = 0;
  bool converged = false;

  ConstraintDuals duals;
};

}  // namespace conex
