#pragma once
#include <Eigen/Dense>

namespace conex {

struct SolveResult {
  Eigen::VectorXd x;          // primal solution in Model (original) space
  double objective = 0;        // c'x + (1/2)x'Qx
  double mu = 0;               // barrier parameter at termination
  double dual_residual = 0;
  double complementarity = 0;
  int iterations = 0;
  int factorizations = 0;
  bool converged = false;
};

}  // namespace conex
