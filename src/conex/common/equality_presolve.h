#pragma once
#include "conex/common/model.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

struct EqualityPresolveResult {
  Model reduced;           // Problem with no equality constraints.
  Eigen::MatrixXd N;       // Null space basis (n × (n-p)).
  Eigen::VectorXd x0;      // Particular solution (n × 1).
  int original_n = 0;
};

// Eliminate equality constraints by substituting x = x0 + N*z.
// Returns a reduced problem in z with no equality constraints.
// The original solution is recovered via x = x0 + N*z.
EqualityPresolveResult EliminateEqualities(const Model& problem);

// Map reduced-space solution z back to original-space x = x0 + N*z.
inline Eigen::VectorXd ExpandSolution(
    const EqualityPresolveResult& presolve,
    const Eigen::VectorXd& z) {
  return presolve.x0 + presolve.N * z;
}

}  // namespace conex
