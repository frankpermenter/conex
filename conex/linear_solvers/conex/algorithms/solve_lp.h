// Solve a linear program and verify feasibility.

#pragma once
#include <Eigen/Dense>
#include "conex/common/problem.h"

namespace conex {

struct LPResult {
  Eigen::VectorXd x;
  double objective;
  double gap;
  int factorizations;
  int solves;
};

// Solve the LP defined by problem.  Problem must have:
//   - A linear cost (via SetLinearCost)
//   - One or more linear constraints (via AddLinearConstraint)
// Uses the geodesic IPM (0 centering steps).
LPResult SolveLP(const Problem& problem, double tolerance = 1e-8);

// Compute the maximum constraint violation for x.
// Returns min_i(b_stored_i - A_stored_i * x) over all linear constraints.
// Non-negative means feasible.
double ComputeConstraintViolation(const Problem& problem,
                                  const Eigen::VectorXd& x);

}  // namespace conex
