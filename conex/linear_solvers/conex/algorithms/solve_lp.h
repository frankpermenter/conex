// Solve a linear program:  min c^T x  s.t.  Ax <= b.
// Builds the solver from Problem internally.

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

}  // namespace conex
