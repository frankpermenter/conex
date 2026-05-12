#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>
#include "conex/common/model.h"

namespace conex {

struct EqualityConstrainedLeastSquaresResult {
  Eigen::VectorXd x;
  double construction_time_us = 0;
  double assemble_and_factor_time_us = 0;
  double solve_time_us = 0;
};

// Solve min ||Ax - b||_2^2  subject to  Cx = d
// via the KKT system [A^T A, C^T; C, 0] [x; lambda] = [A^T b; d].
EqualityConstrainedLeastSquaresResult EqualityConstrainedLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double>& C,
    const Eigen::VectorXd& d);

// Solve a Model with only quadratic cost, linear cost, and equality
// constraints (no cone/inequality constraints).
//
//   min c'x + (1/2)x'Qx   s.t.  Cx = d
//
// via the KKT system [Q, C'; C, 0] [x; nu] = [-c; d].
// Falls back to AddLinearConstraint with a dummy identity block to
// give the tree solver a positive-definite cone contribution.
//
// Returns the primal x (not the dual nu).
struct QPEqualityResult {
  Eigen::VectorXd x;
  double objective;  // c'x + (1/2)x'Qx at the solution
  bool success;
};

QPEqualityResult SolveQPEquality(const Model& problem);

}  // namespace conex
