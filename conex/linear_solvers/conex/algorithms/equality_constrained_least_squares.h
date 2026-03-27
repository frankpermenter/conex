#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Solve min ||Ax - b||_2^2  subject to  Cx = d
// via the KKT system [A^T A, C^T; C, 0] [x; lambda] = [A^T b; d].
// Uses the tree solver with automatic clique ordering.
struct EqualityConstrainedLeastSquaresResult {
  Eigen::VectorXd x;
  double construction_time_us;
  double assemble_and_factor_time_us;
  double solve_time_us;
};

EqualityConstrainedLeastSquaresResult EqualityConstrainedLeastSquares(
    const Eigen::SparseMatrix<double>& A,  // m x n, least-squares matrix
    const Eigen::VectorXd& b,              // m, target vector
    const Eigen::MatrixXd& C,              // p x n, equality constraint matrix
    const Eigen::VectorXd& d);             // p, equality constraint RHS

}  // namespace conex
