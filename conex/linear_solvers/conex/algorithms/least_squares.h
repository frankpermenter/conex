#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Solve min ||Ax - b||_2^2 via the normal equations A^T A x = A^T b.
// Uses the tree solver with automatic clique ordering.
// Drops structurally rank-deficient columns before solving.
struct SparseLeastSquaresResult {
  Eigen::VectorXd x;
  double construction_time_us;
  double assemble_and_factor_time_us;
  double solve_time_us;

  // Sub-phase breakdown of construction_time_us:
  double grouping_us;
  double add_constraints_us;
  double init_workspace_us;
  double clique_extraction_us;
  double finalize_us;
};

SparseLeastSquaresResult SparseLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

// Solve (Q + A^T A) x = rhs.
// Preprocesses isolated diagonal variables (Q(i,i) only, no A or
// off-diagonal Q) and solves them directly.
struct SparseQuadraticTermLeastSquaresResult {
  Eigen::VectorXd x;
  double construction_time_us;
  double factor_time_us;
  double solve_time_us;
};

SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquares(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquares(
    const Eigen::MatrixXd& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

}  // namespace conex
