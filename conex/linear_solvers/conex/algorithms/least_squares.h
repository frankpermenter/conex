// Deprecated: Use Problem + Solver API instead.
// This file is kept for backward compatibility with profile_mtx
// and equality_constrained_least_squares.cc which still call
// SparseLeastSquares internally.
#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

struct SparseLeastSquaresResult {
  Eigen::VectorXd x;
  double construction_time_us;
  double assemble_and_factor_time_us;
  double solve_time_us;
  double grouping_us;
  double add_constraints_us;
  double init_workspace_us;
  double clique_extraction_us;
  double finalize_us;
};

SparseLeastSquaresResult SparseLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

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
