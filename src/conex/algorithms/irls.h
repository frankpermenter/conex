#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>

namespace conex {

// Iteratively Reweighted Least Squares (IRLS) for L1-approximation.
//
// Solves: min ||A x - b||_1  (L1-norm minimization)
// via iterating: min ||W^{1/2} (A x - b)||_2
// where W_ii = 1 / max(|r_i|, epsilon), r = A x - b.
//
// Uses the tree solver's reweighting support: builds the solver once,
// then calls SetWeights + AssembleAndFactor + Solve in a loop.
struct IRLSResult {
  Eigen::VectorXd x;
  int iterations;
  double l1_objective;
  double solve_time_us;
};

IRLSResult SolveIRLS(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    int max_iterations = 50,
    double epsilon = 1e-6,
    double tolerance = 1e-8);

}  // namespace conex
