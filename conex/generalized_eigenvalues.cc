#include "conex/generalized_eigenvalues.h"
#include "conex/approximate_eigenvalues.h"

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Finds generalized eigenvalues of (A, B)
// where A is assumed to be positive definite.
Eigen::VectorXd GeneralizedEigenvalues(const Eigen::MatrixXd& A,
                                       const Eigen::MatrixXd& B,
                                       const Eigen::MatrixXd& r,
                                       int num_iteration) {
  Eigen::MatrixXd Ainv = A.inverse();
  // Eigenvalues of A^{-1} B
  return ApproximateEigenvalues(Ainv * B, Ainv, r, num_iteration, 1);
}

}  // namespace conex
