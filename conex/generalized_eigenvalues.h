#include <Eigen/Dense>

namespace conex {

Eigen::VectorXd GeneralizedEigenvalues(const Eigen::MatrixXd& A,
                                       const Eigen::MatrixXd& B,
                                       const Eigen::MatrixXd& r,
                                       int num_iterations);

}  // namespace conex
