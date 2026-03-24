#include "conex/common/linear_constraint.h"

namespace conex {

LinearConstraint::LinearConstraint(const Eigen::MatrixXd& constraint_matrix,
                                   const Eigen::MatrixXd& constraint_affine)
    : workspace_(constraint_matrix.rows(), constraint_matrix.cols()),
      constraint_matrix_(constraint_matrix),
      constraint_affine_(constraint_affine) {
  CONEX_CHECK(constraint_matrix_.rows() == constraint_affine_.rows());
}

}  // namespace conex
