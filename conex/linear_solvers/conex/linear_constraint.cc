#include "linear_constraint.h"

#include "newton_step.h"

namespace conex {
using Eigen::VectorXd;

using Eigen::MatrixXd;

LinearConstraint::LinearConstraint(const Eigen::MatrixXd& constraint_matrix,
                                   const Eigen::MatrixXd& constraint_affine)
    : workspace_(constraint_matrix.rows(), constraint_matrix.cols()),
      constraint_matrix_(constraint_matrix),
      constraint_affine_(constraint_affine) {
  CONEX_CHECK(constraint_matrix_.rows() == constraint_affine_.rows());
}

void LinearConstraint::ConstructSchurComplementSystemImpl(
    bool initialize, SchurComplementSystem* sys) {
  const auto& W = workspace_.W;
  const auto& r = workspace_.r;
  auto G = &sys->G;

  auto& WA = workspace_.weighted_constraints;
  auto& WC = workspace_.temp_1;
  int m = number_of_variables();

  WA = W.asDiagonal() * (constraint_matrix_);
  WC = W.cwiseProduct(constraint_affine_);

  if (initialize) {
    sys->inner_product_of_w_and_c =
        W.cwiseProduct(r).col(0).dot(constraint_affine_.col(0));
    sys->inner_product_of_c_and_Qc = WC.squaredNorm();
    sys->inner_product_of_c_and_Qe = WC.col(0).dot(W.col(0));
    sys->inner_product_of_c_and_e = constraint_affine_.sum();
    if (G->rows() != m) {
      sys->setZero();
    }
    (*G).topLeftCorner(m, m).noalias() = WA.transpose() * WA;
    sys->AQc.topRows(m).noalias() = WA.transpose() * WC;
    sys->AW.topRows(m).noalias() =
        constraint_matrix_.transpose() * r.cwiseProduct(W);
    sys->AQe.topRows(m).noalias() =
        constraint_matrix_.transpose() * (W.cwiseProduct(W));
    sys->Ae.topRows(m).noalias() =
        constraint_matrix_.colwise().sum().transpose();
  } else {
    std::runtime_error("obsolete");
    const auto& WA = workspace_.weighted_constraints;
    sys->inner_product_of_w_and_c += WC.sum();
    sys->inner_product_of_c_and_Qc += WC.squaredNorm();
    sys->inner_product_of_c_and_Qe += WC.col(0).dot(W.col(0));
    sys->inner_product_of_c_and_e += constraint_affine_.sum();
    (*G).topLeftCorner(m, m).noalias() += WA.transpose() * WA;
    sys->AW.topRows(m).noalias() += constraint_matrix_.transpose() * W;
    sys->AQc.topRows(m).noalias() += WA.transpose() * WC;
    sys->AQe.topRows(m).noalias() +=
        constraint_matrix_.transpose() * (W.cwiseProduct(W));
    sys->Ae.topRows(m).noalias() +=
        constraint_matrix_.colwise().sum().transpose();
  }
}

}  // namespace conex
