#include "linear_constraint.h"

#include "conex.h"
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
void AppendRow(MatrixXd* A, const MatrixXd& new_rows) {
  int num_cols = A->cols();
  if (A->rows() == 0) {
    num_cols = new_rows.cols();
  }
  if (num_cols != new_rows.cols()) {
    throw std::runtime_error(
        "Cannot stack matrices with different number of columns.");
  }
  A->conservativeResize(A->rows() + new_rows.rows(), num_cols);
  A->bottomRows(new_rows.rows()) = new_rows;
}

void PreprocessLinearInequality(const MatrixXd& A, const MatrixXd& lb,
                                const MatrixXd& ub, MatrixXd* Aineq,
                                MatrixXd* bineq, MatrixXd* Aeq, MatrixXd* beq,
                                double rescale) {
  for (int i = 0; i < A.rows(); i++) {
    if (lb.row(i) == ub.row(i)) {
      double scale = 1.0;
      if (rescale) {
        scale = 1.0 / std::sqrt(A.row(i).squaredNorm());
      }
      if (std::isfinite(scale)) {
        AppendRow(Aeq, scale * A.row(i));
        AppendRow(beq, scale * ub.row(i));
      }
    } else {
      if (ub(i, 0) < 1e8) {
        double scale = 1.0;
        if (rescale) {
          scale = 1.0 / std::sqrt(A.row(i).squaredNorm());
        }
        if (std::isfinite(scale)) {
          AppendRow(Aineq, scale * A.row(i));
          AppendRow(bineq, scale * ub.row(i));
        }
      }
      if (lb(i, 0) > -1e8) {
        double scale = 1.0;
        if (rescale) {
          scale = 1.0 / std::sqrt(A.row(i).squaredNorm());
        }
        if (std::isfinite(scale)) {
          AppendRow(Aineq, -scale * A.row(i));
          AppendRow(bineq, -scale * lb.row(i));
        }
      }
    }
  }
}

template <typename T>
bool FindMinimumMu(const T& d0, const T& delta, double dinfmax,
                   LineSearchOutput* output) {
  auto& upper_bound = output->upper_bound;
  auto& lower_bound = output->lower_bound;
  double upper_bound_i;
  double lower_bound_i;
  double temp;
  int i = 0;

  for (i = 0; i < d0.size(); i++) {
    upper_bound_i = (dinfmax - d0(i)) / delta(i);
    lower_bound_i = (-dinfmax - d0(i)) / delta(i);

    if (lower_bound_i > upper_bound_i) {
      temp = upper_bound_i;
      upper_bound_i = lower_bound_i;
      lower_bound_i = temp;
    }

    if (upper_bound_i < upper_bound || i == 0) {
      upper_bound = upper_bound_i;
    }

    if (lower_bound_i > lower_bound || i == 0) {
      lower_bound = lower_bound_i;
    }
  }
  output->d0_dot_dt = d0.col(0).dot(delta.col(0));
  output->dt_squared_norm = delta.col(0).dot(delta.col(0));
  output->d0_squared_norm = d0.col(0).dot(d0.col(0));

  bool success = true;
  if (lower_bound > upper_bound) {
    success = false;
  }
  return success;
}

bool LinearConstraint::PerformLineSearchImpl(
    const LineSearchParameters& params,
    const Eigen::Ref<const Eigen::MatrixXd>& y0,
    const Eigen::Ref<const Eigen::MatrixXd>& y1, LineSearchOutput* output) {
  auto* workspace = &workspace_;

  auto& d0 = workspace->temp_1;
  auto& d1 = workspace->temp_2;

  // d =  e + w \circ (A'y  - c k_1)
  ComputeNegativeSlack(params.options_0.c_weight, y0, d0);
  d0.array() -= params.options_0.w_weight;
  d0 = d0.cwiseProduct(workspace_.W);
  d0 = d0.cwiseQuotient(workspace_.r);
  d0.array() += params.options_0.e_weight;

  ComputeNegativeSlack(params.options_1.c_weight, y1, d1);
  d1.array() -= params.options_1.w_weight;
  d1 = d1.cwiseProduct(workspace_.W);
  d1 = d1.cwiseQuotient(workspace_.r);
  d1.array() += params.options_1.e_weight;

  d1 = d1 - d0;

  bool success = FindMinimumMu(d0, d1, params.dinf_upper_bound, output);
  return !success;
}

void LinearConstraint::SetIdentityImpl() {
  workspace_.W.setConstant(1);
  workspace_.r.setConstant(1);
  if (gram_evaluator_.is_active()) {
    gram_evaluator_.update_weights();
  }
}

// TODO: use e_weight and c_weight
void LinearConstraint::PrepareStepImpl(
    const StepOptions& options, const Eigen::Ref<const Eigen::MatrixXd>& y,
    StepInfo* info) {
  auto* workspace = &workspace_;
  auto& d = workspace->temp_2;

  // d =  e + w \circ ( A'y  - c k_1 - k_0 e)
  ComputeNegativeSlack(options.c_weight, y, d);
  d.array() -= options.w_weight;
  d = d.cwiseProduct(workspace_.W);
  d = d.cwiseQuotient(workspace_.r);
  d.array() += options.e_weight;

  double norminf = (d).array().abs().maxCoeff();
  info->norminfd = norminf;
  VectorXd rd = d.col(0).cwiseProduct(workspace_.r);
  info->normsqrd = rd.squaredNorm();
}

bool LinearConstraint::TakeStepImpl(const StepOptions& options) {
  auto& d = workspace_.temp_2;
  auto& W = workspace_.W;
  bool use_geodesic = options.step_type == CONEX_STEP_TYPE_GEODESIC;
  VectorXd r = workspace_.r;
  if (options.update_scaling) {
    for (int i = 0; i < d.rows(); i++) {
#if 0
      if (std::abs(d(i)) < 1) {
        workspace_.r(i) = std::sqrt(product(i));
        //workspace_.r(i) *= std::sqrt( (1 - d(i)) * (1 + d(i)));
        //workspace_.r(i) = std::sqrt(workspace_.r(i));
      }
#else
      // Absorb max(1-d, 1+d) into r.
      double max_scale = 1e5;
      double min_scale = 1e-5;
      if (d(i) < 0) {
        double scale = 1 - d(i) * options.step_size;
        if (scale > max_scale) {
          scale = max_scale;
        }
        if (scale < min_scale) {
          scale = min_scale;
        }
        workspace_.r(i) *= scale;
      } else {
        double scale = 1 + d(i) * options.step_size;
        if (scale > max_scale) {
          scale = max_scale;
        }
        workspace_.r(i) *= scale;
      }
#endif
    }
    // This normalization necessarity to ensure <r, r> = rank K
    workspace_.r =
        workspace_.r / workspace_.r.norm() * std::sqrt(workspace_.r.rows());
  } else {
    if (use_geodesic) {
      if (options.step_size != 1) {
        d.array() *= options.step_size;
      }
      d = d.array().exp();
      W = W.cwiseProduct(d);
    } else {
      AffineUpdate(d, options.step_type);
    }
  }
  if (gram_evaluator_.is_active()) {
    gram_evaluator_.update_weights();
  }
  return true;
}

// Eigenvalues of Q(w/2)(C - A'y).
void LinearConstraint::GetWeightedSlackEigenvaluesImpl(
    const Ref& y, double c_weight, WeightedSlackEigenvalues* p) {
  auto* workspace = &workspace_;
  auto& minus_s = workspace->temp_1;
  auto& Ws = workspace->temp_2;
  ComputeNegativeSlack(c_weight, y, minus_s);
  Ws.noalias() = workspace->W.cwiseProduct(minus_s);

  const double lamda_max = -Ws.minCoeff();
  const double lamda_min = -Ws.maxCoeff();

  p->lambda_max = lamda_max;
  p->lambda_min = lamda_min;
  p->frobenius_norm_squared = Ws.squaredNorm();
  p->trace = -Ws.sum();
}

void LinearConstraint::ComputeNegativeSlack(
    double inv_sqrt_mu, const Eigen::Ref<const Eigen::MatrixXd>& y,
    Eigen::Ref<Eigen::MatrixXd> minus_s) {
  minus_s.noalias() = (constraint_matrix_)*y.topRows(number_of_variables());
  minus_s.noalias() -= (constraint_affine_)*inv_sqrt_mu;
}

void LinearConstraint::AffineUpdate(const Eigen::Ref<const Eigen::MatrixXd>& d,
                                    int step_type) {
  auto& W = workspace_.W;
  bool dual_barrier = step_type == CONEX_STEP_TYPE_DUAL_BARRIER;
  if (!dual_barrier) {
    CONEX_CHECK(step_type == CONEX_STEP_TYPE_PRIMAL_BARRIER);
    VectorXd Winv = W.cwiseInverse();
    Winv -= Winv.cwiseProduct(d);
    W = Winv.cwiseInverse();
  } else {
    W += W.cwiseProduct(d);
    // TODO(FrankPermenter): remove this hack.
    // We scale by r so that AffineStep in PrepareDualVariables
    // will place the dual variable in workspace_.W;
    // We need workspace_.W  to hold the dual variable
    // because of the dual variable interface.
    W = W.cwiseProduct(workspace_.r);
  }
}

void LinearConstraint::ApplyRescalingImpl(Eigen::Ref<Eigen::MatrixXd> Aw,
                                          double* ip) {
  const auto& W = workspace_.W;
  const auto& r = workspace_.r;
  *ip += W.cwiseProduct(r).col(0).dot(constraint_affine_.col(0));
  int m = number_of_variables();
  Aw.topRows(m).noalias() += constraint_matrix_.transpose() * r.cwiseProduct(W);
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
    sys->AW.topRows(m).noalias() =
        constraint_matrix_.transpose() * r.cwiseProduct(W);
    sys->AQc.topRows(m).noalias() = WA.transpose() * WC;
    sys->AQe.topRows(m).noalias() =
        constraint_matrix_.transpose() * (W.cwiseProduct(W));
    sys->Ae.topRows(m).noalias() =
        constraint_matrix_.colwise().sum().transpose();
  } else {
    std::runtime_error("obsolete");
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

CONEX_STATUS LinearConstraint::UpdateLinearOperatorImpl(double val, int var,
                                                        int r, int c, int dim) {
  CONEX_RETURN_ON_FAIL(dim == 0, "Complex linear constraints not supported.");
  CONEX_RETURN_ON_FAIL(c == 0, "Linear constraint is not matrix valued.");
  CONEX_RETURN_ON_FAIL(r < constraint_matrix_.rows(),
                       "Row index out of bounds.");
  CONEX_RETURN_ON_FAIL((var >= 0) && (r >= 0), "Indices cannot be negative.");

  constraint_matrix_(r, var) = val;
  gram_evaluator_.invalidate_order();
  return CONEX_SUCCESS;
}

CONEX_STATUS LinearConstraint::UpdateAffineTermImpl(double val, int r, int c,
                                                    int dim) {
  CONEX_RETURN_ON_FAIL(dim == 0, "Complex linear cone not supported.");
  CONEX_RETURN_ON_FAIL(c == 0, "Linear constraint is not matrix valued.");
  CONEX_RETURN_ON_FAIL(r < constraint_matrix_.rows(),
                       "Row index out of bounds.");
  CONEX_RETURN_ON_FAIL(r >= 0, "Indices cannot be negative.");

  constraint_affine_(r) = val;
  return CONEX_SUCCESS;
}

}  // namespace conex
