#define EIGEN_NO_MALLOC
#include "linear_constraint.h"
#include "newton_step.h"

namespace conex {

void SetIdentity(LinearConstraint* o) { o->workspace_.W.setConstant(1); }

// TODO: use e_weight and c_weight
void PrepareStep(LinearConstraint* o, const StepOptions& options, const Ref& y,
                 StepInfo* info) {
  auto* workspace = &o->workspace_;
  auto& minus_s = workspace->temp_1;
  if (!options.affine) {
    o->ComputeNegativeSlack(options.inv_sqrt_mu, y, &minus_s);
    auto& W = workspace->W;
    auto& SW = workspace->temp_1;
    auto& d = workspace->temp_2;
    SW = minus_s.cwiseProduct(W);

    int n = SW.rows();

    d = SW + DenseMatrix::Ones(n, 1);
    double norminf = (d).array().abs().maxCoeff();
    info->norminfd = norminf;
    info->normsqrd = d.squaredNorm();

  } else {
    o->ComputeNegativeSlack(0, y, &minus_s);
    TakeStep(o, options);
  }
}


void PrepareParametrizedSlack(LinearConstraint* o, const StepOptions& opt, const Ref& y1, const Ref& y2, StepInfo* data) {
  int n = o->constraint_affine_.rows();
  auto& W = o->workspace_.W;
  Eigen::VectorXd slack1(n); 
  Eigen::VectorXd slack2(n);
  Ref s1(slack1.data(), n, 1);
  Ref s2(slack2.data(), n, 1);
  o->ComputeNegativeSlack(0, y1, &s1);
  o->ComputeNegativeSlack(1, y2, &s2);

  double lower_bound_primal = -1e30;
  double upper_bound_primal = 1e30;
  double lower_bound_dual = -1e30;
  double upper_bound_dual = 1e30;

  Eigen::VectorXd SW0 = -s1.cwiseProduct(W);
  Eigen::VectorXd SW1 = -s2.cwiseProduct(W);

  for (int i = 0; i < SW0.rows(); i++) {
    double temp = -SW0(i) / SW1(i); 
    if (SW1(i) > 0) {
      // SW0 + SW1 * t >= 0 => t >= - SW0 / SW1 
      if (temp > lower_bound_primal) {
        lower_bound_primal = temp;
      }
    } else {
      // SW0 + SW1 * t >= 0 => t <= - SW0 / SW1 
      if (temp < upper_bound_primal) {
        upper_bound_primal = temp;
      }
    }

    temp = (2 - SW0(i)) / SW1(i); 
    if (SW1(i) > 0) {
      // SW0 + SW1 * t <= 1 => t <= (2- SW0) / SW1 
      if (temp < upper_bound_dual) {
        upper_bound_dual = temp;
      }
    } else {
      // SW0 + SW1 * t >= 1 => t >=  (2- SW0) / SW1 
      if (temp > lower_bound_dual) {
        lower_bound_dual = temp;
      }
    }
  }
 // Eigen::VectorXd e(n); e.setConstant(1);
  //DUMP(e - (SW0 + SW1 * lower_bound_primal));
  //DUMP(e - (SW0 + SW1 * upper_bound_primal));
  //DUMP(e - (SW0 + SW1 * lower_bound_dual));
  //DUMP(e  - (SW0 + SW1 * upper_bound_dual));
  data->inv_sqrt_mu_primal_lower_bound = lower_bound_primal;
  data->inv_sqrt_mu_dual_lower_bound = lower_bound_dual;
  data->inv_sqrt_mu_primal_upper_bound = upper_bound_primal;
  data->inv_sqrt_mu_dual_upper_bound = upper_bound_dual;
}

bool TakeStep(LinearConstraint* o, const StepOptions& options) {
  if (!options.affine) {
    auto& d = o->workspace_.temp_2;
    auto& W = o->workspace_.W;
    if (options.step_size != 1) {
      d.array() *= options.step_size;
    }
    d = d.array().exp();
    W = W.cwiseProduct(d);
  } else {
    auto& minus_s = o->workspace_.temp_1;
    o->AffineUpdate(minus_s);
  }
  return true;
}

// Eigenvalues of Q(w/2)(C - A'y).
void GetWeightedSlackEigenvalues(LinearConstraint* o, const Ref& y,
                                 WeightedSlackEigenvalues* p) {
  auto* workspace = &o->workspace_;
  auto& minus_s = workspace->temp_1;
  auto& Ws = workspace->temp_2;
  o->ComputeNegativeSlack(1, y, &minus_s);
  Ws.noalias() = workspace->W.cwiseProduct(minus_s);

  const double lamda_max = -Ws.minCoeff();
  const double lamda_min = -Ws.maxCoeff();

  p->lambda_max = lamda_max;
  p->lambda_min = lamda_min;
  p->frobenius_norm_squared = Ws.squaredNorm();
  p->trace = -Ws.sum();
}

void LinearConstraint::ComputeNegativeSlack(double inv_sqrt_mu, const Ref& y,
                                            Ref* minus_s) {
  minus_s->noalias() = (constraint_matrix_)*y.topRows(number_of_variables());
  minus_s->noalias() -= (constraint_affine_)*inv_sqrt_mu;
}

void LinearConstraint::AffineUpdate(const Ref& minus_s) {
  auto& W = workspace_.W;
  auto& SW = workspace_.temp_1;
  SW = minus_s.cwiseProduct(W);
  W += W.cwiseProduct(SW);
}

void ConstructSchurComplementSystem(LinearConstraint* o, bool initialize,
                                    SchurComplementSystem* sys) {
  const auto& W = o->workspace_.W;
  auto G = &sys->G;

  auto& WA = o->workspace_.weighted_constraints;
  auto& WC = o->workspace_.temp_1;
  int m = o->number_of_variables();

  WA = W.asDiagonal() * (o->constraint_matrix_);
  WC = W.cwiseProduct(o->constraint_affine_);

  if (initialize) {
    sys->inner_product_of_w_and_c = WC.sum();
    if (G->rows() != m) {
      G->setZero();
      sys->AW.setZero();
      sys->AQc.setZero();
    }
    (*G).topLeftCorner(m, m).noalias() = WA.transpose() * WA;
    sys->AW.topRows(m).noalias() = o->constraint_matrix_.transpose() * W;
    sys->AQc.topRows(m).noalias() = WA.transpose() * WC;
  } else {
    sys->inner_product_of_w_and_c += WC.sum();
    (*G).topLeftCorner(m, m).noalias() += WA.transpose() * WA;
    sys->AW.topRows(m).noalias() += o->constraint_matrix_.transpose() * W;
    sys->AQc.topRows(m).noalias() += WA.transpose() * WC;
  }
}

}  // namespace conex
