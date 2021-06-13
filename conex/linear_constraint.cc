#define EIGEN_NO_MALLOC
#include "linear_constraint.h"
#include "newton_step.h"

namespace conex {
using Eigen::MatrixXd;
using Eigen::VectorXd;
void SetIdentity(LinearConstraint* o) { o->workspace_.W.setConstant(1); }

// TODO: use e_weight and c_weight
void PrepareStep(LinearConstraint* o, const StepOptions& options, const Ref& y,
                 StepInfo* info) {
  auto* workspace = &o->workspace_;
  auto& minus_s = workspace->temp_1;
  auto& SW0 = o->workspace_.temp_1;
  auto& SW1 = o->workspace_.temp_2;

  if (!options.affine) {
    auto& d = workspace->temp_2;

    o->ComputeNegativeSlack(options.c_weight, y, &d);
    d.array() -= options.w_weight;
    d = d.cwiseProduct(o->workspace_.W);

    d.array() += options.e_weight;

    double norminf = (d).array().abs().maxCoeff();
    info->norminfd = norminf;
    info->normsqrd = d.squaredNorm();

  } else {
    o->ComputeNegativeSlack(0, y, &minus_s);
    TakeStep(o, options);
  }
}

bool DoPrimalDualLineSearch(LinearConstraint* o, const LineSearchParameters& p,
                            StepInfo* data) {
  double dinf_limit = p.dinf_limit;
  double lower_bound_primal = -std::numeric_limits<double>::max();
  double upper_bound_primal = std::numeric_limits<double>::max();
  double lower_bound_dual = -std::numeric_limits<double>::max();
  double upper_bound_dual = std::numeric_limits<double>::max();
  const auto& SW0 = o->workspace_.temp_1;
  const auto& SW1 = o->workspace_.temp_2;

  VectorXd SW2 = -(SW0 + SW1 * p.inv_sqrt_mu);

  SW2 = -SW1.cwiseProduct(SW2.cwiseInverse());
  // DUMP(1.0/(p.inv_sqrt_mu * p.inv_sqrt_mu));
  // DUMP(-1.0/SW2.minCoeff() + p.inv_sqrt_mu);
  double upper_bound_primal_2 = -1.0 / SW2.minCoeff() + p.inv_sqrt_mu;

  // 2 + (SW0 + SW1 t)  >= 0
  SW2.setConstant(2);
  SW2 += SW0 + SW1 * p.inv_sqrt_mu;
  SW2 = SW1.cwiseProduct(SW2.cwiseInverse());
  double upper_bound_dual_2 = -1.0 / SW2.minCoeff() + p.inv_sqrt_mu;
  // DUMP(SW0 + SW1 * upper_bound_dual_2);
  // DUMP(SW0 + SW1 * upper_bound_primal_2);

  for (int i = 0; i < SW0.rows(); i++) {
    // e + At * y - k * c
    double temp = (-dinf_limit + 1 + SW0(i)) / -SW1(i);
    if (-SW1(i) > 0) {
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

    temp = (dinf_limit + 1 + SW0(i)) / -SW1(i);
    if (-SW1(i) > 0) {
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
  // DUMP(SW1);
  // DUMP(std::numeric_limits<double>::min());
  // DUMP(e - (SW0 + upper_bound_primal*SW1));
  // DUMP(e - (SW0 + lower_bound_primal*SW1));
  // DUMP(e - (SW0 + upper_bound_dual*SW1));
  // DUMP(e - (SW0 + lower_bound_dual*SW1));
  // DUMP(upper_bound_dual);
  // DUMP(lower_bound_primal);
  // DUMP(upper_bound_primal);
  // DUMP(lower_bound_dual);
  data->inv_sqrt_mu_primal_lower_bound = lower_bound_primal;
  data->inv_sqrt_mu_dual_lower_bound = lower_bound_dual;
  // data->inv_sqrt_mu_primal_upper_bound = upper_bound_primal;
  data->inv_sqrt_mu_primal_upper_bound = upper_bound_primal;
  data->inv_sqrt_mu_dual_upper_bound = upper_bound_dual;
  return false;
}

void PrepareParametrizedSlack(LinearConstraint* o, const SlackWeights& p1,
                              const Ref& y1, const SlackWeights& p2,
                              const Ref& y2) {
  auto& W = o->workspace_.W;
  auto& s1 = o->workspace_.temp_1;
  auto& s2 = o->workspace_.temp_2;
  auto& SW0 = o->workspace_.temp_1;
  auto& SW1 = o->workspace_.temp_2;
  o->ComputeNegativeSlack(p1.c_weight, y1, &s1);
  o->ComputeNegativeSlack(p2.c_weight, y2, &s2);

  SW0 = s1.cwiseProduct(W);
  SW1 = s2.cwiseProduct(W);
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

// Eigenvalues of Q(w/2)(k C - A'y).
void GetWeightedSlackEigenvalues(LinearConstraint* o, const Ref& y,
                                 double c_weight, WeightedSlackEigenvalues* p) {
  auto* workspace = &o->workspace_;
  auto& minus_s = workspace->temp_1;
  auto& Ws = workspace->temp_2;
  o->ComputeNegativeSlack(c_weight, y, &minus_s);
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
    if (G->rows() != m) {
      G->setZero();
      sys->setZero();
    }
    (*G).topLeftCorner(m, m).noalias() = WA.transpose() * WA;

    sys->inner_product_of_c_and_w = WC.sum();
    sys->inner_product_of_c_and_Qc = WC.squaredNorm();
    sys->inner_product_of_c_and_Qe = WC.col(0).dot(W.col(0));
    sys->AW.topRows(m).noalias() = o->constraint_matrix_.transpose() * W;
    sys->AQc.topRows(m).noalias() = WA.transpose() * WC;
    sys->AQe.topRows(m).noalias() =
        o->constraint_matrix_.transpose() * (W.cwiseProduct(W));

    // TODO(FrankPermenter): cache this quantity
    sys->inner_product_of_c_and_e = o->constraint_affine_.sum();
    sys->Ae.topRows(m).noalias() =
        o->constraint_matrix_.colwise().sum().transpose();
  } else {
    (*G).topLeftCorner(m, m).noalias() += WA.transpose() * WA;

    sys->inner_product_of_c_and_w += WC.sum();
    sys->inner_product_of_c_and_Qc += WC.squaredNorm();
    sys->inner_product_of_c_and_Qe += WC.col(0).dot(W.col(0));
    sys->inner_product_of_c_and_e += o->constraint_affine_.sum();

    sys->AW.topRows(m).noalias() += o->constraint_matrix_.transpose() * W;
    sys->AQc.topRows(m).noalias() += WA.transpose() * WC;

    // TODO(FrankPermenter): cache this quantity
    sys->AQe.topRows(m).noalias() +=
        o->constraint_matrix_.transpose() * (W.cwiseProduct(W));
    sys->Ae.topRows(m).noalias() +=
        o->constraint_matrix_.colwise().sum().transpose();
  }
}

}  // namespace conex
