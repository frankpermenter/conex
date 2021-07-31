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
    sys->inner_product_of_w_and_c = WC.sum();
    sys->inner_product_of_c_and_Qc = WC.squaredNorm();
    sys->inner_product_of_c_and_Qe = WC.col(0).dot(W.col(0));
    sys->inner_product_of_c_and_e = o->constraint_affine_.sum();
    if (G->rows() != m) {
      sys->setZero();
    }
    (*G).topLeftCorner(m, m).noalias() = WA.transpose() * WA;
    sys->AW.topRows(m).noalias() = o->constraint_matrix_.transpose() * W;
    sys->AQc.topRows(m).noalias() = WA.transpose() * WC;
    sys->AQe.topRows(m).noalias() =
        o->constraint_matrix_.transpose() * (W.cwiseProduct(W));
    sys->Ae.topRows(m).noalias() =
        o->constraint_matrix_.colwise().sum().transpose();
  } else {
    sys->inner_product_of_w_and_c += WC.sum();
    sys->inner_product_of_c_and_Qc += WC.squaredNorm();
    sys->inner_product_of_c_and_Qe += WC.col(0).dot(W.col(0));
    sys->inner_product_of_c_and_e += o->constraint_affine_.sum();
    (*G).topLeftCorner(m, m).noalias() += WA.transpose() * WA;
    sys->AW.topRows(m).noalias() += o->constraint_matrix_.transpose() * W;
    sys->AQc.topRows(m).noalias() += WA.transpose() * WC;
    sys->AQe.topRows(m).noalias() +=
        o->constraint_matrix_.transpose() * (W.cwiseProduct(W));
    sys->Ae.topRows(m).noalias() +=
        o->constraint_matrix_.colwise().sum().transpose();
  }
}

}  // namespace conex
