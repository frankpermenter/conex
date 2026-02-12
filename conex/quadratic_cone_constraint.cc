#include "conex/quadratic_cone_constraint.h"

#include "conex/newton_step.h"

namespace conex {

using EigenType = DenseMatrix;
using Real = double;

namespace {

template <typename T1, typename T2, typename T3>
double SquaredNorm(const T1& Q, const T2& x, T3* workspace) {
  if (Q.rows() > 0) {
    workspace->noalias() = Q * x;
    return x.col(0).dot(workspace->col(0));
  } else {
    return x.col(0).dot(x.col(0));
  }
}

double square_root(const double& x) { return std::sqrt(std::fabs(x)); }

template <typename T1, typename T2, typename T3>
double Norm(const T1& Q, const T2& x, T3* workspace) {
  return square_root(SquaredNorm(Q, x, workspace));
}

template <typename T1, typename T2, typename T4, typename T3>
double InnerProduct(const T1& Q, const T2& x, const T3& y, T4* workspace) {
  if (Q.rows() > 0) {
    workspace->noalias() = Q * y;
    return x.col(0).dot(workspace->col(0));
  } else {
    return x.col(0).dot(y.col(0));
  }
}

template <typename T1, typename T2, typename T3>
void QuadraticRepresentation(double x1_norm_squared,
                             double inner_product_of_x1_and_y1, double x0,
                             const T1& x1, double y0, const T2& y1,
                             double* z_q0, T3* z_q1) {
  // We use the formula from Example 11.12 of "Formally Real Jordan Algebras
  // and Their Applications to Optimization"  by Alizadeh, which states the
  // quadratic representation of x equals the linear map
  //                          2xx' - (det x) * R
  // where R is the reflection operator R = diag(1, -1, ..., -1) and det x is
  // the determinate of x = (x0, x1), i.e., det x = x0^2 - |x1|^2.
  double det_x = x0 * x0 - x1_norm_squared;
  double scale = 2 * (x0 * y0 + inner_product_of_x1_and_y1);
  *z_q0 = scale * x0 - det_x * y0;
  z_q1->noalias() = scale * x1 + det_x * y1;
}

template <typename T>
void Exp(double norm_x1, double* x0, T* x1) {
  double k = norm_x1;
  if (k > 0) {
    (*x1) *= .5 * (exp(*x0 + k) - exp(*x0 - k)) / k;
  }
  (*x0) = (.5 * (exp(*x0 + k) + exp(*x0 - k)));
}

template <typename T>
void Sqrt(double norm_x1, double* x0, T* x1) {
  double k = norm_x1;
  if (k > 0) {
    (*x1) *= .5 * (square_root(*x0 + k) - square_root(*x0 - k)) / k;
  }
  (*x0) = (.5 * (square_root(*x0 + k) + square_root(*x0 - k)));
}

Eigen::Vector2d Eigenvalues(double norm_of_x1, double x0) {
  Eigen::Vector2d eigenvalues(2, 1);
  eigenvalues(0) = x0 + norm_of_x1;
  eigenvalues(1) = x0 - norm_of_x1;
  return eigenvalues;
}

template <typename T>
void SchurComplement(const Eigen::VectorXd& A0, const Eigen::MatrixXd& A_gram,
                     const double W0, double det_w,
                     const Eigen::MatrixXd& A_dot_w, bool initialize, T* G) {
  if (initialize) {
    G->noalias() = A0 * A0.transpose();
    G->noalias() -= A_gram;
    G->array() *= -det_w;
  } else {
    G->noalias() += det_w * (A_gram - A0 * A0.transpose());
  }
  G->noalias() += (A_dot_w + A0 * W0) * (A_dot_w + A0 * W0).transpose();
  G->noalias() += (A_dot_w + A0 * W0) * (A_dot_w + A0 * W0).transpose();
}

}  // namespace

DenseMatrix QuadraticConstraintBase::EvalAtQX(const DenseMatrix& X,
                                              DenseMatrix* QX) {
  if (Q_.rows() > 0) {
    QX->noalias() = Q_ * X;
    return A1_.transpose() * (*QX);
  } else {
    return A1_.transpose() * X;
  }
}

DenseMatrix QuadraticConstraintBase::EvalAtQX(const DenseMatrix& X,
                                              NonConstRefType QX) {
  if (Q_.rows() > 0) {
    QX.noalias() = Q_ * X;
    return A1_.transpose() * (QX);
  } else {
    return A1_.transpose() * X;
  }
}

double QuadraticConstraintBase::EvalCQX(const DenseMatrix& X,
                                        NonConstRefType QX) {
  if (Q_.rows() > 0) {
    QX.noalias() = Q_ * X;
    return C1_.dot(QX.col(0));
  } else {
    return C1_.dot(X.col(0));
  }
}

void QuadraticConstraintBase::ComputeNegativeSlack(double inv_sqrt_mu,
                                                   const RefType& y,
                                                   double* minus_s_0,
                                                   NonConstRefType minus_s_1) {
  *minus_s_0 = A0_.dot(y.col(0));
  *minus_s_0 -= C0_ * inv_sqrt_mu;
  minus_s_1.noalias() = A1_ * y;
  minus_s_1.noalias() -= C1_ * inv_sqrt_mu;
}

// Combine this with PrepareStep
void QuadraticConstraintBase::GetWeightedSlackEigenvaluesImpl(const RefType& y,
                                 double c_weight, WeightedSlackEigenvalues* p) {
  auto* workspace = &workspace_;
  auto& minus_s_1 = workspace->temp1_1;
  double minus_s_0;
  ComputeNegativeSlack(c_weight, y, &minus_s_0, minus_s_1);

  auto& Ws_1 = workspace->temp2_1;
  double Ws_0;
  QuadraticRepresentation(
      workspace_.wsqrt_q1_norm_sqr,
      InnerProduct(Q_, workspace->sqrtW_1, minus_s_1, &workspace->temp2_1),
      *workspace->sqrtW_0, workspace->sqrtW_1, minus_s_0, minus_s_1, &Ws_0,
      &Ws_1);
  auto ev = Eigenvalues(Norm(Q_, Ws_1, &workspace->temp1_1), Ws_0);

  const double lamda_max = -ev.minCoeff();
  const double lamda_min = -ev.maxCoeff();

  p->lambda_max = lamda_max;
  p->lambda_min = lamda_min;
  p->frobenius_norm_squared = std::pow(lamda_max, 2) + std::pow(lamda_min, 2);
  p->trace = (lamda_max + lamda_min);
}

void QuadraticConstraintBase::ComputeNewtonDirection(
    const StepOptions opts, const RefType& y, double* d_q0,
    Eigen::Ref<Eigen::MatrixXd> d_q1) {
  auto workspace = &workspace_;
  auto& minus_s_1 = workspace_.temp1_1;
  double minus_s_0;
  ComputeNegativeSlack(opts.c_weight, y, &minus_s_0, minus_s_1);

  minus_s_0 -= opts.w_weight;
  QuadraticRepresentation(workspace_.wsqrt_q1_norm_sqr,
                          InnerProduct(Q_, workspace->sqrtW_1, minus_s_1,
                                       &workspace_.temp3_1),
                          *workspace->sqrtW_0, workspace->sqrtW_1, minus_s_0,
                          minus_s_1, d_q0, &d_q1);
  *d_q0 += opts.e_weight;
}

void QuadraticConstraintBase::PrepareStepImpl(const StepOptions& opt,
                 const RefType& y, StepInfo* info) {
  auto& d_q1 = workspace_.temp2_1;
  double& d_q0 = workspace_.d0;

  ComputeNewtonDirection(opt, y, &d_q0, d_q1);

  // Compute rescaling.
  auto ev = Eigenvalues(Norm(Q_, d_q1, &workspace_.temp1_1), d_q0);
  info->norminfd = std::fabs(ev(0));
  if (info->norminfd < std::fabs(ev(1))) {
    info->norminfd = std::fabs(ev(1));
  }
  info->normsqrd = ev.squaredNorm();
}

namespace {

std::vector<double> SolveNormEquationsPlus(double a, double x0, double x1,
                                           double y0, double y1, double k) {
  std::vector<double> t;
  t.reserve(2);
  double a_squared = a * a;
  double under_radical = a_squared * y1 + 2 * a * k * y0 - 2 * a * x0 * y1 +
                         k * k - 2 * k * x0 * y0 + x0 * x0 * y1 + x1 * y0 * y0 -
                         x1 * y1;

  if (under_radical > 1e-16) {
    t.push_back((-sqrt(under_radical) + a * y0 + k - x0 * y0) / (y0 * y0 - y1));
    t.push_back((sqrt(under_radical) + a * y0 + k - x0 * y0) / (y0 * y0 - y1));
  } else {
    if (under_radical >= 0) {
      t.push_back((a * y0 + k - x0 * y0) / (y0 * y0 - y1));
    }
  }

  return t;
}

Eigen::VectorXd GetCandidateK(double dinfmax, double x0, double x1, double y0,
                              double y1, double k) {
  using Eigen::VectorXd;
  std::vector<double> t = SolveNormEquationsPlus(dinfmax, x0, x1, y0, y1, k);
  std::vector<double> val;
  double eps = 0.01;
  for (size_t i = 0; i < t.size(); i++) {
    double ti = t[i];
    double error_minus =
        x0 + ti * y0 - sqrt(x1 + 2 * ti * k + ti * ti * y1) + dinfmax;
    double error_plus =
        x0 + ti * y0 + sqrt(x1 + 2 * ti * k + ti * ti * y1) - dinfmax;

    if ((fabs(error_plus) < eps && error_minus > -eps) ||
        (fabs(error_minus) < eps && error_plus < eps)) {
      val.push_back(ti);
    }
  }
  t = SolveNormEquationsPlus(-dinfmax, x0, x1, y0, y1, k);
  for (size_t i = 0; i < t.size(); i++) {
    double ti = t[i];
    double error_minus =
        x0 + ti * y0 - sqrt(x1 + 2 * ti * k + ti * ti * y1) + dinfmax;
    double error_plus =
        x0 + ti * y0 + sqrt(x1 + 2 * ti * k + ti * ti * y1) - dinfmax;

    if ((fabs(error_plus) < eps && error_minus > -eps) ||
        (fabs(error_minus) < eps && error_plus < eps)) {
      val.push_back(ti);
    }
  }
  return Eigen::Map<const VectorXd>(val.data(), static_cast<int>(val.size()));
}

double GetMinSqrtMu(double dinfmax, const double& x0,
                    const double& x1_squared_norm, const double& y0,
                    const double& y1_squared_norm, const double x1_dot_y1,
                    LineSearchOutput* output) {
  double upper_bound = 1e45;
  double lower_bound = -1e45;
  auto t = GetCandidateK(dinfmax, x0, x1_squared_norm, y0, y1_squared_norm,
                         x1_dot_y1);

  if (t.size() < 2) {
    // Force failure
    upper_bound = -1;
    lower_bound = 1;
  } else {
    double lower_bound_i = t.minCoeff();
    double upper_bound_i = t.maxCoeff();

    if (lower_bound_i > lower_bound) {
      lower_bound = lower_bound_i;
    }
    if (upper_bound_i < upper_bound) {
      upper_bound = upper_bound_i;
    }
  }

  output->lower_bound = lower_bound;
  output->upper_bound = upper_bound;

  return upper_bound;
}

}  // namespace

bool QuadraticConstraintBase::PerformLineSearchImpl(
    const LineSearchParameters& params, const RefType& y0,
                       const RefType& y1, LineSearchOutput* output) {
  int n = workspace_.n_;
  double d0_0;
  Eigen::VectorXd d0_1(n);
  ComputeNewtonDirection(params.options_0, y0, &d0_0, d0_1);

  double d1_0;
  Eigen::VectorXd d1_1(n);
  ComputeNewtonDirection(params.options_1, y1, &d1_0, d1_1);

  double dt_0 = d1_0 - d0_0;
  Eigen::VectorXd dt_1 = d1_1 - d0_1;

  GetMinSqrtMu(params.dinf_upper_bound, d0_0,
               SquaredNorm(Q_, d0_1, &workspace_.temp1_1), dt_0,
               SquaredNorm(Q_, dt_1, &workspace_.temp1_1),
               InnerProduct(Q_, dt_1, d0_1, &workspace_.temp1_1), output);

  output->d0_dot_dt = 2 * (d0_0 * dt_0 + InnerProduct(Q_, dt_1, d0_1,
                                                      &workspace_.temp1_1));
  output->dt_squared_norm =
      2 *
      (dt_0 * dt_0 + InnerProduct(Q_, dt_1, dt_1, &workspace_.temp1_1));
  output->d0_squared_norm =
      2 *
      (d0_0 * d0_0 + InnerProduct(Q_, d0_1, d0_1, &workspace_.temp1_1));

  bool failure = false;
  return failure;
}

void QuadraticConstraintBase::Initialize() {
  DenseMatrix W;
  A_gram_ = EvalAtQX(A1_, &W);
}

bool QuadraticConstraintBase::TakeStepImpl(const StepOptions& options) {
  auto& d_q1 = workspace_.temp2_1;
  double& d_q0 = workspace_.d0;
  double& wsqrt_q0 = *workspace_.sqrtW_0;
  auto& wsqrt_q1 = workspace_.sqrtW_1;
  double& wsqrt_q1_norm_sqr = workspace_.wsqrt_q1_norm_sqr;
  if (options.step_size != 1) {
    d_q0 = options.step_size * d_q0;
    d_q1 = options.step_size * d_q1;
  }

  Exp(Norm(Q_, d_q1, &workspace_.temp1_1), &d_q0, &d_q1);
  const auto& expd_q1 = d_q1;
  const auto& expd_q0 = d_q0;

  QuadraticRepresentation(
      wsqrt_q1_norm_sqr,
      InnerProduct(Q_, wsqrt_q1, expd_q1, &workspace_.temp1_1), wsqrt_q0,
      wsqrt_q1, expd_q0, expd_q1, workspace_.W0, &workspace_.W1);

  *workspace_.sqrtW_0 = *workspace_.W0;
  workspace_.sqrtW_1 = workspace_.W1;
  workspace_.wsqrt_q1_norm_sqr =
      SquaredNorm(Q_, workspace_.sqrtW_1, &workspace_.temp2_1);
  Sqrt(std::sqrt(workspace_.wsqrt_q1_norm_sqr), workspace_.sqrtW_0,
       &workspace_.sqrtW_1);
  workspace_.wsqrt_q1_norm_sqr =
      SquaredNorm(Q_, workspace_.sqrtW_1, &workspace_.temp2_1);
  return true;
}

//  A' Q(w) A
//  = A( w * w' + det w R) A
void QuadraticConstraintBase::ConstructSchurComplementSystemImpl(bool initialize,
                                    SchurComplementSystem* sys) {
  const auto& A0 = A0_;
  const auto& C0 = C0_;
  const auto& C1 = C1_;
  const auto& A_gram = A_gram_;
  auto& temp = workspace_.temp1_1;
  auto& A_dot_x = A_dot_x_;

  double c_dot_x = EvalCQX(workspace_.W1, temp);
  A_dot_x = EvalAtQX(workspace_.W1, temp);

  auto& Q_W1 = workspace_.temp2_1;
  double det_w = (*workspace_.W0) * (*workspace_.W0) -
                 SquaredNorm(Q_, workspace_.W1, &Q_W1);

  if (initialize) {
    SchurComplement(A0, A_gram, *workspace_.W0, det_w, A_dot_x, true,
                    &sys->G);
    sys->AW.noalias() = A_dot_x + A0 * (*workspace_.W0);
    sys->AQc.noalias() = det_w * (EvalAtQX(C1, temp) - A0 * C0);
    sys->inner_product_of_c_and_Qc = det_w * (EvalCQX(C1, temp) - C0 * C0);

    sys->AQe.noalias() = -det_w * (A0);
    sys->Ae = A0;
    sys->inner_product_of_c_and_e = C0;
    sys->inner_product_of_c_and_Qe = -det_w * C0;

  } else {
    throw std::runtime_error("Not supported.");
  }

  double c_scale;
  if (Q_.size() > 0) {
    c_scale = Q_W1.col(0).dot(C1.col(0)) + C0 * (*workspace_.W0);
  } else {
    c_scale = workspace_.W1.col(0).dot(C1.col(0)) + C0 * (*workspace_.W0);
  }
  sys->AQc.noalias() += 2 * (A_dot_x + A0 * (*workspace_.W0)) * c_scale;
  sys->AQe.noalias() +=
      2 * (A_dot_x + A0 * (*workspace_.W0)) * (*workspace_.W0);

  sys->inner_product_of_c_and_Qc +=
      2 * (c_dot_x + C0 * (*workspace_.W0)) * c_scale;

  sys->inner_product_of_c_and_Qe +=
      2 * (c_dot_x + C0 * (*workspace_.W0)) * (*workspace_.W0);

  if (initialize) {
    sys->inner_product_of_w_and_c = c_scale;
  } else {
    throw std::runtime_error("Not supported.");
  }

  // Account for Jordan inner-product  <x, y> := 2 x^T y.
  sys->AQc *= 2;
  sys->AW *= 2;
  sys->inner_product_of_c_and_Qc *= 2;
  sys->inner_product_of_w_and_c *= 2;

  sys->AQe *= 2;
  sys->Ae *= 2;
  sys->inner_product_of_c_and_e *= 2;
  sys->inner_product_of_c_and_Qe *= 2;
  sys->G *= 2;
}

void QuadraticConstraintBase::SetIdentityImpl() {
  *workspace_.W0 = 1;
  workspace_.W1.setZero();
  *workspace_.sqrtW_0 = 1;
  workspace_.sqrtW_1.setZero();
  workspace_.wsqrt_q1_norm_sqr = 0;
}

}  // namespace conex
