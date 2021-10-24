#include "utils.h"
#include "conex/debug_macros.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace quadratic_programs {

Direction PredictorDirection(const ProblemData& data, const VectorXd& exp_v) {
  const MatrixXd& W = data.W;
  const VectorXd& c = data.c;
  const MatrixXd& A = data.A;
  const VectorXd& b = data.b;

  MatrixXd Q = exp_v.cwiseProduct(exp_v).asDiagonal();
  MatrixXd S = A.transpose() * Q * A + W;

  VectorXd rhs = -(c + A.transpose() * Q * b);

  Eigen::LDLT<MatrixXd> llt(S);
  Direction y;
  y.x = llt.solve(rhs);
#ifdef ITERATIVE_REFINEMENT
  for (int i = 0; i < 10; i++) {
    y.x += llt.solve(rhs - S * y.x);
  }
#endif

  // DUMP(S*y.x - rhs);
  y.d = exp_v.cwiseProduct(b - A * y.x);
  return y;
}

double UpperBound(const ProblemData& data, const VectorXd& exp_v) {
  const MatrixXd& W = data.W;
  const VectorXd& c = data.c;
  const MatrixXd& A = data.A;
  const VectorXd& b = data.b;

  MatrixXd Q = exp_v.cwiseProduct(exp_v).asDiagonal();
  MatrixXd S = A.transpose() * Q * A + W;

  VectorXd rhs1 = -(c + A.transpose() * Q * b);
  VectorXd rhs0 = 2 * A.transpose() * exp_v;

  Eigen::LDLT<MatrixXd> llt(S);
  VectorXd x0 = llt.solve(rhs0);
  VectorXd x1 = llt.solve(rhs1);

  // VectorXd  A*(x0 + k * x1) + k * b \ge 0
  //    A * (1/k * x0) + x1) + b \ge 0
  //    A * (1/k * x0) \ge -(A*x1 + b)
  //

  VectorXd sqrtmu = (A * x0).cwiseInverse().cwiseProduct(-(A * x1 + b));

  VectorXd Ax0 = A * x0;
  VectorXd minus_Ax1_plus_b = -(A * x1 + b);

  double min_mu = -1;
  for (int i = 0; i < A.rows(); i++) {
    if (Ax0(i) > 0) {
      double lb = minus_Ax1_plus_b(i) / Ax0(i);
      if (lb > min_mu) {
        min_mu = lb;
      }
    }
  }
  return 0;
}

Direction NewtonDirection(const ProblemData& data, const VectorXd& exp_v,
                          const double sqrtmuinv) {
  const MatrixXd& W = data.W;
  const VectorXd& c = data.c;
  const MatrixXd& A = data.A;
  const VectorXd& b = data.b;

  MatrixXd Q = exp_v.cwiseProduct(exp_v).asDiagonal();
  MatrixXd S = A.transpose() * Q * A + W;

  Eigen::LDLT<MatrixXd> llt(S);
  Direction y;

  VectorXd rhs = -sqrtmuinv * (c + A.transpose() * Q * b);
  rhs = rhs + 2 * A.transpose() * exp_v;
  y.x = llt.solve(rhs);

#ifdef ITERATIVE_REFINEMENT
  for (int i = 0; i < 10; i++) {
    y.x += llt.solve(rhs - S * y.x);
  }
#endif

  // DUMP(S*y.x - rhs);
  y.d = -exp_v.cwiseProduct(A * y.x + sqrtmuinv * b);

  y.d.array() += 1;
  return y;
}

Direction DualNewtonDirection(const ProblemData& data, const VectorXd& exp_v,
                              const VectorXd& x, const double sqrtmuinv) {
  const MatrixXd& W = data.W;
  const VectorXd& c = data.c;
  const MatrixXd& A = data.A;
  const VectorXd& b = data.b;

  MatrixXd Q = exp_v.cwiseProduct(exp_v).asDiagonal();
  MatrixXd S = A.transpose() * Q * A + data.W;

  VectorXd rhs = A.transpose() * exp_v - sqrtmuinv * (c + W * x);

  Eigen::LLT<MatrixXd> llt(S);
  Direction y;

  y.x = llt.solve(rhs);

#if ITERATIVE_REFINEMENT
  for (int i = 0; i < 4; i++) {
    y.x += llt.solve(rhs - S * y.x);
  }
#endif

  y.d = -exp_v.cwiseProduct(A * y.x);
  return y;
}

double Rescale(const ProblemData& data, double sqrtmu, const Variable& v) {
  VectorXd expmv = v.expv.cwiseInverse();
  VectorXd zgrad = sqrtmu * data.A.transpose() * v.expv;
  VectorXd zslack = sqrtmu * expmv;
  double k1 = (data.W * v.x + data.c).dot(zgrad) / zgrad.squaredNorm();
  double k2 = (data.A * v.x + data.b).dot(zslack) / zslack.squaredNorm();

  double ratio = k1 / k2;

  // recompute without using mu
  zgrad = data.A.transpose() * v.expv;
  zslack = expmv;
  k1 = (data.W * v.x + data.c).dot(zgrad) / zgrad.squaredNorm();
  k2 = (data.A * v.x + data.b).dot(zslack) / zslack.squaredNorm();

  if (std::fabs(ratio - k1 / k2) > 1e-9) {
    throw std::runtime_error("Scaling depends on mu");
  }

  if (k2 <= 0) {
    k2 = 0.00001;
  }
  if (k1 <= 0) {
    k1 = 0.00001;
  }

  return std::sqrt(k1 / k2);
}

double FindMinimumMu(const VectorXd& d0, const VectorXd& delta,
                     double dinfmax) {
  double upper_bound = -1e14;
  double lower_bound = 1e14;
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

  if (lower_bound > upper_bound) {
    return -1;
  } else {
    return upper_bound;
  }
}

}  // namespace quadratic_programs
}  // namespace conex
