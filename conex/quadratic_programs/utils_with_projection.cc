#include "conex/debug_macros.h"
#include "utils.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace quadratic_programs {

// For LP, we rescale A, b, c  so that
//
//   inf |Ax + b\| = 1,  | A(A'A)^{-1} c| = 1
//
// This means the min. norm points in the affine
// sets
//
//  \{ l : A'l = c\}   \{ Ax + b : x \in R^m \}
//
//  have unit norm.
//
//  For QP, we generalize this by simply replacing
//  all instances of A'A in the normal equations
//  with A'A + W.
ProblemData RescaleProblemData(const ProblemData& data) {
  MatrixXd G = data.W + data.A.transpose() * data.A;
  Eigen::LLT<MatrixXd> llt(G);
  MatrixXd P = data.A * llt.solve(data.A.transpose());
  VectorXd b0 = data.b - P * data.b;
  VectorXd l0 = data.A * llt.solve(data.c);

  auto datain = data;
  int n = data.A.rows();
  double scale_b = 1;
  if (b0.squaredNorm() > 1.0 / n) {
    scale_b = 1.0 / b0.norm() * 1.0 / std::sqrt(n);
    datain.A *= scale_b;
    datain.b *= scale_b;
  }

  if (l0.squaredNorm() > 1.0 / n) {
    // We have already scaled A, so the minimum
    // norm solution now has value
    //   l0_hat = (1/scale_b) * A(A'A)^{-1} c
    // implying that
    //   |l0_hat| = (1/scale_b) * \|l0||
    // double scale = (1.0 / std::sqrt(n)) * 1.0 / l0.norm();
    double scale = scale_b / l0.norm() * 1.0 / std::sqrt(n);
    datain.c *= scale;
    datain.W *= scale;
  }
  return datain;
}

// Want to find an improving ray satisfying
//
// A' r = W z
// <b, r> < 0
// r > 0
//
// Given feasible lambda, i.e., lambda > 0
// satisfying
//
//  A'lambda = Wx + c
//
// We have that
//
//  A' (lambda + alp * r) = W(x+alp*z) + c
//
//  minimize (b'lambda)
//
bool CheckPrimalInfeasibility(const ProblemData& data, const Variable& w,
                              const Direction& d, double* descent,
                              Variable* certificate) {
  // Verify that
  // A' w(e + d) = Wx
  VectorXd lambda = w.expv;
  lambda += w.expv.cwiseProduct(d.d);
  double residual = (data.A.transpose() * (lambda)-data.W * d.x).norm();
  residual = residual / lambda.norm();
  *descent = data.b.dot(lambda);
  certificate->x = d.x;
  certificate->lambda = lambda;

  double descent_normalized = *descent / (lambda.norm() * data.b.norm());
  return descent_normalized < -1e-3 && residual < 1e-9 &&
         d.d.minCoeff() > -(1 + 5e-2);
}

// Want point x satisfying
// Ax > 0
// x'Wx + c'x < 0
bool CheckDualInfeasibility(const ProblemData& data, const Variable& w,
                            const Direction& d, double* directional_deriv,
                            Variable* certificate) {
  *directional_deriv = d.x.dot(data.W * d.x + data.c);
  certificate->x = d.x;
  certificate->slack = data.A * d.x;
  // d = e - A'x =>
  //
  //  Ax = e - d
  return *directional_deriv / d.x.norm() < -1 && d.d.maxCoeff() < 1 + 1e-4;
}

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

class Contraction {
 public:
  Contraction(const MatrixXd& B, const MatrixXd& W)
      : B_(B), llt_(B.transpose() * B + W) {}
  VectorXd Eval(const VectorXd& x) {
    return B_ * llt_.solve(B_.transpose() * x);
  }

 private:
  Eigen::LLT<MatrixXd> llt_;
  MatrixXd B_;
};

class CoordinateProjection {
 public:
  CoordinateProjection(const VectorXd& D) : D_(D) {}
  VectorXd Eval(const VectorXd& x) {
    VectorXd y = x;
    for (int i = 0; i < x.rows(); i++) {
      if (D_(i) < 1) {
        y(i) = 0;
      }
    }
    return y;
  }

 private:
  VectorXd D_;
};

double EvalProjection(const ProblemData& data, const VectorXd& exp_v) {
  MatrixXd AD = exp_v.asDiagonal() * data.A;
  Contraction P(AD, data.W);
  VectorXd v(exp_v.rows());
  v.setZero();
  for (int i = 0; i < exp_v.rows(); i++) {
    if (exp_v(i) < 1) {
      v(i) = std::rand();
    }
  }
  v = v / v.norm();
  double val = (v - P.Eval(v)).norm();
  // VectorXd e = v;
  // e.setConstant(1);
  // MatrixXd M(e.rows(), 2);
  // M << P.Eval(e), exp_v;
  // DUMP(M);
  return val;
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

  VectorXd x0 = data.A.transpose().colPivHouseholderQr().solve(data.c);
  //  VectorXd x0 = VectorXd::Zero(data.b.rows());

  MatrixXd AD = exp_v.asDiagonal() * data.A;
  Contraction P(AD, data.W);
  CoordinateProjection Pcoord(exp_v);
  VectorXd e(exp_v.rows());
  e.setConstant(1);
  VectorXd weighted_x0 = exp_v.asDiagonal().inverse() * x0;
  VectorXd weighted_b = exp_v.asDiagonal() * data.b;
  VectorXd project = 2 * e - sqrtmuinv * (weighted_x0 + weighted_b);
  VectorXd weighted_slack = sqrtmuinv * weighted_b + P.Eval(project);

  VectorXd project_coord = 2 * e - sqrtmuinv * (weighted_x0 + weighted_b);
  DUMP(Pcoord.Eval(project_coord) + sqrtmuinv * weighted_b);

  // VectorXd weighted_slackP.Eval(2*e + -sqrtmuinv*(weighted_b + weighted_x0))

  y.x = llt.solve(rhs);
#ifdef ITERATIVE_REFINEMENT
  for (int i = 0; i < 10; i++) {
    y.x += llt.solve(rhs - S * y.x);
  }
#endif

  //  y.d = -exp_v.cwiseProduct(A * y.x + sqrtmuinv * b);
  y.d.array() = -weighted_slack;
  y.d.array() += 1;

  y.dlambda_times_d_slack = 1 - EvalProjection(data, exp_v);
  // d_lambda.cwiseProduct(d_slack).eval().array().abs().maxCoeff();

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
