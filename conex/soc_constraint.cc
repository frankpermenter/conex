#include "conex/soc_constraint.h"

#include "conex/error_checking_macros.h"
#include "conex/error_codes.h"
#include "conex/newton_step.h"

namespace conex {

using EigenType = DenseMatrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Real = double;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Real = double;

struct SpinFactorProduct {
  Eigen::MatrixXd w1;
  Eigen::VectorXd w0;
};

Eigen::VectorXd SolveNormEquationsPlus(double a, double x0, double x1,
                                       double y0, double y1, double k) {
  Eigen::VectorXd t(2);
  double a_squared = a * a;
  double under_radical = a_squared * y1 + 2 * a * k * y0 - 2 * a * x0 * y1 +
                         k * k - 2 * k * x0 * y0 + x0 * x0 * y1 + x1 * y0 * y0 -
                         x1 * y1;

  if (under_radical > 1e-16) {
    t(0) = (-sqrt(under_radical) + a * y0 + k - x0 * y0) / (y0 * y0 - y1);
    t(1) = (sqrt(under_radical) + a * y0 + k - x0 * y0) / (y0 * y0 - y1);
  } else {
    if (under_radical >= 0) {
      t.resize(1);
      t(0) = (a * y0 + k - x0 * y0) / (y0 * y0 - y1);
    } else {
      t.resize(0);
    }
  }

  return t;
}

Eigen::VectorXd SolveNormEquationsMinus(double a, double x0, double x1,
                                        double y0, double y1, double k) {
  Eigen::VectorXd t(2);
  double a_squared = a * a;
  double under_radical = a_squared * y1 + 2 * a * k * y0 - 2 * a * x0 * y1 +
                         k * k - 2 * k * x0 * y0 + x0 * x0 * y1 + x1 * y0 * y0 -
                         x1 * y1;

  if (under_radical > 1e-16) {
    t(0) =
        (-sqrt(under_radical) + a * y0 + k - x0 * y0) / (y0 * y0 - y1 + 1e-15);
    t(1) =
        (sqrt(under_radical) + a * y0 + k - x0 * y0) / (y0 * y0 - y1 + 1e-15);
  } else {
    if (under_radical >= 0) {
      t.resize(1);
      t(0) = (a * y0 + k - x0 * y0) / (y0 * y0 - y1 + 1e-15);
    } else {
      t.resize(0);
    }
  }

  return t;
}

Eigen::VectorXd GetCandidateK(double dinfmax, double x0, double x1, double y0,
                              double y1, double k) {
  auto t = SolveNormEquationsPlus(dinfmax, x0, x1, y0, y1, k);
  std::vector<double> val;
  double eps = 0.01;
  for (int i = 0; i < t.size(); i++) {
    double error_minus =
        x0 + t(i) * y0 - sqrt(x1 + 2 * t(i) * k + t(i) * t(i) * y1) + dinfmax;
    double error_plus =
        x0 + t(i) * y0 + sqrt(x1 + 2 * t(i) * k + t(i) * t(i) * y1) - dinfmax;

    if ((fabs(error_plus) < eps && error_minus > -eps) ||
        (fabs(error_minus) < eps && error_plus < eps)) {
      val.push_back(t(i));
    }
  }
  t = SolveNormEquationsPlus(-dinfmax, x0, x1, y0, y1, k);
  for (int i = 0; i < t.size(); i++) {
    double error_minus =
        x0 + t(i) * y0 - sqrt(x1 + 2 * t(i) * k + t(i) * t(i) * y1) + dinfmax;
    double error_plus =
        x0 + t(i) * y0 + sqrt(x1 + 2 * t(i) * k + t(i) * t(i) * y1) - dinfmax;

    if ((fabs(error_plus) < eps && error_minus > -eps) ||
        (fabs(error_minus) < eps && error_plus < eps)) {
      val.push_back(t(i));
    }
  }
  return Eigen::Map<const VectorXd>(val.data(), static_cast<int>(val.size()));
}

double GetMinSqrtMu(double dinfmax, const SpinFactorProduct& x,
                    const SpinFactorProduct& y, LineSearchOutput* output) {
  double upper_bound = 1e45;
  double lower_bound = -1e45;
  for (int i = 0; i < x.w0.size(); i++) {
    auto t =
        GetCandidateK(dinfmax, x.w0(i), x.w1.col(i).squaredNorm(), y.w0(i),
                      y.w1.col(i).squaredNorm(), x.w1.col(i).dot(y.w1.col(i)));

    if (t.size() < 2) {
      // Force failure
      upper_bound = -1;
      lower_bound = 1;
      break;
    }

    double lower_bound_i = t.minCoeff();
    double upper_bound_i = t.maxCoeff();

    if (lower_bound_i > lower_bound) {
      lower_bound = lower_bound_i;
    }
    if (upper_bound_i < upper_bound) {
      upper_bound = upper_bound_i;
    }

#ifndef NDEBUG
    for (int j = 0; j < t.size(); j++) {
      double v0 = x.w0(i) + t(j) * y.w0(i);
      VectorXd v1 = x.w1.col(i) + t(j) * y.w1.col(i);
      double val = fabs(v0 + v1.norm());
      if (fabs(v0 - v1.norm()) > val) {
        val = fabs(v0 - v1.norm());
      }
      if (fabs(val - dinfmax) > 0.02) {
        return -1;
        throw std::runtime_error("Bad calculation.");
      }
    }
#endif
  }

  output->lower_bound = lower_bound;
  output->upper_bound = upper_bound;

#if 0
  VectorXd d = x.w1 + lower_bound * y.w1;
  VectorXd d0 = x.w0 + lower_bound * y.w0;
  double test0 = std::fabs(d0(0)  - d.norm());
  double test1 = std::fabs(d0(0)  + d.norm());
  if (test1 > test0) {
    test0 = test1;
  }
  DUMP(test0);
  if (fabs(test0 - dinfmax) > 0.02) {
    throw std::runtime_error("Bad calculation.");
  }


  d = x.w1 + upper_bound * y.w1;
  d0 = x.w0 + upper_bound * y.w0;
  test0 = std::fabs(d0(0)  - d.norm());
  test1 = std::fabs(d0(0)  + d.norm());
  if (test1 > test0) {
    test0 = test1;
  }
  DUMP(test0);
  if (fabs(test0 - dinfmax) > 0.02) {
    throw std::runtime_error("Bad calculation.");
  }
#endif

  return upper_bound;
}

// Implements the spectral decomposition of the Spin Factor algebra.
// See
// http://rutcor.rutgers.edu/~alizadeh/CLASSES/12fallSDP/Notes/Lecture08/lec08.pdf
// or "Analysis on Symmetric Cones" by Faraut and Koranyi.
class SpectralDecompSpinFactor {
 public:
  using EssentialVectorType = DenseMatrix;

  SpectralDecompSpinFactor(int n) : n_(n), q_(n, 1) {}
  int n_;

  struct PeirceDecompType {
    PeirceDecompType(int n) : X00(n + 1), X11(n + 1), X01(n + 1) {}

    Eigen::VectorXd X00;
    Eigen::VectorXd X11;
    Eigen::VectorXd X01;
    Eigen::VectorXd Component(int i, int j) {
      if ((i == 0) && (j == 0)) {
        return X00;
      }
      if ((i + j == 1)) {
        return X01;
      }
      if ((i == 1) && (j == 1)) {
        return X11;
      }
      return EigenType();
    }
  };

  Eigen::Matrix<Real, 2, 1> Eigenvalues() const { return eigenvalues_; }
  void Compute(const Eigen::VectorXd& x) {
    assert(x.rows() == n_ + 1);
    q_ = x.col(0).tail(n_);
    norm_of_q_ = q_.norm();
    if (norm_of_q_ > 0) {
      q_ = q_ / norm_of_q_;
    }
    eigenvalues_(0) = x(0) + norm_of_q_;
    eigenvalues_(1) = x(0) - norm_of_q_;
  }

  EigenType Idempotent(int i) const { return Idempotents().col(i); }

  // Implements equations from page 7 of
  // http://rutcor.rutgers.edu/~alizadeh/CLASSES/12fallSDP/Notes/Lecture08/lec08.pdf
  DenseMatrix Idempotents() const {
    int n = n_;
    DenseMatrix idempotents(n + 1, 2);
    if (norm_of_q_ > 0) {
      idempotents.col(0) << .5, .5 * q_;
      idempotents.col(1) << .5, -.5 * q_;
    } else {
      idempotents.setZero();
      idempotents(0, 0) = .5;
      idempotents(0, 1) = .5;
    }
    return idempotents;
  }

  // The 3 Peirce components of x are the orthogonal projections of
  // x onto the following 3 subspaces:
  //
  //   S00 := span { (1,  q) }               (dim = 0)
  //   S11 := span { (1, -q) }               (dim = 0)
  //   S01 :=  (S00 + S11)^{\perp}
  //        = { (0, p) : <p, q> = 0 }
  //
  // See, e.g., Example 06 of "An Introduction to
  // Formally Real Jordan Algebras and Their Applications in Optimization" by
  // Alizadeh.
  PeirceDecompType TransformToPeirceComponents(const Eigen::VectorXd& x) const {
    PeirceDecompType peirce_decomp(x.rows());
    // Compute X00 and X11 by directly computing orthogonal projection
    // onto S00 and S11
    int size = n_;
    const double c0 = .5 * x(0);
    const double c1 = .5 * q_.dot(x.tail(size - 1));
    peirce_decomp.X00(0) = c0 + c1;
    peirce_decomp.X00.tail(size - 1) = (c0 + c1) * q_;
    peirce_decomp.X11(0) = c0 - c1;
    peirce_decomp.X11.tail(size - 1) = (c1 - c0) * q_;

    // Compute X01 by using the fact that S01 + S00 + S11 is a direct-sum
    // decomposition
    peirce_decomp.X01 = x - peirce_decomp.X00 - peirce_decomp.X11;
    return peirce_decomp;
  }

  EigenType TransformFromPeirceComponents(const PeirceDecompType& X) const {
    return X.X00 + X.X11 + X.X01;
  }

  // If we have computed the spectral decomposition of (x0, x1), then the
  // essential unit vector is x1*1/|x1|.
  auto EssentialUnitVector() const { return q_; }
  auto NormOfEssentialVector() const { return norm_of_q_; }

  // Let z = (z0, z1) have Peirce decomposition
  //      z = c0 (1, q) + c1 (1, -q)  + (0, p).
  //
  // This function returns (c0-c1) q, i.e., the essential vector of just the
  // "diagonal" Peirce components
  //        c0 (1, q) + c1 (1, -q).
  //
  // Since z1 = (c0 - c1) q + p  and  <p , q> = 0, we compute this simply by
  // projecting the essential vector z1 onto the span of q.
  EssentialVectorType EssentialVectorOfDiagonalPeirceComponents(
      const Eigen::VectorXd& z) const {
    const double inner_product = q_.tail(n_ - 1).dot(z);
    return q_ * inner_product;
  }

 private:
  Eigen::Matrix<Real, 2, 1> eigenvalues_;
  Eigen::VectorXd q_;
  Real norm_of_q_;
};

DenseMatrix QuadraticRepresentation(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& y) {
  // We use the formula from Example 11.12 of "Formally Real Jordan Algebras
  // and Their Applications to Optimization"  by Alizadeh, which states the
  // quadratic representation of x equals the linear map
  //                          2xx' - (det x) * R
  // where R is the reflection operator R = diag(1, -1, ..., -1) and det x is
  // the determinate of x = (x0, x1), i.e., det x = x0^2 - |x1|^2.
  int order = x.rows();
  double det_x = x(0) * x(0) - x.tail(order - 1).squaredNorm();
  EigenType z = det_x * y;
  z(0) *= -1;
  return (2 * x.dot(y)) * x + z;
}

DenseMatrix Sqrt(double x0, const DenseMatrix& x) {
  int n = x.rows();
  DenseMatrix z(n + 1, 1);
  z(0, 0) = x0;
  z.bottomRows(n) = x;
  SpectralDecompSpinFactor spec(n);
  spec.Compute(z);
  auto ev = spec.Eigenvalues();

  CONEX_DEMAND(ev.minCoeff() > 0, "Sqrt failed: element not in cone");

  DenseMatrix zsqrt = std::sqrt(ev(0, 0)) * spec.Idempotent(0) +
                      std::sqrt(ev(1, 0)) * spec.Idempotent(1);
  return zsqrt;
}

DenseMatrix Exp(double x0, const DenseMatrix& x) {
  int n = x.rows();
  DenseMatrix z(n + 1, 1);
  z(0, 0) = x0;
  z.bottomRows(n) = x;
  SpectralDecompSpinFactor spec(n);
  spec.Compute(z);
  auto ev = spec.Eigenvalues();
  DenseMatrix zsqrt = std::exp(ev(0, 0)) * spec.Idempotent(0) +
                      std::exp(ev(1, 0)) * spec.Idempotent(1);
  return zsqrt;
}

double NormInf(double x0, const DenseMatrix& x) {
  int n = x.rows();
  DenseMatrix z(n + 1, 1);
  z(0, 0) = x0;
  z.bottomRows(n) = x;
  SpectralDecompSpinFactor spec(n);
  spec.Compute(z);
  auto ev = spec.Eigenvalues();
  if (std::fabs(ev(0)) > std::fabs(ev(1))) {
    return std::fabs(ev(0));
  } else {
    return std::fabs(ev(1));
  }
}

void SOCConstraint::ComputeNegativeSlack(double inv_sqrt_mu, const Ref& y,
                                         Ref* minus_s) {
  minus_s->noalias() = (constraint_matrix_)*y;
  minus_s->noalias() -= (constraint_affine_)*inv_sqrt_mu;
}

// Combine this with PrepareStep
void GetWeightedSlackEigenvalues(SOCConstraint* o, const Ref& y,
                                 double c_weight, WeightedSlackEigenvalues* p) {
  auto* workspace = &o->workspace_;
  int n = workspace->n_;
  Eigen::VectorXd minus_s_data(n + 1);
  Ref minus_s(minus_s_data.data(), n + 1, 1);
  Eigen::VectorXd Ws(n + 1);
  o->ComputeNegativeSlack(c_weight, y, &minus_s);

  auto wsqrt = Sqrt(*o->workspace_.W0, o->workspace_.W1);
  Ws = QuadraticRepresentation(wsqrt, minus_s);

  SpectralDecompSpinFactor spec(n);
  spec.Compute(Ws);
  auto ev = spec.Eigenvalues();

  const double lamda_max = -ev.minCoeff();
  const double lamda_min = -ev.maxCoeff();

  p->lambda_max = lamda_max;
  p->lambda_min = lamda_min;
  p->frobenius_norm_squared = std::pow(lamda_max, 2) + std::pow(lamda_min, 2);
  p->trace = (lamda_max + lamda_min);
}

bool TakeStep(SOCConstraint* o, const StepOptions& opt) {
  auto d1 = o->workspace_.temp1_1;
  auto d0 = o->workspace_.d0;

  int n = d1.rows() + 1;
  Eigen::VectorXd wsqrt(n);
  wsqrt(0, 0) = *o->workspace_.W0;
  wsqrt.bottomRows(n - 1) = o->workspace_.W1;

  if (opt.step_size != 1.0) {
    d0 *= opt.step_size;
    d1 *= opt.step_size;
  }
  auto expd = Exp(d0, d1);
  auto wn = QuadraticRepresentation(wsqrt, expd);
  *o->workspace_.W0 = wn(0, 0);
  o->workspace_.W1 = wn.bottomRows(n - 1);

  CONEX_ASSERT(o->workspace_.W1.norm() <= *o->workspace_.W0,
               "Element not in cone.");

  return true;
}

void PrepareStep(SOCConstraint* o, const StepOptions& opt, const Ref& y,
                 StepInfo* info) {
  Eigen::VectorXd minus_s_data(o->workspace_.n_ + 1);
  Ref minus_s(minus_s_data.data(), o->workspace_.n_ + 1, 1);
  o->ComputeNegativeSlack(opt.c_weight, y, &minus_s);

  // e - Q(w^{1/2})(C-A^y)
  int n = minus_s.rows();
  auto wsqrt = Sqrt(*o->workspace_.W0, o->workspace_.W1);
  *o->workspace_.W0 = wsqrt(0, 0);
  o->workspace_.W1 = wsqrt.bottomRows(n - 1);

  auto d = QuadraticRepresentation(wsqrt, minus_s);
  d(0, 0) += 1;
  o->workspace_.temp1_1 = d.bottomRows(n - 1);
  o->workspace_.d0 = d(0, 0);

  info->norminfd = NormInf(d(0, 0), d.bottomRows(n - 1));
  info->normsqrd = 2 * d.squaredNorm();
}

VectorXd SOCConstraint::BuildNewtonDirection(double c_weight, const Ref& y) {
  int n = workspace_.n_;
  Eigen::VectorXd minus_s_data(n + 1);
  Ref minus_s(minus_s_data.data(), n + 1, 1);
  ComputeNegativeSlack(c_weight, y, &minus_s);

  auto wsqrt = Sqrt(*workspace_.W0, workspace_.W1);
  auto d = QuadraticRepresentation(wsqrt, minus_s);
  d(0, 0) += 1;
  return d;
}
double InnerProduct(const SpinFactorProduct& x, const SpinFactorProduct& y) {
  MatrixXd temp = x.w1.transpose() * y.w1;
  double val = temp(0, 0);
  if (temp.rows() > 1 || temp.cols() > 1) {
    throw std::runtime_error("Expected trivial spin-factor product.");
  }
  val += x.w0(0) * y.w0(0);
  return 2 * val;
}

bool PerformLineSearch(SOCConstraint* o, const LineSearchParameters& params,
                       const Ref& y0, const Ref& y1, LineSearchOutput* output) {
  int n = o->workspace_.n_;

  auto temp = o->BuildNewtonDirection(params.c0_weight, y0);
  SpinFactorProduct d0;
  d0.w0 = temp.head(1);
  d0.w1 = temp.tail(n);

  temp = o->BuildNewtonDirection(params.c1_weight, y1);
  SpinFactorProduct d1;
  d1.w0 = temp.head(1);
  d1.w1 = temp.tail(n);

  SpinFactorProduct dt;
  dt.w0 = d1.w0 - d0.w0;
  dt.w1 = d1.w1 - d0.w1;

  GetMinSqrtMu(params.dinf_upper_bound, d0, dt, output);
  bool failure = false;
  // if (success == -1) {
  //   failure = true;
  // }
  return failure;
}
void ConstructSchurComplementSystem(SOCConstraint* o, bool initialize,
                                    SchurComplementSystem* sys) {
  int n = o->workspace_.n_;
  auto Wsqrt = Sqrt(*o->workspace_.W0, o->workspace_.W1);
  DenseMatrix W(n + 1, 1);
  W(0, 0) = *o->workspace_.W0;
  W.bottomRows(n) = o->workspace_.W1;

  auto G = &sys->G;

  Eigen::MatrixXd WA = o->constraint_matrix_;
  Eigen::MatrixXd WsqrtC =
      QuadraticRepresentation(Wsqrt, o->constraint_affine_);

  for (int i = 0; i < WA.cols(); i++) {
    WA.col(i) = QuadraticRepresentation(Wsqrt, WA.col(i));
  }

  if (initialize) {
    (*G).noalias() = 2 * WA.transpose() * WA;
    sys->AW.noalias() = 2 * o->constraint_matrix_.transpose() * W;
    sys->AQc.noalias() = 2 * WA.transpose() * WsqrtC;
    sys->inner_product_of_w_and_c = 2 * WsqrtC(0);
    sys->inner_product_of_c_and_Qc = 2 * WsqrtC.squaredNorm();
  } else {
    (*G).noalias() += 2 * WA.transpose() * WA;
    sys->AW.noalias() += 2 * o->constraint_matrix_.transpose() * W;
    sys->AQc.noalias() += 2 * WA.transpose() * WsqrtC;
    sys->inner_product_of_w_and_c += 2 * WsqrtC(0);
    sys->inner_product_of_c_and_Qc += 2 * WsqrtC.squaredNorm();
  }
}

template <typename T>
void ConservativeResizeHelper(T* constraint_matrix_, int var, int rows) {
  if (!(var < constraint_matrix_->cols())) {
    int cols_new = var + 1 - constraint_matrix_->cols();
    constraint_matrix_->conservativeResize(rows, var + 1);
    constraint_matrix_->rightCols(cols_new).setZero();
  }
}

CONEX_STATUS UpdateLinearOperator(SOCConstraint* o, double val, int var, int r,
                                  int c, int dim) {
  CONEX_RETURN_ON_FAIL(dim == 0, "Complex second-order cone not supported.");
  CONEX_RETURN_ON_FAIL(c == 0, "Second-order constraint is not matrix valued.");
  CONEX_RETURN_ON_FAIL(r <= o->n_, "Row index out of bounds.");
  CONEX_RETURN_ON_FAIL((var >= 0) && (r >= 0), "Indices cannot be negative.");

  ConservativeResizeHelper(&o->constraint_matrix_, var, o->n_ + 1);
  o->constraint_matrix_(r, var) = val;
  return CONEX_SUCCESS;
}

CONEX_STATUS UpdateAffineTerm(SOCConstraint* o, double val, int r, int c,
                              int dim) {
  CONEX_RETURN_ON_FAIL(dim == 0, "Complex second-order cone not supported.");
  CONEX_RETURN_ON_FAIL(c == 0, "Second-order constraint is not matrix valued.");
  CONEX_RETURN_ON_FAIL(r <= o->n_, "Row index out of bounds.");
  CONEX_RETURN_ON_FAIL(r >= 0, "Indices cannot be negative.");

  ConservativeResizeHelper(&o->constraint_affine_, 0, o->n_ + 1);
  o->constraint_affine_(r) = val;
  return CONEX_SUCCESS;
}

}  // namespace conex
