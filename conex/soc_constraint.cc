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

SOCConstraint::SOCConstraint(const Eigen::MatrixXd& constraint_matrix,
                             const Eigen::MatrixXd& constraint_affine)
    : workspace_(constraint_matrix.rows() - 1),
      constraint_matrix_(constraint_matrix),
      constraint_affine_(constraint_affine) {
  CONEX_DEMAND(constraint_matrix_.rows() == constraint_affine_.rows(),
               "Invalid SOC problem data.");
}

namespace {
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

namespace {
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
}  // namespace

void SOCConstraint::ComputeNegativeSlack(double inv_sqrt_mu, const RefType& y,
                                         NonConstRefType minus_s) {
  minus_s.noalias() = (constraint_matrix_)*y;
  minus_s.noalias() -= (constraint_affine_)*inv_sqrt_mu;
}

void SOCConstraint::SetIdentityImpl() {
  *workspace_.W0 = 1;
  workspace_.W1.setZero();
}

// Combine this with PrepareStep
void SOCConstraint::GetWeightedSlackEigenvaluesImpl(
    const RefType& y, double c_weight, WeightedSlackEigenvalues* p) {
  auto* workspace = &workspace_;
  int n = workspace->n_;
  Eigen::VectorXd minus_s(n + 1);
  Eigen::VectorXd Ws(n + 1);
  ComputeNegativeSlack(c_weight, y, minus_s);

  auto wsqrt = Sqrt(*workspace_.W0, workspace_.W1);
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

bool SOCConstraint::TakeStepImpl(const StepOptions& opt) {
  auto d1 = workspace_.temp1_1;
  auto d0 = workspace_.d0;

  int n = d1.rows() + 1;

  auto wsqrt = Sqrt(*workspace_.W0, workspace_.W1);

  if (opt.step_size != 1.0) {
    d0 *= opt.step_size;
    d1 *= opt.step_size;
  }
  auto expd = Exp(d0, d1);
  auto wn = QuadraticRepresentation(wsqrt, expd);
  *workspace_.W0 = wn(0, 0);
  workspace_.W1 = wn.bottomRows(n - 1);

  CONEX_ASSERT(workspace_.W1.norm() <= *workspace_.W0, "Element not in cone.");
  return true;
}

void SOCConstraint::PrepareStepImpl(const StepOptions& opt, const RefType& y,
                                    StepInfo* info) {
  int n = workspace_.n_;
  auto d = BuildNewtonDirection(opt, y);
  workspace_.temp1_1 = d.bottomRows(n);
  workspace_.d0 = d(0, 0);

  info->norminfd = NormInf(d(0, 0), d.bottomRows(n));
  info->normsqrd = 2 * d.squaredNorm();
}

VectorXd SOCConstraint::BuildNewtonDirection(const StepOptions& options,
                                             const RefType& y) {
  int n = workspace_.n_;
  Eigen::VectorXd minus_s(n + 1);
  ComputeNegativeSlack(options.c_weight, y, minus_s);
  minus_s(0) -= options.w_weight;

  auto wsqrt = Sqrt(*workspace_.W0, workspace_.W1);
  auto d = QuadraticRepresentation(wsqrt, minus_s);
  d(0, 0) += options.e_weight;
  return d;
}

bool SOCConstraint::PerformLineSearchImpl(const LineSearchParameters& params,
                                          const RefType& y0, const RefType& y1,
                                          LineSearchOutput* output) {
  int n = workspace_.n_;

  auto temp = BuildNewtonDirection(params.options_0, y0);
  double d0_0 = temp(0);
  VectorXd d0_1 = temp.tail(n);

  temp = BuildNewtonDirection(params.options_1, y1);
  double d1_0 = temp(0);
  VectorXd d1_1 = temp.tail(n);

  double dt_0 = d1_0 - d0_0;
  VectorXd dt_1 = d1_1 - d0_1;

  GetMinSqrtMu(params.dinf_upper_bound, d0_0, d0_1.squaredNorm(), dt_0,
               dt_1.squaredNorm(), dt_1.dot(d0_1), output);
  output->dt_squared_norm = 2 * (dt_0 * dt_0 + dt_1.dot(dt_1));
  output->d0_squared_norm = 2 * (d0_0 * d0_0 + d0_1.dot(d0_1));
  output->d0_dot_dt = 2 * (d0_0 * dt_0 + d0_1.dot(dt_1));
  bool failure = false;
  return failure;
}

void SOCConstraint::ConstructSchurComplementSystemImpl(
    bool initialize, SchurComplementSystem* sys) {
  int n = workspace_.n_;
  auto Wsqrt = Sqrt(*workspace_.W0, workspace_.W1);
  DenseMatrix W(n + 1, 1);
  W(0, 0) = *workspace_.W0;
  W.bottomRows(n) = workspace_.W1;

  auto G = &sys->G;
  Eigen::MatrixXd WA = constraint_matrix_;
  Eigen::MatrixXd WsqrtC = QuadraticRepresentation(Wsqrt, constraint_affine_);

  for (int i = 0; i < WA.cols(); i++) {
    WA.col(i) = QuadraticRepresentation(Wsqrt, WA.col(i));
  }

  if (initialize) {
    (*G).noalias() = 2 * WA.transpose() * WA;
    sys->AW.noalias() = 2 * constraint_matrix_.transpose() * W;
    sys->AQc.noalias() = 2 * WA.transpose() * WsqrtC;
    sys->inner_product_of_w_and_c = 2 * WsqrtC(0);
    sys->inner_product_of_c_and_Qc = 2 * WsqrtC.squaredNorm();

    sys->inner_product_of_c_and_Qe = 2 * WsqrtC.col(0).dot(W.col(0));
    sys->inner_product_of_c_and_e = 2 * constraint_affine_(0);
    sys->AQe.noalias() = 2 * WA.transpose() * W;
    sys->Ae = 2 * constraint_matrix_.row(0).transpose();

  } else {
    (*G).noalias() += 2 * WA.transpose() * WA;
    sys->AW.noalias() += 2 * constraint_matrix_.transpose() * W;
    sys->AQc.noalias() += 2 * WA.transpose() * WsqrtC;

    sys->AQe.noalias() += 2 * WA.transpose() * W;
    sys->Ae += 2 * constraint_matrix_.row(0);

    sys->inner_product_of_w_and_c += 2 * WsqrtC(0);
    sys->inner_product_of_c_and_Qc += 2 * WsqrtC.squaredNorm();
    sys->inner_product_of_c_and_Qe += 2 * WsqrtC.col(0).dot(W.col(0));
    sys->inner_product_of_c_and_e += 2 * constraint_affine_(0);
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

CONEX_STATUS SOCConstraint::UpdateLinearOperatorImpl(double val, int var, int r,
                                                     int c, int dim) {
  CONEX_RETURN_ON_FAIL(dim == 0, "Complex second-order cone not supported.");
  CONEX_RETURN_ON_FAIL(c == 0, "Second-order constraint is not matrix valued.");
  CONEX_RETURN_ON_FAIL(r <= n_, "Row index out of bounds.");
  CONEX_RETURN_ON_FAIL((var >= 0) && (r >= 0), "Indices cannot be negative.");

  ConservativeResizeHelper(&constraint_matrix_, var, n_ + 1);
  constraint_matrix_(r, var) = val;
  return CONEX_SUCCESS;
}

CONEX_STATUS SOCConstraint::UpdateAffineTermImpl(double val, int r, int c,
                                                 int dim) {
  CONEX_RETURN_ON_FAIL(dim == 0, "Complex second-order cone not supported.");
  CONEX_RETURN_ON_FAIL(c == 0, "Second-order constraint is not matrix valued.");
  CONEX_RETURN_ON_FAIL(r <= n_, "Row index out of bounds.");
  CONEX_RETURN_ON_FAIL(r >= 0, "Indices cannot be negative.");

  ConservativeResizeHelper(&constraint_affine_, 0, n_ + 1);
  constraint_affine_(r) = val;
  return CONEX_SUCCESS;
}

void SOCGramEvaluator::set_order(const std::vector<int>& perm) {
  if (order_set_) return;
  const int nplus1 = A_->rows();
  const int m = static_cast<int>(perm.size());
  num_vars_ = m;
  A_perm_.resize(nplus1, m);
  for (int i = 0; i < m; ++i) {
    A_perm_.col(i) = A_->col(perm[i]);
  }
  WA_perm_.resize(nplus1, m);
  order_set_ = true;
  update_weights();
}

void SOCGramEvaluator::update_weights() {
  if (!order_set_) return;
  auto Wsqrt = Sqrt(*ws_->W0, ws_->W1);
  const int m = A_perm_.cols();
  const double sqrt2 = std::sqrt(2.0);
  for (int i = 0; i < m; ++i) {
    WA_perm_.col(i) = sqrt2 * QuadraticRepresentation(Wsqrt, A_perm_.col(i));
  }
}

void SOCGramEvaluator::add_block(int row, int col, int rows, int cols,
                                  Eigen::Ref<Eigen::MatrixXd> dest) const {
  dest.noalias() += WA_perm_.middleCols(row, rows).transpose() *
                    WA_perm_.middleCols(col, cols);
}

void SOCGramEvaluator::add_block_lower(
    int pos, int size, Eigen::Ref<Eigen::MatrixXd> dest) const {
  dest.selfadjointView<Eigen::Lower>().rankUpdate(
      WA_perm_.middleCols(pos, size).transpose());
}

}  // namespace conex
