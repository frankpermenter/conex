// Free functions on EuclideanJordanAlgebra::Variable.
// Each function iterates segments and dispatches to the per-segment SymmetricConeOperations.

#pragma once
#include "conex/common/symmetric_cone_operations.h"
#include "conex/common/tree_rhs.h"

namespace conex {
namespace EuclideanJordanAlgebra {

// Helper: create a Variable with same layout as src.
inline Variable like(const Variable& src) {
  Variable out;
  out.offsets = src.offsets;
  out.sizes = src.sizes;
  out.ops = src.ops;
  out.setZero(src.total_rows(), src.cols());
  return out;
}

// Element-wise product (Jordan product).
inline Variable cwiseProduct(const Variable& a, const Variable& b) {
  Variable out = like(a);
  for (int i = 0; i < a.num_constraints(); ++i)
    a.ops[i]->product(out.segment_ptr(i), a.segment_ptr(i),
                      b.segment_ptr(i), a.sizes[i]);
  return out;
}

// Symmetric square root.
inline Variable sqrt(const Variable& a) {
  Variable out = like(a);
  for (int i = 0; i < a.num_constraints(); ++i)
    a.ops[i]->sqrt(out.segment_ptr(i), a.segment_ptr(i), a.sizes[i]);
  return out;
}

// Quadratic representation: P(a)b.
//   Nonneg: out_i = a_i² * b_i.
//   PSD:    Out = A * B * A.
inline Variable quadraticRepresentation(const Variable& a, const Variable& b) {
  Variable out = like(a);
  for (int i = 0; i < a.num_constraints(); ++i)
    a.ops[i]->quadraticRepresentation(out.segment_ptr(i), a.segment_ptr(i),
                                      b.segment_ptr(i), a.sizes[i]);
  return out;
}

// out = alpha * a + beta * b.
inline Variable addScaled(const Variable& a, const Variable& b,
                          double alpha, double beta) {
  Variable out = like(a);
  // addScaled is a linear operation — same for all cone types.
  for (int i = 0; i < a.num_constraints(); ++i) {
    int sz = a.sizes[i];
    const double* ap = a.segment_ptr(i);
    const double* bp = b.segment_ptr(i);
    double* op = out.segment_ptr(i);
    for (int j = 0; j < sz; ++j)
      op[j] = alpha * ap[j] + beta * bp[j];
  }
  return out;
}

// Geodesic update: W *= exp(alpha * d).  (No R tracking.)
inline void geodesicUpdate(Variable& W, double alpha, const Variable& d) {
  for (int i = 0; i < W.num_constraints(); ++i)
    W.ops[i]->geodesicUpdate(W.segment_ptr(i), W.segment_ptr(i),
                             alpha, d.segment_ptr(i), W.sizes[i]);
}

// Sqrt-free geodesic update from raw slack S = -(k*b + A*y).
// PSD: W_new = exp(α(I + WS))·W (no eigendecomposition of W).
inline void geodesicUpdateFromSlack(Variable& W, double alpha,
                                    const Variable& slack) {
  for (int i = 0; i < W.num_constraints(); ++i)
    W.ops[i]->geodesicUpdateFromSlack(W.segment_ptr(i), W.segment_ptr(i),
                                      alpha, slack.segment_ptr(i), W.sizes[i]);
}

// Update automorphism: T <- T exp(alpha*D/2), polar decompose,
// update W = P^2 and R = T^T R T.
inline void updateAutomorphism(Variable& W, Variable& R, double alpha,
                               const Variable& d) {
  for (int i = 0; i < W.num_constraints(); ++i)
    W.ops[i]->updateAutomorphism(W.segment_ptr(i), R.segment_ptr(i),
                                 alpha, d.segment_ptr(i), W.sizes[i]);
}

// Same as updateAutomorphism but operates on P = sqrt(W) directly.
// Updates P in-place to P_new (not P_new²), avoiding an eigendecomposition.
inline void updateAutomorphismP(Variable& P, Variable& R, double alpha,
                                const Variable& d) {
  for (int i = 0; i < P.num_constraints(); ++i)
    P.ops[i]->updateAutomorphismP(P.segment_ptr(i), R.segment_ptr(i),
                                  alpha, d.segment_ptr(i), P.sizes[i]);
}

// Polar-free automorphism update: M_new = M_old * exp(alpha*D/2).
// PSD: r unchanged (rotation absorbed into M).
// SOC: r rotated by polar T (O(n), since polar is spectral).
// Nonneg: r unchanged (T = I).
inline void updateM(Variable& M, Variable& r, double alpha,
                    const Variable& d) {
  for (int i = 0; i < M.num_constraints(); ++i)
    M.ops[i]->updateM(M.segment_ptr(i), r.segment_ptr(i), alpha,
                       d.segment_ptr(i), M.sizes[i]);
}

// Apply automorphism: out = M * x * M^T.
inline Variable applyM(const Variable& M, const Variable& x) {
  Variable out = like(x);
  for (int i = 0; i < x.num_constraints(); ++i)
    x.ops[i]->applyM(out.segment_ptr(i), M.segment_ptr(i),
                      x.segment_ptr(i), x.sizes[i]);
  return out;
}

// Apply transpose automorphism: out = M^T * x * M.
inline Variable applyMt(const Variable& M, const Variable& x) {
  Variable out = like(x);
  for (int i = 0; i < x.num_constraints(); ++i)
    x.ops[i]->applyMt(out.segment_ptr(i), M.segment_ptr(i),
                       x.segment_ptr(i), x.sizes[i]);
  return out;
}

// Compute W = M * M^T (symmetric scaling from automorphism).
inline Variable squareM(const Variable& M) {
  Variable out = like(M);
  for (int i = 0; i < M.num_constraints(); ++i)
    M.ops[i]->squareM(out.segment_ptr(i), M.segment_ptr(i), M.sizes[i]);
  return out;
}

// Solve Lyapunov R*D + D*R = 2*Delta for D.: solve R*D + D*R = 2*Delta for D.
inline Variable solveLyapunovForD(const Variable& r, const Variable& delta) {
  Variable out = like(r);
  for (int i = 0; i < r.num_constraints(); ++i)
    r.ops[i]->solveLyapunovForD(out.segment_ptr(i), r.segment_ptr(i),
                                 delta.segment_ptr(i), r.sizes[i]);
  return out;
}

// EJA absolute value (eigenvalue abs for PSD, elementwise for nonneg).
inline Variable absEJA(const Variable& a) {
  Variable out = like(a);
  for (int i = 0; i < a.num_constraints(); ++i)
    a.ops[i]->abs(out.segment_ptr(i), a.segment_ptr(i), a.sizes[i]);
  return out;
}

// Minimum eigenvalue across all segments.
inline double minEigenvalue(const Variable& a) {
  double result = std::numeric_limits<double>::max();
  for (int i = 0; i < a.num_constraints(); ++i)
    result = std::min(result,
                      a.ops[i]->minEigenvalue(a.segment_ptr(i), a.sizes[i]));
  return result;
}

// Set to identity element.
inline void setOnes(Variable& v) {
  for (int i = 0; i < v.num_constraints(); ++i)
    v.ops[i]->setIdentity(v.segment_ptr(i), v.sizes[i]);
}

// Compute W = P² = quadraticRepresentation(P, e).
inline Variable square(const Variable& P) {
  Variable ones = like(P);
  setOnes(ones);
  return quadraticRepresentation(P, ones);
}

// ||a||_inf.
inline double normInf(const Variable& a) {
  double result = 0;
  for (int i = 0; i < a.num_constraints(); ++i)
    result = std::max(result,
                      a.ops[i]->normInf(a.segment_ptr(i), a.sizes[i]));
  return result;
}

// ||a||^2.
inline double squaredNorm(const Variable& a) {
  double result = 0;
  for (int i = 0; i < a.num_constraints(); ++i)
    result += a.ops[i]->squaredNorm(a.segment_ptr(i), a.sizes[i]);
  return result;
}

// <a, b>.
inline double dot(const Variable& a, const Variable& b) {
  double result = 0;
  for (int i = 0; i < a.num_constraints(); ++i)
    result += a.ops[i]->dot(a.segment_ptr(i), b.segment_ptr(i),
                            a.sizes[i]);
  return result;
}

// Gap in terms of (R, Delta): ||R||² - ||Delta||².
// Equivalent to old gap(r, d) since Delta = R*D for nonneg.
inline double gap(const Variable& r, const Variable& delta) {
  return squaredNorm(r) - squaredNorm(delta);
}

// Minimum eigenvalue of (R - |Delta|).
// Equivalent to old minSlack(r, d) = min(r_i - |r_i*d_i|) for nonneg.
inline double minSlack(const Variable& r, const Variable& delta) {
  Variable abs_delta = absEJA(delta);
  Variable slack = addScaled(r, abs_delta, 1.0, -1.0);
  return minEigenvalue(slack);
}

// Shrink: R = (R + |Delta|) / 2.
// Equivalent to old r_i *= (1 + |d_i|) / 2 since |delta_i| = r_i*|d_i|.
inline void shrinkR(Variable& r, const Variable& delta) {
  Variable abs_delta = absEJA(delta);
  Variable sum = addScaled(r, abs_delta, 1.0, 1.0);
  // Copy back into r.
  for (int i = 0; i < r.num_constraints(); ++i) {
    int sz = r.sizes[i];
    const double* sp = sum.segment_ptr(i);
    double* rp = r.segment_ptr(i);
    for (int j = 0; j < sz; ++j) rp[j] = 0.5 * sp[j];
  }
}

// Largest k > 0 with ||d0 + k*d1||_inf <= 1 per segment.
//   Nonneg: per-element bound.
//   PSD:    GEV on (D1, I ± D0).
// Note: each block's feasible set is an interval [k_low, k_high].
// Taking min(k_high_i) can land below another block's k_low when
// ||d0_block|| > 1.  Verify and bisect if needed.
inline double lineSearchK(const Variable& d0, const Variable& d1) {
  double k_max = std::numeric_limits<double>::max();
  for (int i = 0; i < d0.num_constraints(); ++i)
    k_max = std::min(k_max,
        d0.ops[i]->lineSearchK(d0.segment_ptr(i), d1.segment_ptr(i),
                               d0.sizes[i]));
  if (k_max <= 0 || k_max >= 1e15) return k_max;

  // Verify: check that all blocks satisfy the bound at k_max.
  double actual_norm = normInf(addScaled(d0, d1, 1.0, k_max));
  if (actual_norm <= 1.0 + 1e-10) return k_max;

  // Bisect: some block's k_low > k_max.  Find the largest feasible k.
  double lo = 0, hi = k_max;
  // Check if k=0 is feasible (||d0||_inf <= 1).
  if (normInf(d0) > 1.0) return 0;
  for (int b = 0; b < 60; ++b) {
    double mid = 0.5 * (lo + hi);
    if (normInf(addScaled(d0, d1, 1.0, mid)) <= 1.0)
      lo = mid;
    else
      hi = mid;
  }
  return lo;
}

// Largest k > 0 with ||d0 + k*d1||_inf <= bound.  Equivalent to scaling
// d0 and d1 by 1/bound and using the unit-bound version.
inline double lineSearchK(const Variable& d0, const Variable& d1,
                          double bound) {
  if (bound <= 0) return 0.0;
  double inv = 1.0 / bound;
  Variable d0_s = like(d0);
  Variable d1_s = like(d1);
  for (int i = 0; i < d0.num_constraints(); ++i) {
    int sz = d0.sizes[i];
    const double* p0 = d0.segment_ptr(i);
    const double* p1 = d1.segment_ptr(i);
    double* q0 = d0_s.segment_ptr(i);
    double* q1 = d1_s.segment_ptr(i);
    for (int j = 0; j < sz; ++j) {
      q0[j] = p0[j] * inv;
      q1[j] = p1[j] * inv;
    }
  }
  return lineSearchK(d0_s, d1_s);
}

// Project onto the cone.
inline void project(Variable& out, const Variable& a) {
  for (int i = 0; i < a.num_constraints(); ++i)
    a.ops[i]->project(out.segment_ptr(i), a.segment_ptr(i), a.sizes[i]);
}

// Initialize from a VectorXd (copies data into col 0).
inline void setFromVector(Variable& v, const Eigen::VectorXd& vec) {
  v.col() = vec;
}

// --- z-space operations for geodesic IPM on general cones ---

inline void computeGradient(const Variable& z, Variable& grad) {
  for (int i = 0; i < z.num_constraints(); ++i)
    z.ops[i]->computeGradient(grad.segment_ptr(i), z.segment_ptr(i),
                              z.sizes[i]);
}

inline void hessianProduct(const Variable& z, const Variable& v,
                           Variable& out) {
  for (int i = 0; i < z.num_constraints(); ++i)
    z.ops[i]->hessianProduct(out.segment_ptr(i), z.segment_ptr(i),
                             v.segment_ptr(i), z.sizes[i]);
}

inline double hessianNormSquared(const Variable& z, const Variable& target) {
  double result = 0;
  for (int i = 0; i < z.num_constraints(); ++i)
    result += z.ops[i]->hessianNormSquared(z.segment_ptr(i),
                                           target.segment_ptr(i), z.sizes[i]);
  return result;
}

inline double stepSize(const Variable& z, const Variable& target) {
  double alpha = std::numeric_limits<double>::max();
  for (int i = 0; i < z.num_constraints(); ++i)
    alpha = std::min(alpha,
        z.ops[i]->stepSize(z.segment_ptr(i), target.segment_ptr(i),
                           z.sizes[i]));
  return alpha;
}

inline void geodesicStepTarget(Variable& z, double alpha,
                               const Variable& target) {
  for (int i = 0; i < z.num_constraints(); ++i)
    z.ops[i]->geodesicStepTarget(z.segment_ptr(i), alpha,
                                 target.segment_ptr(i), z.sizes[i]);
}

inline double lineSearchTarget(const Variable& z, const Variable& target0,
                               const Variable& target1) {
  double k_max = std::numeric_limits<double>::max();
  for (int i = 0; i < z.num_constraints(); ++i)
    k_max = std::min(k_max,
        z.ops[i]->lineSearchTarget(z.segment_ptr(i), target0.segment_ptr(i),
                                   target1.segment_ptr(i), z.sizes[i]));
  return k_max;
}

inline double barrierParameter(const Variable& z) {
  double nu = 0;
  for (int i = 0; i < z.num_constraints(); ++i)
    nu += z.ops[i]->barrierParameter(z.sizes[i]);
  return nu;
}

}  // namespace EuclideanJordanAlgebra

// Bring free functions into conex namespace.
namespace EJA = EuclideanJordanAlgebra;
using EuclideanJordanAlgebra::cwiseProduct;
using EuclideanJordanAlgebra::quadraticRepresentation;
using EuclideanJordanAlgebra::addScaled;
using EuclideanJordanAlgebra::geodesicUpdate;
using EuclideanJordanAlgebra::geodesicUpdateFromSlack;
using EuclideanJordanAlgebra::updateAutomorphism;
using EuclideanJordanAlgebra::updateAutomorphismP;
using EuclideanJordanAlgebra::square;
using EuclideanJordanAlgebra::absEJA;
using EuclideanJordanAlgebra::minEigenvalue;
using EuclideanJordanAlgebra::setOnes;
using EuclideanJordanAlgebra::normInf;
using EuclideanJordanAlgebra::squaredNorm;
using EuclideanJordanAlgebra::dot;
using EuclideanJordanAlgebra::gap;
using EuclideanJordanAlgebra::minSlack;
using EuclideanJordanAlgebra::shrinkR;
using EuclideanJordanAlgebra::setFromVector;
using EuclideanJordanAlgebra::lineSearchK;
using EuclideanJordanAlgebra::project;
using EuclideanJordanAlgebra::computeGradient;
using EuclideanJordanAlgebra::hessianProduct;
using EuclideanJordanAlgebra::hessianNormSquared;
using EuclideanJordanAlgebra::stepSize;
using EuclideanJordanAlgebra::geodesicStepTarget;
using EuclideanJordanAlgebra::lineSearchTarget;
using EuclideanJordanAlgebra::barrierParameter;

}  // namespace conex
