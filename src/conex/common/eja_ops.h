// Free functions on EuclideanJordanAlgebra::Variable.
//
// This file provides symmetric-cone dispatchers (product, sqrt,
// geodesicUpdate, quadraticRepresentation, etc.) that go through
// SymmetricConeOperations — only valid for segments backed by
// symmetric cones (nonneg, SOC, PSD).
//
// Barrier-level dispatchers (dot, computeGradient, hessianProduct, etc.)
// live in barrier_ops_dispatch.h (included below).

#pragma once
#include "conex/common/barrier_ops_dispatch.h"

namespace conex {
namespace EuclideanJordanAlgebra {

// Cast a BarrierConeOperations* to SymmetricConeOperations*.
// Only valid for segments backed by symmetric cones.
inline const SymmetricConeOperations* sym_ops(const BarrierConeOperations* o) {
  return static_cast<const SymmetricConeOperations*>(o);
}

// --- Output-parameter primary forms ---

inline void cwiseProduct(Variable& out, const Variable& a, const Variable& b) {
  for (int i = 0; i < a.num_constraints(); ++i)
    sym_ops(a.ops[i])->product(out.segment_ptr(i), a.segment_ptr(i),
                      b.segment_ptr(i), a.sizes[i]);
}

inline void sqrt(Variable& out, const Variable& a) {
  for (int i = 0; i < a.num_constraints(); ++i)
    sym_ops(a.ops[i])->sqrt(out.segment_ptr(i), a.segment_ptr(i), a.sizes[i]);
}

inline void inverse(Variable& out, const Variable& a) {
  for (int i = 0; i < a.num_constraints(); ++i)
    sym_ops(a.ops[i])->inverse(out.segment_ptr(i), a.segment_ptr(i), a.sizes[i]);
}

inline void quadraticRepresentation(Variable& out, const Variable& a, const Variable& b) {
  for (int i = 0; i < a.num_constraints(); ++i)
    sym_ops(a.ops[i])->quadraticRepresentation(out.segment_ptr(i), a.segment_ptr(i),
                                      b.segment_ptr(i), a.sizes[i]);
}

inline void applyM(Variable& out, const Variable& M, const Variable& x) {
  for (int i = 0; i < x.num_constraints(); ++i)
    sym_ops(x.ops[i])->applyM(out.segment_ptr(i), M.segment_ptr(i),
                      x.segment_ptr(i), x.sizes[i]);
}

inline void applyMt(Variable& out, const Variable& M, const Variable& x) {
  for (int i = 0; i < x.num_constraints(); ++i)
    sym_ops(x.ops[i])->applyMt(out.segment_ptr(i), M.segment_ptr(i),
                       x.segment_ptr(i), x.sizes[i]);
}

inline void squareM(Variable& out, const Variable& M) {
  for (int i = 0; i < M.num_constraints(); ++i)
    sym_ops(M.ops[i])->squareM(out.segment_ptr(i), M.segment_ptr(i), M.sizes[i]);
}

inline void solveLyapunovForD(Variable& out, const Variable& r, const Variable& delta) {
  for (int i = 0; i < r.num_constraints(); ++i)
    sym_ops(r.ops[i])->solveLyapunovForD(out.segment_ptr(i), r.segment_ptr(i),
                                 delta.segment_ptr(i), r.sizes[i]);
}

inline void absEJA(Variable& out, const Variable& a) {
  for (int i = 0; i < a.num_constraints(); ++i)
    sym_ops(a.ops[i])->abs(out.segment_ptr(i), a.segment_ptr(i), a.sizes[i]);
}

inline void project(Variable& out, const Variable& a) {
  for (int i = 0; i < a.num_constraints(); ++i)
    sym_ops(a.ops[i])->project(out.segment_ptr(i), a.segment_ptr(i), a.sizes[i]);
}

// --- Allocating convenience wrappers ---

inline Variable cwiseProduct(const Variable& a, const Variable& b) {
  Variable out = like(a);
  cwiseProduct(out, a, b);
  return out;
}

inline Variable sqrt(const Variable& a) {
  Variable out = like(a);
  sqrt(out, a);
  return out;
}

inline Variable inverse(const Variable& a) {
  Variable out = like(a);
  inverse(out, a);
  return out;
}

inline Variable quadraticRepresentation(const Variable& a, const Variable& b) {
  Variable out = like(a);
  quadraticRepresentation(out, a, b);
  return out;
}

inline Variable applyM(const Variable& M, const Variable& x) {
  Variable out = like(x);
  applyM(out, M, x);
  return out;
}

inline Variable applyMt(const Variable& M, const Variable& x) {
  Variable out = like(x);
  applyMt(out, M, x);
  return out;
}

inline Variable squareM(const Variable& M) {
  Variable out = like(M);
  squareM(out, M);
  return out;
}

inline Variable solveLyapunovForD(const Variable& r, const Variable& delta) {
  Variable out = like(r);
  solveLyapunovForD(out, r, delta);
  return out;
}

inline Variable absEJA(const Variable& a) {
  Variable out = like(a);
  absEJA(out, a);
  return out;
}

// --- In-place operations (no allocation) ---

inline void geodesicUpdate(Variable& W, double alpha, const Variable& d) {
  for (int i = 0; i < W.num_constraints(); ++i)
    sym_ops(W.ops[i])->geodesicUpdate(W.segment_ptr(i), W.segment_ptr(i),
                             alpha, d.segment_ptr(i), W.sizes[i]);
}

inline void geodesicUpdateFromSlack(Variable& W, double alpha,
                                    const Variable& slack) {
  for (int i = 0; i < W.num_constraints(); ++i)
    sym_ops(W.ops[i])->geodesicUpdateFromSlack(W.segment_ptr(i), W.segment_ptr(i),
                                      alpha, slack.segment_ptr(i), W.sizes[i]);
}

inline void updateAutomorphism(Variable& W, Variable& R, double alpha,
                               const Variable& d) {
  for (int i = 0; i < W.num_constraints(); ++i)
    sym_ops(W.ops[i])->updateAutomorphism(W.segment_ptr(i), R.segment_ptr(i),
                                 alpha, d.segment_ptr(i), W.sizes[i]);
}

inline void updateAutomorphismP(Variable& P, Variable& R, double alpha,
                                const Variable& d) {
  for (int i = 0; i < P.num_constraints(); ++i)
    sym_ops(P.ops[i])->updateAutomorphismP(P.segment_ptr(i), R.segment_ptr(i),
                                  alpha, d.segment_ptr(i), P.sizes[i]);
}

inline void updateM(Variable& M, Variable& r, double alpha,
                    const Variable& d) {
  for (int i = 0; i < M.num_constraints(); ++i)
    sym_ops(M.ops[i])->updateM(M.segment_ptr(i), r.segment_ptr(i), alpha,
                       d.segment_ptr(i), M.sizes[i]);
}

// --- Scalar queries (no allocation) ---

inline double minEigenvalue(const Variable& a) {
  double result = std::numeric_limits<double>::max();
  for (int i = 0; i < a.num_constraints(); ++i)
    result = std::min(result,
                      sym_ops(a.ops[i])->minEigenvalue(a.segment_ptr(i), a.sizes[i]));
  return result;
}

inline double normInf(const Variable& a) {
  double result = 0;
  for (int i = 0; i < a.num_constraints(); ++i)
    result = std::max(result,
                      sym_ops(a.ops[i])->normInf(a.segment_ptr(i), a.sizes[i]));
  return result;
}

inline void setOnes(Variable& v) {
  for (int i = 0; i < v.num_constraints(); ++i)
    v.ops[i]->getInteriorPoint(v.segment_ptr(i), v.sizes[i]);
}

// --- Composite operations ---

inline void square(Variable& out, const Variable& P) {
  setOnes(out);
  quadraticRepresentation(out, P, out);
}

inline Variable square(const Variable& P) {
  Variable out = like(P);
  square(out, P);
  return out;
}

inline double gap(const Variable& r, const Variable& delta) {
  return squaredNorm(r) - squaredNorm(delta);
}

inline double minSlack(const Variable& r, const Variable& delta) {
  Variable abs_delta = absEJA(delta);
  Variable slack = addScaled(r, abs_delta, 1.0, -1.0);
  return minEigenvalue(slack);
}

inline void shrinkR(Variable& r, const Variable& delta) {
  Variable abs_delta = absEJA(delta);
  Variable sum = addScaled(r, abs_delta, 1.0, 1.0);
  for (int i = 0; i < r.num_constraints(); ++i) {
    int sz = r.sizes[i];
    const double* sp = sum.segment_ptr(i);
    double* rp = r.segment_ptr(i);
    for (int j = 0; j < sz; ++j) rp[j] = 0.5 * sp[j];
  }
}

inline double lineSearchK(const Variable& d0, const Variable& d1) {
  double k_max = std::numeric_limits<double>::max();
  for (int i = 0; i < d0.num_constraints(); ++i)
    k_max = std::min(k_max,
        sym_ops(d0.ops[i])->lineSearchK(d0.segment_ptr(i), d1.segment_ptr(i),
                               d0.sizes[i]));
  if (k_max <= 0 || k_max >= 1e15) return k_max;

  double actual_norm = normInf(addScaled(d0, d1, 1.0, k_max));
  if (actual_norm <= 1.0 + 1e-10) return k_max;

  double lo = 0, hi = k_max;
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

}  // namespace EuclideanJordanAlgebra

// Bring symmetric-cone free functions into conex namespace.
// (Barrier-level using declarations already provided by barrier_ops_dispatch.h.)
using EuclideanJordanAlgebra::cwiseProduct;
using EuclideanJordanAlgebra::quadraticRepresentation;
using EuclideanJordanAlgebra::geodesicUpdate;
using EuclideanJordanAlgebra::geodesicUpdateFromSlack;
using EuclideanJordanAlgebra::updateAutomorphism;
using EuclideanJordanAlgebra::updateAutomorphismP;
using EuclideanJordanAlgebra::square;
using EuclideanJordanAlgebra::absEJA;
using EuclideanJordanAlgebra::minEigenvalue;
using EuclideanJordanAlgebra::setOnes;
using EuclideanJordanAlgebra::gap;
using EuclideanJordanAlgebra::minSlack;
using EuclideanJordanAlgebra::shrinkR;
using EuclideanJordanAlgebra::lineSearchK;
using EuclideanJordanAlgebra::project;

}  // namespace conex
