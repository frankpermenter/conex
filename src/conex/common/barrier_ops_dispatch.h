// Free functions dispatching through BarrierConeOperations only.
//
// These work for ALL cone types (barrier, third-deriv, symmetric).
// No dynamic_cast, no sym_ops() — just virtual dispatch on the base class.
//
// Symmetric-only dispatchers (product, sqrt, geodesicUpdate, etc.) remain
// in eja_ops.h.

#pragma once
#include "conex/common/symmetric_cone_operations.h"
#include "conex/common/tree_rhs.h"

namespace conex {
namespace EuclideanJordanAlgebra {

// Helper: create a Variable with same layout as src (heap-allocated).
inline Variable like(const Variable& src) {
  Variable out;
  out.offsets = src.offsets;
  out.sizes = src.sizes;
  out.ops = src.ops;
  out.setZero(src.total_rows(), src.cols());
  return out;
}

// out = alpha * a + beta * b (output-parameter form).
inline void addScaled(Variable& out, const Variable& a, const Variable& b,
                      double alpha, double beta) {
  for (int i = 0; i < a.num_constraints(); ++i) {
    int sz = a.sizes[i];
    const double* ap = a.segment_ptr(i);
    const double* bp = b.segment_ptr(i);
    double* op = out.segment_ptr(i);
    for (int j = 0; j < sz; ++j)
      op[j] = alpha * ap[j] + beta * bp[j];
  }
}

// out = alpha * a + beta * b (allocating convenience wrapper).
inline Variable addScaled(const Variable& a, const Variable& b,
                          double alpha, double beta) {
  Variable out = like(a);
  addScaled(out, a, b, alpha, beta);
  return out;
}

// Initialize from a VectorXd (copies data into col 0).
inline void setFromVector(Variable& v, const Eigen::VectorXd& vec) {
  v.col() = vec;
}

// --- Dispatchers through BarrierConeOperations (no cast needed) ---

// <a, b>.
inline double dot(const Variable& a, const Variable& b) {
  double result = 0;
  for (int i = 0; i < a.num_constraints(); ++i)
    result += a.ops[i]->dot(a.segment_ptr(i), b.segment_ptr(i), a.sizes[i]);
  return result;
}

// ||a||^2.
inline double squaredNorm(const Variable& a) {
  double result = 0;
  for (int i = 0; i < a.num_constraints(); ++i)
    result += a.ops[i]->squaredNorm(a.segment_ptr(i), a.sizes[i]);
  return result;
}

// Recover raw cone point from stored representation (output-parameter form).
inline void getConePoint(Variable& out, const Variable& stored) {
  for (int i = 0; i < stored.num_constraints(); ++i)
    stored.ops[i]->getConePoint(out.segment_ptr(i), stored.segment_ptr(i),
                                stored.sizes[i]);
}

// Recover raw cone point (allocating convenience wrapper).
inline Variable getConePoint(const Variable& stored) {
  Variable out = like(stored);
  getConePoint(out, stored);
  return out;
}

// Interior point (output-parameter form).
inline void getInteriorPoint(Variable& out, const Variable& templ) {
  for (int i = 0; i < templ.num_constraints(); ++i)
    templ.ops[i]->getInteriorPoint(out.segment_ptr(i), templ.sizes[i]);
}

// Interior point (allocating convenience wrapper).
inline Variable getInteriorPoint(const Variable& templ) {
  Variable out = like(templ);
  getInteriorPoint(out, templ);
  return out;
}

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

// Bring barrier-level free functions into conex namespace.
namespace EJA = EuclideanJordanAlgebra;
using EuclideanJordanAlgebra::addScaled;
using EuclideanJordanAlgebra::squaredNorm;
using EuclideanJordanAlgebra::dot;
using EuclideanJordanAlgebra::setFromVector;
using EuclideanJordanAlgebra::computeGradient;
using EuclideanJordanAlgebra::hessianProduct;
using EuclideanJordanAlgebra::hessianNormSquared;
using EuclideanJordanAlgebra::stepSize;
using EuclideanJordanAlgebra::geodesicStepTarget;
using EuclideanJordanAlgebra::lineSearchTarget;
using EuclideanJordanAlgebra::barrierParameter;

}  // namespace conex
