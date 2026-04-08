// Free functions on EuclideanJordanAlgebra::Variable.
// Each function iterates segments and dispatches to the per-segment ConeOps.

#pragma once
#include "conex/common/cone_ops.h"
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

// Element-wise quotient (Jordan division).
inline Variable cwiseQuotient(const Variable& a, const Variable& b) {
  Variable out = like(a);
  for (int i = 0; i < a.num_constraints(); ++i)
    a.ops[i]->quotient(out.segment_ptr(i), a.segment_ptr(i),
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

// Geodesic update: W *= exp(alpha * d).
inline void geodesicUpdate(Variable& W, double alpha, const Variable& d) {
  for (int i = 0; i < W.num_constraints(); ++i)
    W.ops[i]->geodesicUpdate(W.segment_ptr(i), W.segment_ptr(i),
                             alpha, d.segment_ptr(i), W.sizes[i]);
}

// Set to identity element.
inline void setOnes(Variable& v) {
  for (int i = 0; i < v.num_constraints(); ++i)
    v.ops[i]->setIdentity(v.segment_ptr(i), v.sizes[i]);
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

// gap(r, d) = <r.*(1+d), r.*(1-d)> = sum r_i^2 * (1 - d_i^2).
inline double gap(const Variable& r, const Variable& d) {
  Variable r2 = cwiseProduct(r, r);
  Variable d2 = cwiseProduct(d, d);
  Variable ones = like(r);
  setOnes(ones);
  return dot(r2, addScaled(ones, d2, 1.0, -1.0));
}

// min_i(r_i - |r_i * d_i|).
inline double minSlack(const Variable& r, const Variable& d) {
  // For nonneg orthant: r_i * (1 - |d_i|). Dispatch per segment.
  double result = std::numeric_limits<double>::max();
  for (int i = 0; i < r.num_constraints(); ++i) {
    int sz = r.sizes[i];
    const double* rp = r.segment_ptr(i);
    const double* dp = d.segment_ptr(i);
    for (int j = 0; j < sz; ++j)
      result = std::min(result, rp[j] - rp[j] * std::abs(dp[j]));
  }
  return result;
}

// r_i *= (1 + |d_i|) / 2.
inline void shrinkR(Variable& r, const Variable& d) {
  for (int i = 0; i < r.num_constraints(); ++i) {
    int sz = r.sizes[i];
    double* rp = r.segment_ptr(i);
    const double* dp = d.segment_ptr(i);
    for (int j = 0; j < sz; ++j)
      rp[j] *= 0.5 * (1.0 + std::abs(dp[j]));
  }
}

// Largest k > 0 with |d0_i + k * d1_i| <= 1 for all i.
inline double lineSearchK(const Variable& d0, const Variable& d1) {
  double k_max = std::numeric_limits<double>::max();
  for (int i = 0; i < d0.num_constraints(); ++i) {
    int sz = d0.sizes[i];
    const double* p0 = d0.segment_ptr(i);
    const double* p1 = d1.segment_ptr(i);
    for (int j = 0; j < sz; ++j) {
      if (p1[j] > 0)
        k_max = std::min(k_max, (1.0 - p0[j]) / p1[j]);
      else if (p1[j] < 0)
        k_max = std::min(k_max, (-1.0 - p0[j]) / p1[j]);
    }
  }
  return k_max;
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

}  // namespace EuclideanJordanAlgebra

// Bring free functions into conex namespace.
namespace EJA = EuclideanJordanAlgebra;
using EuclideanJordanAlgebra::cwiseProduct;
using EuclideanJordanAlgebra::cwiseQuotient;
using EuclideanJordanAlgebra::addScaled;
using EuclideanJordanAlgebra::geodesicUpdate;
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

}  // namespace conex
