// Free functions on EuclideanJordanAlgebra::Variable.
// Each function iterates segments and dispatches to the per-segment ConeOps.

#pragma once
#include "conex/common/cone_ops.h"
#include "conex/common/tree_rhs.h"

namespace conex {
namespace EuclideanJordanAlgebra {

// Element-wise product (Jordan product).
inline Variable cwiseProduct(const Variable& a, const Variable& b) {
  Variable out = a;
  for (int i = 0; i < a.num_constraints(); ++i)
    a.ops[i]->product(&out.data(a.offsets[i]), &a.data(a.offsets[i]),
                       &b.data(b.offsets[i]), a.sizes[i]);
  return out;
}

// Element-wise quotient (Jordan division).
inline Variable cwiseQuotient(const Variable& a, const Variable& b) {
  Variable out = a;
  for (int i = 0; i < a.num_constraints(); ++i)
    a.ops[i]->quotient(&out.data(a.offsets[i]), &a.data(a.offsets[i]),
                        &b.data(b.offsets[i]), a.sizes[i]);
  return out;
}

// out = alpha * a + beta * b.
inline Variable addScaled(const Variable& a, const Variable& b,
                          double alpha, double beta) {
  Variable out = a;
  out.data = alpha * a.data + beta * b.data;
  return out;
}

// Geodesic update: W *= exp(alpha * d).
inline void geodesicUpdate(Variable& W, double alpha, const Variable& d) {
  for (int i = 0; i < W.num_constraints(); ++i)
    W.ops[i]->geodesicUpdate(&W.data(W.offsets[i]), &W.data(W.offsets[i]),
                              alpha, &d.data(d.offsets[i]), W.sizes[i]);
}

// Set to identity element.
inline void setOnes(Variable& v) {
  for (int i = 0; i < v.num_constraints(); ++i)
    v.ops[i]->setIdentity(&v.data(v.offsets[i]), v.sizes[i]);
}

// ||a||_inf.
inline double normInf(const Variable& a) {
  double result = 0;
  for (int i = 0; i < a.num_constraints(); ++i)
    result = std::max(result,
                      a.ops[i]->normInf(&a.data(a.offsets[i]), a.sizes[i]));
  return result;
}

// ||a||^2.
inline double squaredNorm(const Variable& a) {
  double result = 0;
  for (int i = 0; i < a.num_constraints(); ++i)
    result += a.ops[i]->squaredNorm(&a.data(a.offsets[i]), a.sizes[i]);
  return result;
}

// <a, b>.
inline double dot(const Variable& a, const Variable& b) {
  double result = 0;
  for (int i = 0; i < a.num_constraints(); ++i)
    result += a.ops[i]->dot(&a.data(a.offsets[i]), &b.data(b.offsets[i]),
                             a.sizes[i]);
  return result;
}

// gap(r, d) = <r.*(1+d), r.*(1-d)> = sum r_i^2 * (1 - d_i^2).
inline double gap(const Variable& r, const Variable& d) {
  return dot(cwiseProduct(r, r),
             addScaled(Variable{Eigen::MatrixXd::Ones(r.data.rows(), r.data.cols()),
                                r.offsets, r.sizes, r.ops},
                       cwiseProduct(d, d), 1.0, -1.0));
}

// min_i(r_i - |r_i * d_i|).
inline double minSlack(const Variable& r, const Variable& d) {
  return (r.data - r.data.cwiseProduct(d.data.cwiseAbs())).minCoeff();
}

// r_i *= (1 + |d_i|) / 2.
inline void shrinkR(Variable& r, const Variable& d) {
  r.data = 0.5 * r.data.cwiseProduct(
      Eigen::MatrixXd::Ones(d.data.rows(), d.data.cols()) + d.data.cwiseAbs());
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

}  // namespace conex
