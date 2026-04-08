// Free functions on EuclideanJordanAlgebra::Variable for cone-generic algorithms.
// Current implementations are element-wise (linear constraints / nonneg orthant).
// SDP dispatch will be added when PSD constraints are introduced.

#pragma once
#include "conex/common/tree_rhs.h"

namespace conex {
namespace EuclideanJordanAlgebra {

using Variable = ::conex::EuclideanJordanAlgebra::Variable;

// Element-wise product: out_i = a_i * b_i.
inline Variable cwiseProduct(const Variable& a, const Variable& b) {
  Variable out = a;
  out.data = a.data.cwiseProduct(b.data);
  return out;
}

// Element-wise quotient: out_i = a_i / b_i.
inline Variable cwiseQuotient(const Variable& a, const Variable& b) {
  Variable out = a;
  out.data = a.data.cwiseQuotient(b.data);
  return out;
}

// out = alpha * a + beta * b.
inline Variable addScaled(const Variable& a, const Variable& b,
                          double alpha, double beta) {
  Variable out = a;
  out.data = alpha * a.data + beta * b.data;
  return out;
}

// Geodesic update: W_i *= exp(alpha * d_i).
inline void geodesicUpdate(Variable& W, double alpha, const Variable& d) {
  W.data = W.data.cwiseProduct((alpha * d.data).array().exp().matrix());
}

// Set all entries to identity element (1 for linear, I for SDP).
inline void setOnes(Variable& v) {
  v.data.setOnes();
}

// ||a||_inf = max |a_i|.
inline double normInf(const Variable& a) {
  return a.data.lpNorm<Eigen::Infinity>();
}

// ||a||^2 = sum a_i^2.
inline double squaredNorm(const Variable& a) {
  return a.data.squaredNorm();
}

// <a, b> = sum a_i * b_i.
inline double dot(const Variable& a, const Variable& b) {
  return (a.data.cwiseProduct(b.data)).sum();
}

// gap(r, d) = <r.*(1+d), r.*(1-d)> = sum r_i^2 * (1 - d_i^2).
inline double gap(const Variable& r, const Variable& d) {
  Eigen::MatrixXd r2 = r.data.cwiseProduct(r.data);
  Eigen::MatrixXd d2 = d.data.cwiseProduct(d.data);
  return (r2 - r2.cwiseProduct(d2)).sum();
}

// min_i(r_i - |r_i * d_i|) = min_i r_i * (1 - |d_i|).
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

// Bring free functions into conex namespace via ADL-friendly using declarations.
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
