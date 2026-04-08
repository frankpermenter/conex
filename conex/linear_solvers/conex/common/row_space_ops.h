// Free functions on RowSpace for implementing cone-generic algorithms.
// Current implementations are element-wise (linear constraints).
// SDP dispatch will be added when PSD constraints are introduced.

#pragma once
#include "conex/common/tree_rhs.h"

namespace conex {

// Element-wise product: out_i = a_i * b_i.
inline RowSpace cwiseProduct(const RowSpace& a, const RowSpace& b) {
  RowSpace out = a;
  out.data = a.data.cwiseProduct(b.data);
  return out;
}

// Element-wise quotient: out_i = a_i / b_i.
inline RowSpace cwiseQuotient(const RowSpace& a, const RowSpace& b) {
  RowSpace out = a;
  out.data = a.data.cwiseQuotient(b.data);
  return out;
}

// out = alpha * a + beta * b.
inline RowSpace addScaled(const RowSpace& a, const RowSpace& b,
                          double alpha, double beta) {
  RowSpace out = a;
  out.data = alpha * a.data + beta * b.data;
  return out;
}

// Geodesic update: W_i *= exp(alpha * d_i).
inline void geodesicUpdate(RowSpace& W, double alpha, const RowSpace& d) {
  W.data = W.data.cwiseProduct((alpha * d.data).array().exp().matrix());
}

// Set all entries to 1.
inline void setOnes(RowSpace& rs) {
  rs.data.setOnes();
}

// ||a||_inf = max |a_i|.
inline double normInf(const RowSpace& a) {
  return a.data.lpNorm<Eigen::Infinity>();
}

// ||a||^2 = sum a_i^2.
inline double squaredNorm(const RowSpace& a) {
  return a.data.squaredNorm();
}

// <a, b> = sum a_i * b_i.
inline double dot(const RowSpace& a, const RowSpace& b) {
  return (a.data.cwiseProduct(b.data)).sum();
}

// gap(r, d) = <r.*(1+d), r.*(1-d)> = sum r_i^2 * (1 - d_i^2).
inline double gap(const RowSpace& r, const RowSpace& d) {
  Eigen::MatrixXd r2 = r.data.cwiseProduct(r.data);
  Eigen::MatrixXd d2 = d.data.cwiseProduct(d.data);
  return (r2 - r2.cwiseProduct(d2)).sum();
}

// min_i(r_i - |r_i * d_i|) = min_i r_i * (1 - |d_i|).
inline double minSlack(const RowSpace& r, const RowSpace& d) {
  return (r.data - r.data.cwiseProduct(d.data.cwiseAbs())).minCoeff();
}

// r_i *= (1 + |d_i|) / 2.
inline void shrinkR(RowSpace& r, const RowSpace& d) {
  r.data = 0.5 * r.data.cwiseProduct(
      Eigen::MatrixXd::Ones(d.data.rows(), d.data.cols()) + d.data.cwiseAbs());
}

// Initialize a RowSpace from a VectorXd (copies data into col 0).
inline void setFromVector(RowSpace& rs, const Eigen::VectorXd& v) {
  rs.col() = v;
}

}  // namespace conex
