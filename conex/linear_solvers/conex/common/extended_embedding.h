#pragma once
// Extended embedding (Ye et al.) for linear constraints.
//
// Given the primal-dual pair:
//   P(C): min c'x  s.t. Ax - b >= 0,  x in C
//   D(C): max b'y  s.t. c - A'y in C*
//
// The extended embedding is:
//   minimize   alpha * theta
//   subject to  Ax - b*tau          = rp * theta    (primal)
//              -A'y - s + c*tau     = rd * theta    (dual)
//              b'y - c'x - kappa    = rg * theta    (gap)
//              rp'y + rd'x + rg*tau = -alpha        (normalization)
//              s >= 0, tau >= 0, kappa >= 0
//
// where rp, rd, rg are residuals at a fixed strictly feasible point
// (x_hat, s_hat, y_hat, tau_hat, kappa_hat):
//   rp = A*x_hat - b*tau_hat
//   rd = -A'*y_hat - s_hat + c*tau_hat
//   rg = b'*y_hat - c'*x_hat - kappa_hat
//
// The point (x_hat, s_hat, y_hat, tau_hat, kappa_hat, theta=1) is
// strictly feasible.  The optimal value is theta=0, at which the
// first three constraints reduce to the homogeneous model H(C).
//
// For nonneg cone (linear constraints), C = R+^m, C* = R+^m.

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/model.h"

namespace conex {

struct EmbeddingInfo {
  int n;           // original primal variables
  int m;           // original constraints (rows of A)

  // Variable layout in the embedding Model:
  //   [0, n)       : x  (primal)
  //   [n, n+m)     : y  (dual)
  //   [n+m, n+m+m) : s  (slack, for s >= 0)
  //   n+2m         : tau
  //   n+2m+1       : kappa
  //   n+2m+2       : theta
  int x_start() const { return 0; }
  int y_start() const { return n; }
  int s_start() const { return n + m; }
  int tau_idx() const { return n + 2 * m; }
  int kappa_idx() const { return n + 2 * m + 1; }
  int theta_idx() const { return n + 2 * m + 2; }
  int total_vars() const { return n + 2 * m + 3; }

  // Fixed point.
  Eigen::VectorXd x_hat;
  Eigen::VectorXd s_hat;
  Eigen::VectorXd y_hat;
  double tau_hat;
  double kappa_hat;

  // Residuals.
  Eigen::VectorXd rp;
  Eigen::VectorXd rd;
  double rg;
  double alpha;
};

// Build the extended embedding Model for a linear program.
//
// The original problem:
//   min c'x  s.t.  Ax + b >= 0  (stored internally as Ax + b >= 0)
//
// Note: the paper uses Ax - b = 0 form, so our b corresponds to -b_paper.
// We convert internally.
//
// Default fixed point: x_hat = e, s_hat = e, y_hat = 0, tau_hat = 1,
// kappa_hat = 1.
std::pair<Model, EmbeddingInfo> BuildExtendedEmbedding(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& c);

}  // namespace conex
