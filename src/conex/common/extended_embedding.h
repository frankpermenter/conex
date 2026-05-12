#pragma once
// Extended embedding (Ye et al.) for linear programs with dual equalities.
//
// Given the primal-dual pair:
//   P: min c'x + d'w  s.t. Ax + C'w = b, x >= 0  (w free)
//   D: max b'y        s.t. A'y + s = c, Cy = d, s >= 0  (y free)
//
// When C is empty (p=0), this reduces to the standard form:
//   P: min c'x  s.t. Ax = b, x >= 0
//   D: max b'y  s.t. A'y + s = c, s >= 0
//
// The extended embedding is:
//   minimize   alpha * theta
//   subject to  Ax + C'w - b*tau       = rp * theta   (E1, primal)
//              -A'y - s + c*tau        = rd * theta   (E2, dual x)
//              -Cy + d*tau             = re * theta   (E3, dual w)
//              b'y - c'x - d'w - kappa = rg * theta   (E4, gap)
//              rp'y + rd'x + re'w + rg*tau = -alpha   (E5, normalization)
//              x >= 0, s >= 0, tau >= 0, kappa >= 0
//
// Fixed point: x_hat=e, s_hat=e, w_hat=0, y_hat=0, tau_hat=1, kappa_hat=1.

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "conex/common/clique_tree.h"
#include "conex/common/model.h"

namespace conex {

struct EmbeddingInfo {
  int n;           // original primal variables (cols of A)
  int m;           // original equality constraints (rows of A)
  int p;           // dual equality constraints (rows of C), 0 if none

  // Variable layout in the embedding Model:
  //   [0, n)             : x  (primal, x >= 0)
  //   [n, n+m)           : y  (dual, free)
  //   [n+m, n+m+p)       : w  (dual eq variable, free)
  //   [n+m+p, 2n+m+p)    : s  (dual slack, s >= 0)
  //   2n+m+p             : τ  (>= 0)
  //   2n+m+p+1           : κ  (>= 0)
  //   2n+m+p+2           : θ  (free)
  int x_start() const { return 0; }
  int y_start() const { return n; }
  int w_start() const { return n + m; }
  int s_start() const { return n + m + p; }
  int tau_idx() const { return 2 * n + m + p; }
  int kappa_idx() const { return 2 * n + m + p + 1; }
  int theta_idx() const { return 2 * n + m + p + 2; }
  int total_vars() const { return 2 * n + m + p + 3; }

  // Fixed point.
  Eigen::VectorXd x_hat;
  Eigen::VectorXd s_hat;
  Eigen::VectorXd y_hat;
  Eigen::VectorXd w_hat;
  double tau_hat;
  double kappa_hat;

  // Residuals.
  Eigen::VectorXd rp;
  Eigen::VectorXd rd;
  Eigen::VectorXd re;  // dual equality residual (p-vector), zero when p=0
  double rg;
  double alpha;
};

struct ExtendedEmbedding {
  Model model;
  EmbeddingInfo info;
  CliqueTree tree;
};

// Build from raw LP data: min c'x s.t. Ax = b, x >= 0.
ExtendedEmbedding BuildExtendedEmbedding(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& c);

// Build with dual equalities: min c'x + d'w s.t. Ax + C'w = b, x >= 0.
// Dual: max b'y s.t. A'y + s = c, Cy = d, s >= 0.
ExtendedEmbedding BuildExtendedEmbedding(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double>& C,
    const Eigen::VectorXd& d);

// Extract from a Model (one linear constraint for x>=0, one or two equalities).
ExtendedEmbedding BuildExtendedEmbedding(const Model& model);

}  // namespace conex
