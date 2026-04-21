#include "conex/common/extended_embedding.h"

#include <numeric>

namespace conex {

std::pair<Model, EmbeddingInfo> BuildExtendedEmbedding(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& c) {
  // Paper notation (equation 9):
  //   A is m×n,  b ∈ R^m,  c ∈ R^n
  //   x ∈ C = R^n_+,  s ∈ C* = R^n_+
  //   y ∈ R^m,  τ ≥ 0,  κ ≥ 0,  θ ∈ R
  //
  //   Ax - bτ        = rp θ        (m equations)
  //   -A*y - s + cτ  = rd θ        (n equations)
  //   <b,y> - <c,x> - κ = rg θ    (1 equation)
  //   <rp,y> + <rd,x> + rg τ = -α  (1 equation)
  //
  //   x ≥ 0, s ≥ 0, τ ≥ 0, κ ≥ 0
  //   cost: min α θ

  const int m = A.rows();
  const int n = A.cols();

  EmbeddingInfo info;
  info.n = n;
  info.m = m;

  // Default fixed point: x_hat = e_n, s_hat = e_n, y_hat = 0, τ_hat = 1, κ_hat = 1.
  info.x_hat = Eigen::VectorXd::Ones(n);
  info.s_hat = Eigen::VectorXd::Ones(n);
  info.y_hat = Eigen::VectorXd::Zero(m);
  info.tau_hat = 1.0;
  info.kappa_hat = 1.0;

  // Residuals at the fixed point:
  //   rp = A x_hat - b τ_hat
  //   rd = -A* y_hat - s_hat + c τ_hat
  //   rg = <b, y_hat> - <c, x_hat> - κ_hat
  info.rp = A * info.x_hat - b * info.tau_hat;
  info.rd = -A.transpose() * info.y_hat - info.s_hat + c * info.tau_hat;
  info.rg = b.dot(info.y_hat) - c.dot(info.x_hat) - info.kappa_hat;

  // α = <s_hat, x_hat> + τ_hat κ_hat
  info.alpha = info.s_hat.dot(info.x_hat) + info.tau_hat * info.kappa_hat;

  // Variable layout:
  //   [0, n)         : x
  //   [n, n+m)       : y
  //   [n+m, 2n+m)    : s
  //   2n+m           : τ
  //   2n+m+1         : κ
  //   2n+m+2         : θ
  const int N = info.total_vars();

  Model model;
  std::vector<int> all_vars(N);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // ================================================================
  // Equality 1:  Ax - bτ - rp θ = 0     (m rows)
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    for (int k = 0; k < A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
        trips.emplace_back(it.row(), info.x_start() + it.col(), it.value());
    for (int i = 0; i < m; ++i)
      trips.emplace_back(i, info.tau_idx(), -b(i));
    for (int i = 0; i < m; ++i)
      trips.emplace_back(i, info.theta_idx(), -info.rp(i));
    Eigen::SparseMatrix<double> C(m, N);
    C.setFromTriplets(trips.begin(), trips.end());
    model.AddEqualityConstraint(C, Eigen::VectorXd::Zero(m), all_vars);
  }

  // ================================================================
  // Equality 2:  -A*y - s + cτ - rd θ = 0     (n rows)
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    for (int k = 0; k < A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
        trips.emplace_back(it.col(), info.y_start() + it.row(), -it.value());
    for (int i = 0; i < n; ++i)
      trips.emplace_back(i, info.s_start() + i, -1.0);
    for (int i = 0; i < n; ++i)
      trips.emplace_back(i, info.tau_idx(), c(i));
    for (int i = 0; i < n; ++i)
      trips.emplace_back(i, info.theta_idx(), -info.rd(i));
    Eigen::SparseMatrix<double> C(n, N);
    C.setFromTriplets(trips.begin(), trips.end());
    model.AddEqualityConstraint(C, Eigen::VectorXd::Zero(n), all_vars);
  }

  // ================================================================
  // Equality 3:  <b,y> - <c,x> - κ - rg θ = 0     (1 row)
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < m; ++i)
      trips.emplace_back(0, info.y_start() + i, b(i));
    for (int i = 0; i < n; ++i)
      trips.emplace_back(0, info.x_start() + i, -c(i));
    trips.emplace_back(0, info.kappa_idx(), -1.0);
    trips.emplace_back(0, info.theta_idx(), -info.rg);
    Eigen::SparseMatrix<double> C(1, N);
    C.setFromTriplets(trips.begin(), trips.end());
    model.AddEqualityConstraint(C, Eigen::VectorXd::Zero(1), all_vars);
  }

  // ================================================================
  // Equality 4:  <rp,y> + <rd,x> + rg τ = -α     (1 row)
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < m; ++i)
      trips.emplace_back(0, info.y_start() + i, info.rp(i));
    for (int i = 0; i < n; ++i)
      trips.emplace_back(0, info.x_start() + i, info.rd(i));
    trips.emplace_back(0, info.tau_idx(), info.rg);
    Eigen::SparseMatrix<double> C(1, N);
    C.setFromTriplets(trips.begin(), trips.end());
    Eigen::VectorXd d(1);
    d << -info.alpha;
    model.AddEqualityConstraint(C, d, all_vars);
  }

  // ================================================================
  // Cone: x ≥ 0     (n rows)
  // ================================================================
  {
    Eigen::SparseMatrix<double> I(n, n);
    I.setIdentity();
    std::vector<int> x_vars(n);
    std::iota(x_vars.begin(), x_vars.end(), info.x_start());
    model.AddLinearConstraint(I, Eigen::VectorXd::Zero(n), x_vars);
  }

  // ================================================================
  // Cone: s ≥ 0     (n rows)
  // ================================================================
  {
    Eigen::SparseMatrix<double> I(n, n);
    I.setIdentity();
    std::vector<int> s_vars(n);
    std::iota(s_vars.begin(), s_vars.end(), info.s_start());
    model.AddLinearConstraint(I, Eigen::VectorXd::Zero(n), s_vars);
  }

  // ================================================================
  // Cone: τ ≥ 0, κ ≥ 0     (2 rows)
  // ================================================================
  {
    Eigen::SparseMatrix<double> I(2, 2);
    I.setIdentity();
    std::vector<int> tk_vars = {info.tau_idx(), info.kappa_idx()};
    model.AddLinearConstraint(I, Eigen::VectorXd::Zero(2), tk_vars);
  }

  // ================================================================
  // Cost: min α θ
  // ================================================================
  {
    Eigen::VectorXd cost = Eigen::VectorXd::Zero(N);
    cost(info.theta_idx()) = info.alpha;
    model.SetLinearCost(cost);
  }

  return {model, info};
}

}  // namespace conex
