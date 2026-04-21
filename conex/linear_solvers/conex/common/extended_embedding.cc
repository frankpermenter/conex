#include "conex/common/extended_embedding.h"

#include <numeric>

namespace conex {

std::pair<Model, EmbeddingInfo> BuildExtendedEmbedding(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& c) {
  const int m = A.rows();
  const int n = A.cols();

  EmbeddingInfo info;
  info.n = n;
  info.m = m;

  // Default fixed point.
  info.x_hat = Eigen::VectorXd::Ones(n);
  info.s_hat = Eigen::VectorXd::Ones(m);
  info.y_hat = Eigen::VectorXd::Zero(m);
  info.tau_hat = 1.0;
  info.kappa_hat = 1.0;

  // The paper uses primal form: Ax - b*tau = 0, s = Ax - b*tau.
  // Our Model stores constraints as Ax + b >= 0, i.e., s = Ax + b.
  // So the paper's b corresponds to our -b: b_paper = -b_ours.
  // Residuals in paper form:
  //   rp = A*x_hat - b_paper*tau_hat = A*x_hat + b_ours*tau_hat
  //   rd = -A'*y_hat - s_hat + c*tau_hat
  //   rg = b_paper'*y_hat - c'*x_hat - kappa_hat
  //      = -b_ours'*y_hat - c'*x_hat - kappa_hat
  info.rp = A * info.x_hat + b * info.tau_hat;
  info.rd = -A.transpose() * info.y_hat - info.s_hat + c * info.tau_hat;
  info.rg = -b.dot(info.y_hat) - c.dot(info.x_hat) - info.kappa_hat;
  info.alpha = info.s_hat.dot(info.x_hat) + info.tau_hat * info.kappa_hat;

  const int N = info.total_vars();

  // Build the Model.
  // The embedding has:
  //   - Equality constraints (primal, dual, gap equations)
  //   - Cone constraints: s >= 0, tau >= 0, kappa >= 0
  //   - Cost: alpha * theta
  //   - Normalization: rp'y + rd'x + rg*tau = -alpha (equality)

  Model model;

  // ================================================================
  // Equality 1 (primal): Ax - b_paper*tau - rp*theta = 0
  //   i.e., Ax + b*tau - rp*theta = 0
  //   Variables: x (n), tau (1), theta (1)  →  m rows × N cols
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    // A*x
    for (int k = 0; k < A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
        trips.emplace_back(it.row(), info.x_start() + it.col(), it.value());
    // + b*tau  (paper: -b_paper*tau = +b_ours*tau)
    for (int i = 0; i < m; ++i)
      trips.emplace_back(i, info.tau_idx(), b(i));
    // - rp*theta
    for (int i = 0; i < m; ++i)
      trips.emplace_back(i, info.theta_idx(), -info.rp(i));

    Eigen::SparseMatrix<double> C_primal(m, N);
    C_primal.setFromTriplets(trips.begin(), trips.end());
    Eigen::VectorXd d_primal = Eigen::VectorXd::Zero(m);
    std::vector<int> all_vars(N);
    std::iota(all_vars.begin(), all_vars.end(), 0);
    model.AddEqualityConstraint(C_primal, d_primal, all_vars);
  }

  // ================================================================
  // Equality 2 (dual): -A'y - s + c*tau - rd*theta = 0
  //   Variables: y (m), s (m), tau (1), theta (1)  →  n rows × N cols
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    // -A'y
    for (int k = 0; k < A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
        trips.emplace_back(it.col(), info.y_start() + it.row(), -it.value());
    // -s
    for (int i = 0; i < m; ++i)
      trips.emplace_back(i, info.s_start() + i, -1.0);
    // + c*tau
    for (int i = 0; i < n; ++i)
      trips.emplace_back(i, info.tau_idx(), c(i));
    // - rd*theta
    for (int i = 0; i < n; ++i)
      trips.emplace_back(i, info.theta_idx(), -info.rd(i));

    Eigen::SparseMatrix<double> C_dual(n, N);
    C_dual.setFromTriplets(trips.begin(), trips.end());
    Eigen::VectorXd d_dual = Eigen::VectorXd::Zero(n);
    std::vector<int> all_vars(N);
    std::iota(all_vars.begin(), all_vars.end(), 0);
    model.AddEqualityConstraint(C_dual, d_dual, all_vars);
  }

  // ================================================================
  // Equality 3 (gap): b_paper'y - c'x - kappa - rg*theta = 0
  //   i.e., -b_ours'y - c'x - kappa - rg*theta = 0
  //   1 row × N cols
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    // -b'y  (paper: b_paper = -b_ours)
    for (int i = 0; i < m; ++i)
      trips.emplace_back(0, info.y_start() + i, -b(i));
    // -c'x
    for (int i = 0; i < n; ++i)
      trips.emplace_back(0, info.x_start() + i, -c(i));
    // -kappa
    trips.emplace_back(0, info.kappa_idx(), -1.0);
    // -rg*theta
    trips.emplace_back(0, info.theta_idx(), -info.rg);

    Eigen::SparseMatrix<double> C_gap(1, N);
    C_gap.setFromTriplets(trips.begin(), trips.end());
    Eigen::VectorXd d_gap = Eigen::VectorXd::Zero(1);
    std::vector<int> all_vars(N);
    std::iota(all_vars.begin(), all_vars.end(), 0);
    model.AddEqualityConstraint(C_gap, d_gap, all_vars);
  }

  // ================================================================
  // Equality 4 (normalization): rp'y + rd'x + rg*tau = -alpha
  //   1 row × N cols
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    // rp'y
    for (int i = 0; i < m; ++i)
      trips.emplace_back(0, info.y_start() + i, info.rp(i));
    // rd'x
    for (int i = 0; i < n; ++i)
      trips.emplace_back(0, info.x_start() + i, info.rd(i));
    // rg*tau
    trips.emplace_back(0, info.tau_idx(), info.rg);

    Eigen::SparseMatrix<double> C_norm(1, N);
    C_norm.setFromTriplets(trips.begin(), trips.end());
    Eigen::VectorXd d_norm(1);
    d_norm << -info.alpha;
    std::vector<int> all_vars(N);
    std::iota(all_vars.begin(), all_vars.end(), 0);
    model.AddEqualityConstraint(C_norm, d_norm, all_vars);
  }

  // ================================================================
  // Cone constraints: s >= 0  (m rows)
  // s variables are at indices [s_start, s_start + m).
  // Constraint: I * s + 0 >= 0.
  // ================================================================
  {
    Eigen::SparseMatrix<double> I_s(m, m);
    I_s.setIdentity();
    Eigen::VectorXd b_s = Eigen::VectorXd::Zero(m);
    std::vector<int> s_vars(m);
    std::iota(s_vars.begin(), s_vars.end(), info.s_start());
    model.AddLinearConstraint(I_s, b_s, s_vars);
  }

  // ================================================================
  // Cone constraints: tau >= 0, kappa >= 0  (2 rows)
  // ================================================================
  {
    Eigen::SparseMatrix<double> I_tk(2, 2);
    I_tk.setIdentity();
    Eigen::VectorXd b_tk = Eigen::VectorXd::Zero(2);
    std::vector<int> tk_vars = {info.tau_idx(), info.kappa_idx()};
    model.AddLinearConstraint(I_tk, b_tk, tk_vars);
  }

  // ================================================================
  // Cost: alpha * theta
  // ================================================================
  {
    Eigen::VectorXd cost = Eigen::VectorXd::Zero(N);
    cost(info.theta_idx()) = info.alpha;
    model.SetLinearCost(cost);
  }

  return {model, info};
}

}  // namespace conex
