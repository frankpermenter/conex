#include "conex/common/extended_embedding.h"

#include <numeric>

namespace conex {

// Full implementation with optional dual equalities Cy = d.
ExtendedEmbedding BuildExtendedEmbedding(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double>& C,
    const Eigen::VectorXd& d) {
  const int m = A.rows();
  const int n = A.cols();
  const int p = C.rows();  // 0 if no dual equalities

  if (m + p > n) {
    fprintf(stderr, "BuildExtendedEmbedding: requires m+p <= n (got m=%d, p=%d, n=%d).\n"
            "The embedding has n slack variables s but m+p free variables (y,w).\n"
            "When m+p > n the KKT matrix is structurally singular.\n", m, p, n);
    return {Model(), EmbeddingInfo(), CliqueTree()};
  }

  EmbeddingInfo info;
  info.n = n;
  info.m = m;
  info.p = p;

  // Fixed point: x_hat=e, s_hat=e, w_hat=0, y_hat=0, tau_hat=1, kappa_hat=1.
  info.x_hat = Eigen::VectorXd::Ones(n);
  info.s_hat = Eigen::VectorXd::Ones(n);
  info.y_hat = Eigen::VectorXd::Zero(m);
  info.w_hat = Eigen::VectorXd::Zero(p);
  info.tau_hat = 1.0;
  info.kappa_hat = 1.0;

  // Residuals at the fixed point:
  //   rp = A*x_hat + C'*w_hat - b*tau_hat = Ae - b
  //   rd = -A'*y_hat - s_hat + c*tau_hat = c - e
  //   re = -C*y_hat + d*tau_hat = d
  //   rg = b'*y_hat - c'*x_hat - d'*w_hat - kappa_hat = -c'e - 1
  info.rp = A * info.x_hat - b * info.tau_hat;
  info.rd = -A.transpose() * info.y_hat - info.s_hat + c * info.tau_hat;
  info.re = (p > 0) ? Eigen::VectorXd(d * info.tau_hat)
                     : Eigen::VectorXd();
  info.rg = b.dot(info.y_hat) - c.dot(info.x_hat) - info.kappa_hat;

  // alpha = <s_hat, x_hat> + tau_hat * kappa_hat
  info.alpha = info.s_hat.dot(info.x_hat) + info.tau_hat * info.kappa_hat;

  const int N = info.total_vars();

  Model model;
  std::vector<int> all_vars(N);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // ================================================================
  // E1:  Ax + C'w - b*tau - rp*theta = 0     (m rows)
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    // A*x
    for (int k = 0; k < A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
        trips.emplace_back(it.row(), info.x_start() + it.col(), it.value());
    // C'*w (C is p×m, C' is m×p)
    for (int k = 0; k < C.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(C, k); it; ++it)
        trips.emplace_back(it.col(), info.w_start() + it.row(), it.value());
    // -b*tau
    for (int i = 0; i < m; ++i)
      trips.emplace_back(i, info.tau_idx(), -b(i));
    // -rp*theta
    for (int i = 0; i < m; ++i)
      trips.emplace_back(i, info.theta_idx(), -info.rp(i));
    Eigen::SparseMatrix<double> Eq(m, N);
    Eq.setFromTriplets(trips.begin(), trips.end());
    model.AddEqualityConstraint(Eq, Eigen::VectorXd::Zero(m), all_vars);
  }

  // ================================================================
  // E2:  -A'y - s + c*tau - rd*theta = 0     (n rows)
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    // -A'*y
    for (int k = 0; k < A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
        trips.emplace_back(it.col(), info.y_start() + it.row(), -it.value());
    // -s
    for (int i = 0; i < n; ++i)
      trips.emplace_back(i, info.s_start() + i, -1.0);
    // c*tau
    for (int i = 0; i < n; ++i)
      trips.emplace_back(i, info.tau_idx(), c(i));
    // -rd*theta
    for (int i = 0; i < n; ++i)
      trips.emplace_back(i, info.theta_idx(), -info.rd(i));
    Eigen::SparseMatrix<double> Eq(n, N);
    Eq.setFromTriplets(trips.begin(), trips.end());
    model.AddEqualityConstraint(Eq, Eigen::VectorXd::Zero(n), all_vars);
  }

  // ================================================================
  // E3:  -Cy + d*tau - re*theta = 0     (p rows, skipped if p=0)
  // ================================================================
  if (p > 0) {
    std::vector<Eigen::Triplet<double>> trips;
    // -C*y
    for (int k = 0; k < C.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(C, k); it; ++it)
        trips.emplace_back(it.row(), info.y_start() + it.col(), -it.value());
    // d*tau
    for (int i = 0; i < p; ++i)
      trips.emplace_back(i, info.tau_idx(), d(i));
    // -re*theta
    for (int i = 0; i < p; ++i)
      trips.emplace_back(i, info.theta_idx(), -info.re(i));
    Eigen::SparseMatrix<double> Eq(p, N);
    Eq.setFromTriplets(trips.begin(), trips.end());
    model.AddEqualityConstraint(Eq, Eigen::VectorXd::Zero(p), all_vars);
  }

  // ================================================================
  // E4:  b'y - c'x - d'w - kappa - rg*theta = 0     (1 row)
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < m; ++i)
      trips.emplace_back(0, info.y_start() + i, b(i));
    for (int i = 0; i < n; ++i)
      trips.emplace_back(0, info.x_start() + i, -c(i));
    for (int i = 0; i < p; ++i)
      trips.emplace_back(0, info.w_start() + i, -d(i));
    trips.emplace_back(0, info.kappa_idx(), -1.0);
    trips.emplace_back(0, info.theta_idx(), -info.rg);
    Eigen::SparseMatrix<double> Eq(1, N);
    Eq.setFromTriplets(trips.begin(), trips.end());
    model.AddEqualityConstraint(Eq, Eigen::VectorXd::Zero(1), all_vars);
  }

  // ================================================================
  // E5:  rp'y + rd'x + re'w + rg*tau = -alpha     (1 row)
  // ================================================================
  {
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < m; ++i)
      trips.emplace_back(0, info.y_start() + i, info.rp(i));
    for (int i = 0; i < n; ++i)
      trips.emplace_back(0, info.x_start() + i, info.rd(i));
    for (int i = 0; i < p; ++i)
      trips.emplace_back(0, info.w_start() + i, info.re(i));
    trips.emplace_back(0, info.tau_idx(), info.rg);
    Eigen::SparseMatrix<double> Eq(1, N);
    Eq.setFromTriplets(trips.begin(), trips.end());
    Eigen::VectorXd rhs(1);
    rhs << -info.alpha;
    model.AddEqualityConstraint(Eq, rhs, all_vars);
  }

  // ================================================================
  // Cone: x >= 0     (n rows)
  // ================================================================
  {
    Eigen::SparseMatrix<double> I(n, n);
    I.setIdentity();
    std::vector<int> x_vars(n);
    std::iota(x_vars.begin(), x_vars.end(), info.x_start());
    model.AddLinearConstraint(I, Eigen::VectorXd::Zero(n), x_vars);
  }

  // ================================================================
  // Cone: s >= 0     (n rows)
  // ================================================================
  {
    Eigen::SparseMatrix<double> I(n, n);
    I.setIdentity();
    std::vector<int> s_vars(n);
    std::iota(s_vars.begin(), s_vars.end(), info.s_start());
    model.AddLinearConstraint(I, Eigen::VectorXd::Zero(n), s_vars);
  }

  // ================================================================
  // Cone: tau >= 0, kappa >= 0     (2 rows)
  // ================================================================
  {
    Eigen::SparseMatrix<double> I(2, 2);
    I.setIdentity();
    std::vector<int> tk_vars = {info.tau_idx(), info.kappa_idx()};
    model.AddLinearConstraint(I, Eigen::VectorXd::Zero(2), tk_vars);
  }

  // ================================================================
  // Cost: min alpha * theta
  // ================================================================
  {
    Eigen::VectorXd cost = Eigen::VectorXd::Zero(N);
    cost(info.theta_idx()) = info.alpha;
    model.SetLinearCost(cost);
  }

  // ================================================================
  // CliqueTree for dense A.
  //
  // Leaf: {x, ν₁} with separator {τ, θ, ν_gap, ν_norm}.
  // Root: everything else (y, w, s, κ, and remaining duals).
  //
  // Dual variable allocation (by BuildInternal, in constraint order):
  //   ν₁[N, N+m)             from E1
  //   ν₂[N+m, N+m+n)         from E2
  //   ν₃[N+m+n, N+m+n+p)     from E3 (if p>0)
  //   ν_gap = N+m+n+p        from E4
  //   ν_norm = N+m+n+p+1     from E5
  // ================================================================
  CliqueTree tree;
  {
    const int nu1_start = N;
    const int nu2_start = N + m;
    const int nu3_start = N + m + n;  // E3 duals (p of them)
    const int nu_gap = N + m + n + p;
    const int nu_norm = N + m + n + p + 1;

    // Separator: {tau, theta, nu_gap, nu_norm}.
    std::vector<int> sep = {info.tau_idx(), info.theta_idx(), nu_gap, nu_norm};

    // Leaf: {x, nu1}.
    std::vector<int> leaf_sn;
    for (int j = 0; j < n; j++) leaf_sn.push_back(info.x_start() + j);
    for (int i = 0; i < m; i++) leaf_sn.push_back(nu1_start + i);

    // Root: separator + {y, w, s, kappa, nu2, nu3}.
    std::vector<int> root_sn = sep;
    for (int i = 0; i < m; i++) root_sn.push_back(info.y_start() + i);
    for (int i = 0; i < p; i++) root_sn.push_back(info.w_start() + i);
    for (int j = 0; j < n; j++) root_sn.push_back(info.s_start() + j);
    root_sn.push_back(info.kappa_idx());
    for (int j = 0; j < n; j++) root_sn.push_back(nu2_start + j);
    for (int i = 0; i < p; i++) root_sn.push_back(nu3_start + i);

    tree.supernodes = {leaf_sn, root_sn};
    tree.separators = {sep, {}};
    tree.node_to_parent = {1, -1};
    tree.post_order_position_to_clique = {0, 1};
  }

  return {std::move(model), info, std::move(tree)};
}

// No dual equalities: delegate with empty C, d.
ExtendedEmbedding BuildExtendedEmbedding(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& c) {
  Eigen::SparseMatrix<double> C(0, A.rows());  // 0 rows, m cols
  Eigen::VectorXd d;
  return BuildExtendedEmbedding(A, b, c, C, d);
}

// Extract from Model.
ExtendedEmbedding BuildExtendedEmbedding(const Model& model) {
  const int nc = model.num_constraints();
  const int n = model.num_variables();

  Eigen::SparseMatrix<double> A_eq;
  Eigen::VectorXd b_eq;
  Eigen::SparseMatrix<double> C_dual;
  Eigen::VectorXd d_dual;
  int num_eq = 0;

  for (int i = 0; i < nc; i++) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        // Expand to full n cols.
        const auto& pv = data.primal_vars;
        Eigen::SparseMatrix<double> Cfull;
        if (static_cast<int>(pv.size()) == n) {
          Cfull = data.C;
        } else {
          std::vector<Eigen::Triplet<double>> trips;
          for (int k = 0; k < data.C.outerSize(); k++)
            for (Eigen::SparseMatrix<double>::InnerIterator it(data.C, k); it;
                 ++it)
              trips.emplace_back(it.row(), pv[it.col()], it.value());
          Cfull.resize(data.C.rows(), n);
          Cfull.setFromTriplets(trips.begin(), trips.end());
        }

        if (num_eq == 0) {
          // First equality → primal constraint Ax = b.
          A_eq = Cfull;
          b_eq = data.d;
        } else if (num_eq == 1) {
          // Second equality → dual constraint Cy = d.
          C_dual = Cfull;
          d_dual = data.d;
        } else {
          CONEX_DEMAND(false,
                       "BuildExtendedEmbedding(Model): at most 2 equality "
                       "constraints supported.");
        }
        num_eq++;
      }
    }, model.constraint(i));
  }
  CONEX_DEMAND(num_eq >= 1,
               "BuildExtendedEmbedding(Model): no equality constraint found.");
  CONEX_DEMAND(model.has_linear_cost(),
               "BuildExtendedEmbedding(Model): no linear cost found.");

  if (num_eq == 1) {
    return BuildExtendedEmbedding(A_eq, b_eq, model.linear_cost());
  } else {
    return BuildExtendedEmbedding(A_eq, b_eq, model.linear_cost(),
                                  C_dual, d_dual);
  }
}

}  // namespace conex
