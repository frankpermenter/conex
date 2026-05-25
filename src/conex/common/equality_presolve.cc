#include "conex/common/equality_presolve.h"
#include <Eigen/QR>

namespace conex {

EqualityPresolveResult EliminateEqualities(const Model& problem) {
  EqualityPresolveResult result;
  result.original_n = problem.num_variables();
  int n = result.original_n;

  // Collect all equality constraints into one dense C, d.
  int total_eq_rows = 0;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    if (auto* eq = std::get_if<Model::EqualityConstraintData>(
            &problem.constraint(i))) {
      total_eq_rows += eq->C.rows();
    }
  }

  if (total_eq_rows == 0) {
    // No equalities — return the original problem unchanged.
    result.reduced = problem;
    result.N = Eigen::MatrixXd::Identity(n, n);
    result.x0 = Eigen::VectorXd::Zero(n);
    return result;
  }

  Eigen::MatrixXd C(total_eq_rows, n);
  Eigen::VectorXd d(total_eq_rows);
  C.setZero();
  d.setZero();
  int row = 0;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    if (auto* eq = std::get_if<Model::EqualityConstraintData>(
            &problem.constraint(i))) {
      int p = eq->C.rows();
      for (int k = 0; k < eq->C.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(eq->C, k);
             it; ++it) {
          C(row + it.row(), eq->primal_vars[it.col()]) = it.value();
        }
      }
      d.segment(row, p) = eq->d;
      row += p;
    }
  }

  // QR factorization of C' to get null space.
  // C is p×n. C' is n×p. QR of C': C' = Q*R where Q is n×n, R is n×p.
  // Null space of C = last (n-p) columns of Q.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(C.transpose());
  int rank = qr.rank();
  int nz = n - rank;  // null space dimension

  Eigen::MatrixXd Q = qr.householderQ() *
      Eigen::MatrixXd::Identity(n, n);
  // Null space basis: last nz columns of Q.
  result.N = Q.rightCols(nz);

  // Particular solution: x0 = C^+ * d (minimum-norm via QR of C').
  // C' = Q * R * P' where Q is n×n, R is n×p, P is permutation.
  // x0 = Q * R^{-T} * P' * d (using only the first `rank` rows of R).
  Eigen::VectorXd Pd = qr.colsPermutation().transpose() * d;
  Eigen::MatrixXd R = qr.matrixQR().topLeftCorner(rank, rank)
                          .triangularView<Eigen::Upper>();
  Eigen::VectorXd y = R.transpose().triangularView<Eigen::Lower>()
                          .solve(Pd.head(rank));
  Eigen::VectorXd Qy = Eigen::VectorXd::Zero(n);
  Qy.head(rank) = y;
  result.x0 = qr.householderQ() * Qy;

  // Build reduced problem in z (nz variables).
  // x = x0 + N*z
  // Objective: 0.5*z'*(N'QN)*z + (N'(Qx0+c))'z + const
  // Inequalities: (A*N)*z + (A*x0 + b) >= 0
  result.reduced = Model();
  std::vector<int> z_vars(nz);
  std::iota(z_vars.begin(), z_vars.end(), 0);

  // Transform linear constraints.
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        // A_orig is m × |vars|. Expand to m × n.
        int m = data.A.rows();
        Eigen::MatrixXd A_full(m, n);
        A_full.setZero();
        for (int k = 0; k < data.A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
               it; ++it)
            A_full(it.row(), data.vars[it.col()]) = it.value();

        // A_new = A_full * N (m × nz)
        Eigen::MatrixXd A_new = A_full * result.N;
        // b_new = A_full * x0 + b
        Eigen::VectorXd b_new = A_full * result.x0 + data.b;

        Eigen::SparseMatrix<double> A_sp = A_new.sparseView();
        result.reduced.AddLinearConstraint(A_sp, b_new, z_vars);

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        // Q_orig is |vars| × |vars|. Expand to n × n.
        Eigen::MatrixXd Q_full(n, n);
        Q_full.setZero();
        for (int k = 0; k < data.Q_sparse.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.Q_sparse, k);
               it; ++it)
            Q_full(data.vars[it.row()], data.vars[it.col()]) = it.value();

        // Q_new = N' * Q_full * N (nz × nz)
        Eigen::MatrixXd Q_new = result.N.transpose() * Q_full * result.N;
        Eigen::SparseMatrix<double> Q_sp = Q_new.sparseView();
        result.reduced.AddQuadraticCost(Q_sp, z_vars);

        // Linear cost contribution from Q*x0: N'*Q*x0.
        // Added to linear cost below.

      } else if constexpr (std::is_same_v<T,
                                          Model::EqualityConstraintData>) {
        // Skip — these are being eliminated.
      }
    }, problem.constraint(i));
  }

  // Linear cost: c_new = N' * (c + Q*x0).
  Eigen::VectorXd c_full = Eigen::VectorXd::Zero(n);
  if (problem.has_linear_cost()) {
    c_full = problem.linear_cost();
  }
  // Add Q*x0 contribution.
  for (int i = 0; i < problem.num_constraints(); ++i) {
    if (auto* qc = std::get_if<Model::QuadraticCostData>(
            &problem.constraint(i))) {
      Eigen::MatrixXd Q_full(n, n);
      Q_full.setZero();
      for (int k = 0; k < qc->Q_sparse.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(qc->Q_sparse, k);
             it; ++it)
          Q_full(qc->vars[it.row()], qc->vars[it.col()]) = it.value();
      c_full += Q_full * result.x0;
    }
  }
  Eigen::VectorXd c_new = result.N.transpose() * c_full;
  result.reduced.SetLinearCost(c_new);

  return result;
}

}  // namespace conex
