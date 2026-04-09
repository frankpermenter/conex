#include "conex/common/problem.h"

#include <cmath>
#include <numeric>
#include <set>
#include <stdexcept>

#include "conex/common/structural_rank.h"

namespace conex {

std::pair<Problem, Expansion> Preprocess(const Problem& problem) {
  const int n = problem.num_variables();

  // Phase 1: Column reduction for linear and PSD constraints.
  std::vector<Eigen::Triplet<double>> trips;
  int total_rows = 0;
  for (const auto& c : problem.constraints()) {
    if (auto* lc = std::get_if<Problem::LinearConstraintData>(&c)) {
      for (int k = 0; k < lc->A.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(lc->A, k); it; ++it)
          trips.emplace_back(total_rows + it.row(), it.col(), it.value());
      total_rows += lc->A.rows();
    } else if (auto* pc = std::get_if<Problem::PSDConstraintData>(&c)) {
      // Mark each variable as live if its A_i has any nonzeros.
      for (int k = 0; k < static_cast<int>(pc->A_list.size()); ++k) {
        if (pc->A_list[k].nonZeros() > 0) {
          trips.emplace_back(total_rows, pc->vars[k], 1.0);
        }
      }
      total_rows += 1;
    }
  }

  Expansion expansion;
  expansion.original_n = n;
  expansion.col_map.resize(n);
  std::iota(expansion.col_map.begin(), expansion.col_map.end(), 0);

  int n_reduced = n;
  std::vector<int> inv(n);
  std::iota(inv.begin(), inv.end(), 0);

  if (!trips.empty()) {
    Eigen::SparseMatrix<double> A_combined(total_rows, n);
    A_combined.setFromTriplets(trips.begin(), trips.end());
    DropStructurallyDependentColumns(A_combined, &expansion.col_map);

    if (expansion.was_reduced()) {
      n_reduced = static_cast<int>(expansion.col_map.size());
      inv.assign(n, -1);
      for (int i = 0; i < n_reduced; ++i)
        inv[expansion.col_map[i]] = i;
    }
  }

  // Phase 2: Row reduction for equality constraints.
  // For each equality constraint, drop structurally dependent rows.
  // Check consistency of dropped rows.
  Problem reduced;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
        std::vector<Eigen::Triplet<double>> t;
        for (int k = 0; k < data.A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
               it; ++it) {
            int nc = inv[it.col()];
            if (nc >= 0) t.emplace_back(it.row(), nc, it.value());
          }
        Eigen::SparseMatrix<double> A_new(data.A.rows(), n_reduced);
        A_new.setFromTriplets(t.begin(), t.end());
        std::vector<int> new_vars;
        for (int v : data.vars) {
          int nv = inv[v];
          if (nv >= 0) new_vars.push_back(nv);
        }
        reduced.AddLinearConstraint(A_new, data.b, new_vars);

      } else if constexpr (std::is_same_v<T, Problem::PSDConstraintData>) {
        std::vector<Eigen::SparseMatrix<double>> new_A_list;
        std::vector<int> new_vars;
        for (int k = 0; k < static_cast<int>(data.A_list.size()); ++k) {
          int nv = inv[data.vars[k]];
          if (nv >= 0) {
            new_A_list.push_back(data.A_list[k]);
            new_vars.push_back(nv);
          }
        }
        reduced.AddPSDConstraint(new_A_list, data.B, new_vars);

      } else if constexpr (std::is_same_v<T, Problem::QuadraticCostData>) {
        std::vector<Eigen::Triplet<double>> t;
        for (int k = 0; k < data.Q_sparse.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.Q_sparse, k);
               it; ++it) {
            int nr = inv[it.row()], nc = inv[it.col()];
            if (nr >= 0 && nc >= 0) t.emplace_back(nr, nc, it.value());
          }
        Eigen::SparseMatrix<double> Q_new(n_reduced, n_reduced);
        Q_new.setFromTriplets(t.begin(), t.end());
        std::vector<int> new_vars;
        for (int v : data.vars) {
          int nv = inv[v];
          if (nv >= 0) new_vars.push_back(nv);
        }
        reduced.AddQuadraticCost(Q_new, new_vars);

      } else if constexpr (std::is_same_v<T,
                                          Problem::EqualityConstraintData>) {
        // Remap columns.
        std::vector<Eigen::Triplet<double>> t;
        for (int k = 0; k < data.C.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.C, k);
               it; ++it) {
            int nc = inv[it.col()];
            if (nc >= 0) t.emplace_back(it.row(), nc, it.value());
          }
        Eigen::SparseMatrix<double> C_remapped(data.C.rows(), n_reduced);
        C_remapped.setFromTriplets(t.begin(), t.end());

        // Drop structurally dependent rows.
        std::vector<int> row_map;
        Eigen::SparseMatrix<double> C_reduced =
            DropStructurallyDependentRows(C_remapped, &row_map);

        int p_orig = data.C.rows();
        int p_reduced = C_reduced.rows();

        if (p_reduced < p_orig) {
          // Check consistency of dropped rows.
          // Solve C_reduced * x = d_reduced for the independent rows,
          // then verify dropped rows are consistent.
          Eigen::VectorXd d_reduced(p_reduced);
          for (int r = 0; r < p_reduced; ++r)
            d_reduced(r) = data.d(row_map[r]);

          // Build the set of dropped row indices.
          std::set<int> kept(row_map.begin(), row_map.end());
          Eigen::MatrixXd C_dense(C_remapped);

          // For each dropped row, check if it's a linear combination of
          // kept rows with consistent RHS.
          // Simple check: solve the kept system for x, then verify
          // dropped rows.  Use dense QR for robustness.
          if (p_reduced > 0 && C_reduced.cols() > 0) {
            Eigen::MatrixXd Cr_dense(C_reduced);
            // Use least-squares to find x satisfying kept rows.
            Eigen::VectorXd x_check =
                Cr_dense.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                    .solve(d_reduced);

            for (int r = 0; r < p_orig; ++r) {
              if (kept.count(r)) continue;
              double lhs = C_dense.row(r).dot(x_check);
              double rhs_val = data.d(r);
              if (std::abs(lhs - rhs_val) > 1e-6 * (1 + std::abs(rhs_val))) {
                throw std::runtime_error(
                    "Inconsistent equality constraint: dropped row " +
                    std::to_string(r) + " has residual " +
                    std::to_string(std::abs(lhs - rhs_val)));
              }
            }
          }

          // Use reduced C and d.
          std::vector<int> new_primal;
          for (int v : data.primal_vars) {
            int nv = inv[v];
            if (nv >= 0) new_primal.push_back(nv);
          }
          reduced.AddEqualityConstraint(C_reduced, d_reduced, new_primal);
        } else {
          // No rows dropped.
          std::vector<int> new_primal;
          for (int v : data.primal_vars) {
            int nv = inv[v];
            if (nv >= 0) new_primal.push_back(nv);
          }
          reduced.AddEqualityConstraint(C_remapped, data.d, new_primal);
        }
      }
    }, problem.constraint(i));
  }

  // Reduce the linear cost if present.
  if (problem.has_linear_cost()) {
    reduced.SetLinearCost(expansion.Reduce(problem.linear_cost()));
  }

  return {reduced, expansion};
}

}  // namespace conex
