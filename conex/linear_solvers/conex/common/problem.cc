#include "conex/common/problem.h"

#include <cmath>
#include <numeric>
#include <set>
#include <stdexcept>

#include "conex/common/structural_rank.h"

namespace conex {

std::pair<Problem, Expansion> RemoveStructuralRankDeficiency(
    const Problem& problem) {
  const int n = problem.num_variables();

  // Phase 1: Column reduction for linear and PSD constraints.
  std::vector<Eigen::Triplet<double>> trips;
  int total_rows = 0;
  for (const auto& c : problem.constraints()) {
    // Linear and SOC have the same A matrix structure.
    const Eigen::SparseMatrix<double>* A_ptr = nullptr;
    const std::vector<int>* vars_ptr = nullptr;
    if (auto* lc = std::get_if<Problem::LinearConstraintData>(&c)) {
      A_ptr = &lc->A;
      vars_ptr = &lc->vars;
    } else if (auto* sc = std::get_if<Problem::SOCConstraintData>(&c)) {
      A_ptr = &sc->A;
      vars_ptr = &sc->vars;
    }
    if (A_ptr) {
      for (int k = 0; k < A_ptr->outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(*A_ptr, k); it; ++it)
          trips.emplace_back(total_rows + it.row(),
                             (*vars_ptr)[it.col()], it.value());
      total_rows += A_ptr->rows();
    } else if (auto* qc = std::get_if<Problem::QuadraticCostData>(&c)) {
      // Quadratic cost variables are structurally present (they contribute
      // to the Gram matrix diagonal).  Add one row per variable to prevent
      // them from being dropped as structurally rank-deficient.
      for (int v : qc->vars) {
        trips.emplace_back(total_rows, v, 1.0);
        total_rows++;
      }
    } else if (auto* ec = std::get_if<Problem::EqualityConstraintData>(&c)) {
      // Equality constraint primal variables are structurally present.
      for (int k = 0; k < ec->C.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(ec->C, k); it; ++it)
          trips.emplace_back(total_rows + it.row(),
                             ec->primal_vars[it.col()], it.value());
      total_rows += ec->C.rows();
    } else if (auto* pc = std::get_if<Problem::PSDConstraintData>(&c)) {
      // Add one row per unique nonzero entry position across all A_i.
      // This reflects the true structural rank: each entry (r,c) of the
      // n×n PSD constraint is an independent scalar constraint.
      std::set<std::pair<int,int>> entry_positions;
      for (int k = 0; k < static_cast<int>(pc->A_list.size()); ++k) {
        const auto& Ak = pc->A_list[k];
        for (int col = 0; col < Ak.outerSize(); ++col)
          for (Eigen::SparseMatrix<double>::InnerIterator it(Ak, col); it; ++it) {
            int r = static_cast<int>(it.row());
            int ci = static_cast<int>(it.col());
            entry_positions.insert({std::min(r, ci), std::max(r, ci)});
          }
      }
      std::map<std::pair<int,int>, int> entry_to_row;
      for (const auto& pos : entry_positions)
        entry_to_row[pos] = total_rows++;

      for (int k = 0; k < static_cast<int>(pc->A_list.size()); ++k) {
        const auto& Ak = pc->A_list[k];
        for (int col = 0; col < Ak.outerSize(); ++col)
          for (Eigen::SparseMatrix<double>::InnerIterator it(Ak, col); it; ++it) {
            int r2 = static_cast<int>(it.row());
            int c2 = static_cast<int>(it.col());
            auto pos = std::make_pair(std::min(r2, c2), std::max(r2, c2));
            trips.emplace_back(entry_to_row[pos], pc->vars[k], 1.0);
          }
      }
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

      if constexpr (std::is_same_v<T, Problem::LinearConstraintData> ||
                     std::is_same_v<T, Problem::SOCConstraintData>) {
        // Build new_vars and a local column map: original local col ->
        // new local col.  Dropped vars are skipped entirely.
        std::vector<int> new_vars;
        std::vector<int> local_col_map(data.A.cols(), -1);
        for (int j = 0; j < static_cast<int>(data.vars.size()); ++j) {
          int nv = inv[data.vars[j]];
          if (nv >= 0) {
            local_col_map[j] = static_cast<int>(new_vars.size());
            new_vars.push_back(nv);
          }
        }
        std::vector<Eigen::Triplet<double>> t;
        for (int k = 0; k < data.A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
               it; ++it) {
            int nc = local_col_map[it.col()];
            if (nc >= 0) t.emplace_back(it.row(), nc, it.value());
          }
        int new_ncols = static_cast<int>(new_vars.size());
        Eigen::SparseMatrix<double> A_new(data.A.rows(), new_ncols);
        A_new.setFromTriplets(t.begin(), t.end());
        if constexpr (std::is_same_v<T, Problem::SOCConstraintData>)
          reduced.AddSOCConstraint(A_new, data.b, new_vars);
        else
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
        reduced.AddPSDConstraint(new_A_list, data.B, new_vars,
                                 data.use_chordal);

      } else if constexpr (std::is_same_v<T, Problem::QuadraticCostData>) {
        std::vector<int> new_vars;
        std::vector<int> local_col_map(data.Q_sparse.cols(), -1);
        for (int j = 0; j < static_cast<int>(data.vars.size()); ++j) {
          int nv = inv[data.vars[j]];
          if (nv >= 0) {
            local_col_map[j] = static_cast<int>(new_vars.size());
            new_vars.push_back(nv);
          }
        }
        int new_nv = static_cast<int>(new_vars.size());
        std::vector<Eigen::Triplet<double>> t;
        for (int k = 0; k < data.Q_sparse.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.Q_sparse, k);
               it; ++it) {
            int nr = local_col_map[it.row()];
            int nc = local_col_map[it.col()];
            if (nr >= 0 && nc >= 0) t.emplace_back(nr, nc, it.value());
          }
        Eigen::SparseMatrix<double> Q_new(new_nv, new_nv);
        Q_new.setFromTriplets(t.begin(), t.end());
        reduced.AddQuadraticCost(Q_new, new_vars);

      } else if constexpr (std::is_same_v<T,
                                          Problem::EqualityConstraintData>) {
        // Remap columns using local col map (primal_vars may be a subset).
        std::vector<int> new_primal;
        std::vector<int> local_col_map(data.C.cols(), -1);
        for (int j = 0; j < static_cast<int>(data.primal_vars.size()); ++j) {
          int nv = inv[data.primal_vars[j]];
          if (nv >= 0) {
            local_col_map[j] = static_cast<int>(new_primal.size());
            new_primal.push_back(nv);
          }
        }
        int new_ncols = static_cast<int>(new_primal.size());
        std::vector<Eigen::Triplet<double>> t;
        for (int k = 0; k < data.C.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.C, k);
               it; ++it) {
            int nc = local_col_map[it.col()];
            if (nc >= 0) t.emplace_back(it.row(), nc, it.value());
          }
        Eigen::SparseMatrix<double> C_remapped(data.C.rows(), new_ncols);
        C_remapped.setFromTriplets(t.begin(), t.end());

        // Drop structurally dependent rows.
        std::vector<int> row_map;
        Eigen::SparseMatrix<double> C_reduced =
            DropStructurallyDependentRows(C_remapped, &row_map);

        int p_orig = data.C.rows();
        int p_reduced = C_reduced.rows();

        if (p_reduced < p_orig) {
          // Check consistency of dropped rows.
          Eigen::VectorXd d_reduced(p_reduced);
          for (int r = 0; r < p_reduced; ++r)
            d_reduced(r) = data.d(row_map[r]);

          std::set<int> kept(row_map.begin(), row_map.end());
          Eigen::MatrixXd C_dense(C_remapped);

          if (p_reduced > 0 && C_reduced.cols() > 0) {
            Eigen::MatrixXd Cr_dense(C_reduced);
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

          reduced.AddEqualityConstraint(C_reduced, d_reduced, new_primal);
        } else {
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
