#include "conex/common/problem.h"

#include <numeric>
#include <set>

#include "conex/common/structural_rank.h"

namespace conex {

std::pair<Problem, Expansion> Preprocess(const Problem& problem) {
  const int n = problem.num_variables();

  // Collect all linear constraint matrices to check structural rank.
  std::vector<Eigen::Triplet<double>> trips;
  int total_rows = 0;
  for (const auto& c : problem.constraints()) {
    if (auto* lc = std::get_if<Problem::LinearConstraintData>(&c)) {
      for (int k = 0; k < lc->A.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(lc->A, k); it; ++it)
          trips.emplace_back(total_rows + it.row(), it.col(), it.value());
      total_rows += lc->A.rows();
    }
  }

  Expansion expansion;
  expansion.original_n = n;

  if (trips.empty()) {
    // No linear constraints — nothing to reduce.
    expansion.col_map.resize(n);
    std::iota(expansion.col_map.begin(), expansion.col_map.end(), 0);
    return {problem, expansion};
  }

  Eigen::SparseMatrix<double> A_combined(total_rows, n);
  A_combined.setFromTriplets(trips.begin(), trips.end());

  DropStructurallyDependentColumns(A_combined, &expansion.col_map);

  if (!expansion.was_reduced()) {
    // Full rank — return original problem.
    return {problem, expansion};
  }

  // Build inverse column map.
  const int n_reduced = static_cast<int>(expansion.col_map.size());
  std::vector<int> inv(n, -1);
  for (int i = 0; i < n_reduced; ++i)
    inv[expansion.col_map[i]] = i;

  // Build reduced problem: remap all variable indices.
  Problem reduced;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
        // Remap columns of A.
        std::vector<Eigen::Triplet<double>> t;
        for (int k = 0; k < data.A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
               it; ++it) {
            int nc = inv[it.col()];
            if (nc >= 0) t.emplace_back(it.row(), nc, it.value());
          }
        Eigen::SparseMatrix<double> A_new(data.A.rows(), n_reduced);
        A_new.setFromTriplets(t.begin(), t.end());
        // Remap vars.
        std::vector<int> new_vars;
        for (int v : data.vars) {
          int nv = inv[v];
          if (nv >= 0) new_vars.push_back(nv);
        }
        reduced.AddLinearConstraint(A_new, data.b, new_vars);

      } else if constexpr (std::is_same_v<T, Problem::QuadraticCostData>) {
        // Remap Q columns and rows.
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
        // Remap C columns.
        std::vector<Eigen::Triplet<double>> t;
        for (int k = 0; k < data.C.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.C, k);
               it; ++it) {
            int nc = inv[it.col()];
            if (nc >= 0) t.emplace_back(it.row(), nc, it.value());
          }
        Eigen::SparseMatrix<double> C_new(data.C.rows(), n_reduced);
        C_new.setFromTriplets(t.begin(), t.end());
        std::vector<int> new_primal, new_dual;
        for (int v : data.primal_vars) {
          int nv = inv[v];
          if (nv >= 0) new_primal.push_back(nv);
        }
        // Dual vars are allocated by the solver, not remapped here.
        reduced.AddEqualityConstraint(C_new, data.d, new_primal, data.dual_vars);
      }
    }, problem.constraint(i));
  }

  return {reduced, expansion};
}

}  // namespace conex
