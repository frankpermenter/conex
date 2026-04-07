#include "conex/common/sparse_equality_constraint.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <set>

#include "conex/common/error_checking_macros.h"

namespace conex {

namespace {

bool IsSubset(const std::vector<int>& a, const std::vector<int>& b) {
  return std::includes(b.begin(), b.end(), a.begin(), a.end());
}

}  // namespace

SparseEqualityConstraint::SparseEqualityConstraint(
    const Eigen::SparseMatrix<double>& C, const Eigen::VectorXd& d)
    : C_(C), d_(d) {
  CONEX_DEMAND(C.rows() == d.rows(),
               "C and d must have the same number of rows.");

  // Build row supports (column-major iteration gives sorted supports).
  std::vector<std::vector<int>> row_supports(C.rows());
  for (int k = 0; k < C.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(C, k); it; ++it) {
      row_supports[it.row()].push_back(it.col());
    }
  }

  // Group rows by identical support.
  std::vector<int> row_order(C.rows());
  std::iota(row_order.begin(), row_order.end(), 0);
  std::sort(row_order.begin(), row_order.end(), [&](int a, int b) {
    return row_supports[a] < row_supports[b];
  });

  for (int row : row_order) {
    if (row_supports[row].empty()) continue;
    if (support_groups_.empty() ||
        support_groups_.back().support != row_supports[row]) {
      support_groups_.push_back({row_supports[row], {}});
    }
    support_groups_.back().rows.push_back(row);
  }

  unique_supports_.reserve(support_groups_.size());
  for (const auto& sg : support_groups_) {
    unique_supports_.push_back(sg.support);
  }
}

std::vector<SparseEqualityConstraint::RowGroup>
SparseEqualityConstraint::GetConstraints(
    const std::vector<std::vector<int>>& target_supports,
    const std::vector<int>& row_to_dual) const {
  // Assign each ROW (not support group) to the smallest target containing
  // both its primal support AND its dual variable.  Rows with the same
  // support may end up in different targets if their duals differ.
  std::vector<std::vector<int>> rows_per_target(target_supports.size());

  // Precompute sets for fast membership testing.
  std::vector<std::set<int>> target_sets(target_supports.size());
  for (size_t ti = 0; ti < target_supports.size(); ++ti) {
    target_sets[ti].insert(target_supports[ti].begin(),
                           target_supports[ti].end());
  }

  for (const auto& sg : support_groups_) {
    const auto& support = sg.support;
    for (int row : sg.rows) {
      int dual_var = row_to_dual[row];
      int best = -1;
      size_t best_size = std::numeric_limits<size_t>::max();
      for (size_t ti = 0; ti < target_supports.size(); ++ti) {
        if (target_supports[ti].size() >= best_size) continue;
        if (!target_sets[ti].count(dual_var)) continue;
        if (!IsSubset(support, target_supports[ti])) continue;
        best = static_cast<int>(ti);
        best_size = target_supports[ti].size();
      }
      CONEX_DEMAND(best >= 0,
                   "No target contains both primal support and dual var.");
      rows_per_target[best].push_back(row);
    }
  }

  // Build dense sub-blocks with dual variables.
  // Target supports may contain both primal and dual variable indices;
  // only primal indices (< C.cols()) are used as columns of the dense block.
  const int num_cols = C_.cols();
  std::vector<RowGroup> result;
  for (size_t ti = 0; ti < target_supports.size(); ++ti) {
    if (rows_per_target[ti].empty()) continue;
    const auto& full_vars = target_supports[ti];
    const auto& rows = rows_per_target[ti];

    // Split into primal columns.
    std::vector<int> primal_vars;
    for (int v : full_vars) {
      if (v < num_cols) primal_vars.push_back(v);
    }

    int nrows = static_cast<int>(rows.size());
    int ncols = static_cast<int>(primal_vars.size());
    RowGroup group;
    group.primal_variables = primal_vars;
    group.global_rows = rows;
    group.C.setZero(nrows, ncols);
    group.d.resize(nrows);
    group.dual_variables.resize(nrows);

    // Map global row/col → local index for O(1) lookup.
    std::vector<int> row_map(C_.rows(), -1);
    for (int i = 0; i < nrows; ++i) {
      row_map[rows[i]] = i;
      group.d(i) = d_(rows[i]);
      group.dual_variables[i] = row_to_dual[rows[i]];
    }
    std::vector<int> col_map(C_.cols(), -1);
    for (int j = 0; j < ncols; ++j) col_map[primal_vars[j]] = j;

    // Iterate sparse nonzeros: O(nnz) instead of O(nrows * ncols * log).
    for (int k = 0; k < C_.outerSize(); ++k) {
      int lj = col_map[k];
      if (lj < 0) continue;
      for (Eigen::SparseMatrix<double>::InnerIterator it(C_, k); it; ++it) {
        int li = row_map[it.row()];
        if (li >= 0) group.C(li, lj) = it.value();
      }
    }

    // Clear maps for next target.
    for (int r : rows) row_map[r] = -1;
    for (int v : primal_vars) col_map[v] = -1;

    result.push_back(std::move(group));
  }
  return result;
}

SparseEqualityConstraintAssembler::SparseEqualityConstraintAssembler(
    std::unique_ptr<SparseEqualityConstraint> sec,
    const std::vector<int>& primal_variables,
    const std::vector<int>& dual_variables)
    : CliqueProvider(primal_variables, dual_variables),
      sec_(std::move(sec)),
      row_to_dual_(dual_variables) {}

std::vector<std::vector<int>>
SparseEqualityConstraintAssembler::get_cliques() const {
  // Each support group contributes a clique: {support ∪ dual vars of group}.
  std::vector<std::vector<int>> cliques;
  int dual_offset = 0;
  for (const auto& support : sec_->row_supports()) {
    // Count rows in this support group.
    // (support_groups_ are indexed identically to unique_supports_)
    // We need the number of rows per group. Since we don't expose
    // support_groups_ directly, reconstruct from row_supports().
    // Actually, we can't easily get group size from unique_supports_.
    // Instead, iterate row_to_dual_ to find dual vars per support.
  }

  // Rebuild from the internal structure by scanning the sparse matrix.
  // Map local column indices to global primal variable indices.
  const auto& pv = primal_variables();
  const int num_rows = sec_->C().rows();
  std::vector<std::vector<int>> row_supports(num_rows);
  for (int k = 0; k < sec_->C().outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sec_->C(), k); it;
         ++it) {
      // Map local column to global primal variable.
      int global_col = (it.col() < static_cast<int>(pv.size()))
                            ? pv[it.col()]
                            : it.col();
      row_supports[it.row()].push_back(global_col);
    }
  }
  // Sort each row's support (may be unsorted after global mapping).
  for (auto& rs : row_supports) std::sort(rs.begin(), rs.end());

  // Group rows by support, collect dual vars per group.
  std::vector<int> row_order(num_rows);
  std::iota(row_order.begin(), row_order.end(), 0);
  std::sort(row_order.begin(), row_order.end(), [&](int a, int b) {
    return row_supports[a] < row_supports[b];
  });

  for (int row : row_order) {
    if (row_supports[row].empty()) continue;
    if (cliques.empty() || !std::equal(
            row_supports[row].begin(), row_supports[row].end(),
            cliques.back().begin(),
            cliques.back().begin() +
                static_cast<int>(row_supports[row].size()))) {
      // New support group: start with the primal support (global indices).
      cliques.push_back(row_supports[row]);
    }
    // Append this row's dual variable.
    cliques.back().push_back(row_to_dual_[row]);
  }

  return cliques;
}

std::vector<SupernodalAssemblerBase*>
SparseEqualityConstraintAssembler::Decompose(
    const std::vector<std::vector<int>>& maximal_cliques) {
  // Remap maximal cliques from global variable indices back to local
  // column indices so GetConstraints (which uses local supports) can
  // match rows to targets correctly.
  const auto& pv = primal_variables();
  std::unordered_map<int, int> global_to_local;
  for (int j = 0; j < static_cast<int>(pv.size()); ++j)
    global_to_local[pv[j]] = j;

  // Build local cliques: remap primal vars to local indices,
  // keep dual vars as-is (they're global and checked by GetConstraints).
  std::vector<std::vector<int>> local_cliques;
  local_cliques.reserve(maximal_cliques.size());
  for (const auto& clique : maximal_cliques) {
    std::vector<int> local;
    for (int v : clique) {
      auto it = global_to_local.find(v);
      if (it != global_to_local.end()) {
        local.push_back(it->second);  // primal: remap to local
      } else {
        local.push_back(v);  // dual: keep global
      }
    }
    std::sort(local.begin(), local.end());
    local_cliques.push_back(std::move(local));
  }

  auto groups = sec_->GetConstraints(local_cliques, row_to_dual_);

  // Remap group primal variables back to global indices.
  std::vector<SupernodalAssemblerBase*> result;
  for (auto& group : groups) {
    std::vector<int> global_primals;
    global_primals.reserve(group.primal_variables.size());
    for (int lc : group.primal_variables) {
      global_primals.push_back(pv[lc]);
    }
    owned_assemblers_.emplace_back(
        group.C, group.d, global_primals, group.dual_variables);
    result.push_back(&owned_assemblers_.back());
  }
  return result;
}

}  // namespace conex
