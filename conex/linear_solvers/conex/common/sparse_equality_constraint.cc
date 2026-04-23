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
  // Assign each nonzero C(row,col) to the smallest target clique containing
  // both primal_var[col] and dual_var[row].  A single row may be split
  // across multiple cliques (e.g., a dense equality on a star tree).
  // The RHS d(row) is assigned to the smallest target containing the dual.

  const int num_cols = C_.cols();
  const int num_rows = C_.rows();
  const int num_targets = static_cast<int>(target_supports.size());

  // Precompute sets for fast membership testing.
  std::vector<std::set<int>> target_sets(num_targets);
  for (int ti = 0; ti < num_targets; ++ti) {
    target_sets[ti].insert(target_supports[ti].begin(),
                           target_supports[ti].end());
  }

  // For each nonzero C(row, col), find the smallest target containing
  // both the column and the row's dual.
  struct Entry { int row, col; double value; };
  std::vector<std::vector<Entry>> entries_per_target(num_targets);

  for (int k = 0; k < C_.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(C_, k); it; ++it) {
      int row = it.row(), col = it.col();
      int dual = row_to_dual[row];
      int best = -1;
      size_t best_size = std::numeric_limits<size_t>::max();
      for (int ti = 0; ti < num_targets; ++ti) {
        if (target_supports[ti].size() >= best_size) continue;
        if (!target_sets[ti].count(dual)) continue;
        if (!target_sets[ti].count(col)) continue;
        best = ti;
        best_size = target_supports[ti].size();
      }
      CONEX_DEMAND(best >= 0,
                   "No target contains both primal column and dual var.");
      entries_per_target[best].push_back({row, col, it.value()});
    }
  }

  // RHS d(row) goes to the smallest target containing the dual.
  std::vector<int> rhs_target(num_rows, -1);
  for (int row = 0; row < num_rows; ++row) {
    int dual = row_to_dual[row];
    int best = -1;
    size_t best_size = std::numeric_limits<size_t>::max();
    for (int ti = 0; ti < num_targets; ++ti) {
      if (target_supports[ti].size() >= best_size) continue;
      if (!target_sets[ti].count(dual)) continue;
      best = ti;
      best_size = target_supports[ti].size();
    }
    rhs_target[row] = best;
  }

  // Build RowGroups from the per-target entries.
  std::vector<RowGroup> result;
  for (int ti = 0; ti < num_targets; ++ti) {
    if (entries_per_target[ti].empty()) continue;

    // Collect unique rows and columns for this target.
    std::set<int> row_set, col_set;
    for (const auto& e : entries_per_target[ti]) {
      row_set.insert(e.row);
      col_set.insert(e.col);
    }
    std::vector<int> rows(row_set.begin(), row_set.end());
    std::vector<int> cols(col_set.begin(), col_set.end());
    int nrows = static_cast<int>(rows.size());
    int ncols = static_cast<int>(cols.size());

    // Build local index maps.
    std::vector<int> row_map(num_rows, -1);
    for (int i = 0; i < nrows; ++i) row_map[rows[i]] = i;
    std::vector<int> col_map(num_cols, -1);
    for (int j = 0; j < ncols; ++j) col_map[cols[j]] = j;

    RowGroup group;
    group.primal_variables = cols;
    group.global_rows = rows;
    group.C.setZero(nrows, ncols);
    group.d.setZero(nrows);
    group.dual_variables.resize(nrows);

    for (int i = 0; i < nrows; ++i) {
      group.dual_variables[i] = row_to_dual[rows[i]];
      if (rhs_target[rows[i]] == ti) {
        group.d(i) = d_(rows[i]);
      }
    }
    for (const auto& e : entries_per_target[ti]) {
      group.C(row_map[e.row], col_map[e.col]) = e.value;
    }

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
  // The saddle-point matrix [0 C'; C 0] creates edges (x_j, ν_i) for each
  // nonzero C_{ij}, but NO edges between primal variables.  The maximal
  // cliques are size-2: {x_j, ν_i}.  Reporting these individually lets the
  // tree builder exploit the star structure (dual at root, primals as leaves).
  std::vector<std::vector<int>> cliques;
  const auto& C = sec_->C();
  const auto& pv = primal_variables();
  for (int k = 0; k < C.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(C, k); it; ++it) {
      int col = it.col();
      if (col >= static_cast<int>(pv.size())) continue;
      int global_primal = pv[col];
      int dual = row_to_dual_[it.row()];
      cliques.push_back({std::min(global_primal, dual),
                          std::max(global_primal, dual)});
    }
  }
  return cliques;
}

std::vector<SupernodalAssemblerBase*>
SparseEqualityConstraintAssembler::Decompose(
    const std::vector<std::vector<int>>& maximal_cliques) {
  // Remap primal vars to local; keep dual vars as-is (global).
  // GetConstraints checks both primal support (local) and dual var (global).
  std::unordered_map<int, int> g2l;
  const auto& pv = primal_variables();
  for (int j = 0; j < static_cast<int>(pv.size()); ++j)
    g2l[pv[j]] = j;

  std::vector<std::vector<int>> local_cliques;
  local_cliques.reserve(maximal_cliques.size());
  for (const auto& clique : maximal_cliques) {
    std::vector<int> lc;
    for (int v : clique) {
      auto it = g2l.find(v);
      lc.push_back(it != g2l.end() ? it->second : v);  // primal→local, dual→keep
    }
    std::sort(lc.begin(), lc.end());
    local_cliques.push_back(std::move(lc));
  }

  auto groups = sec_->GetConstraints(local_cliques, row_to_dual_);

  std::vector<SupernodalAssemblerBase*> result;
  for (auto& group : groups) {
    owned_assemblers_.emplace_back(
        group.C, group.d, RemapToGlobal(group.primal_variables),
        group.dual_variables);
    result.push_back(&owned_assemblers_.back());
  }
  return result;
}

}  // namespace conex
