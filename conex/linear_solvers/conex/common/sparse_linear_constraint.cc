#include "conex/common/sparse_linear_constraint.h"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <set>

#include "conex/common/linear_constraint.h"
#include "conex/common/workspace.h"

namespace conex {

namespace {

// Returns true if a ⊆ b (both sorted).
bool IsSubset(const std::vector<int>& a, const std::vector<int>& b) {
  return std::includes(b.begin(), b.end(), a.begin(), a.end());
}

}  // namespace

SparseLinearConstraint::SparseLinearConstraint(
    const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b)
    : A_(A), b_(b) {
  CONEX_DEMAND(A.rows() == b.rows(),
               "A and b must have the same number of rows.");

  // Build row supports in O(nnz).
  // A is column-major, so iterating columns gives sorted supports directly.
  std::vector<std::vector<int>> row_supports(A.rows());
  for (int k = 0; k < A.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      row_supports[it.row()].push_back(it.col());
    }
  }

  // Group rows by identical support.
  std::vector<int> row_order(A.rows());
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

std::vector<SparseLinearConstraint::RowGroup>
SparseLinearConstraint::GetConstraints(
    const std::vector<std::vector<int>>& target_supports) const {
  // Assign each support group to the smallest target that contains it.
  // target_index[i] = index into target_supports for support_groups_[i].
  std::vector<int> target_index(support_groups_.size(), -1);

  for (size_t si = 0; si < support_groups_.size(); ++si) {
    const auto& support = support_groups_[si].support;
    int best = -1;
    size_t best_size = std::numeric_limits<size_t>::max();
    for (size_t ti = 0; ti < target_supports.size(); ++ti) {
      const auto& target = target_supports[ti];
      if (target.size() < support.size()) continue;
      if (target.size() >= best_size) continue;
      if (IsSubset(support, target)) {
        best = static_cast<int>(ti);
        best_size = target.size();
      }
    }
    CONEX_DEMAND(best >= 0,
                 "No target support contains a row support set.");
    target_index[si] = best;
  }

  // Collect rows per target.
  std::vector<std::vector<int>> rows_per_target(target_supports.size());
  for (size_t si = 0; si < support_groups_.size(); ++si) {
    auto& dst = rows_per_target[target_index[si]];
    dst.insert(dst.end(), support_groups_[si].rows.begin(),
               support_groups_[si].rows.end());
  }

  // Build dense sub-blocks.
  std::vector<RowGroup> result;
  for (size_t ti = 0; ti < target_supports.size(); ++ti) {
    if (rows_per_target[ti].empty()) continue;
    const auto& vars = target_supports[ti];
    const auto& rows = rows_per_target[ti];
    int nrows = static_cast<int>(rows.size());
    int ncols = static_cast<int>(vars.size());
    RowGroup group;
    group.variables = vars;
    group.global_rows = rows;
    group.A.resize(nrows, ncols);
    group.b.resize(nrows);
    for (int i = 0; i < nrows; ++i) {
      group.b(i) = b_(rows[i]);
      for (int j = 0; j < ncols; ++j) {
        group.A(i, j) = A_.coeff(rows[i], vars[j]);
      }
    }
    result.push_back(std::move(group));
  }
  return result;
}


std::vector<SparseLinearConstraint::RowGroup>
SparseLinearConstraintAssembler::DecomposeRaw(
    const std::vector<std::vector<int>>& maximal_cliques) const {
  const int num_cols = slc_->A().cols();
  std::vector<std::vector<int>> primal_cliques;
  primal_cliques.reserve(maximal_cliques.size());
  for (const auto& clique : maximal_cliques) {
    std::vector<int> filtered;
    for (int v : clique) {
      if (v < num_cols) filtered.push_back(v);
    }
    if (!filtered.empty()) primal_cliques.push_back(std::move(filtered));
  }
  return slc_->GetConstraints(primal_cliques);
}

SparseLinearConstraintAssembler::SparseLinearConstraintAssembler(
    std::unique_ptr<SparseLinearConstraint> slc,
    const std::vector<int>& all_variables)
    : SupernodalAssemblerBase(all_variables), slc_(std::move(slc)) {}

std::vector<SupernodalAssemblerBase*>
SparseLinearConstraintAssembler::Decompose(
    const std::vector<std::vector<int>>& maximal_cliques) {
  // Filter maximal cliques to only include primal variables (< A.cols()).
  // MakeTreeSolver may produce cliques containing dual variable indices
  // from equality constraints, which are beyond the SLC's column range.
  const int num_cols = slc_->A().cols();
  std::vector<std::vector<int>> primal_cliques;
  primal_cliques.reserve(maximal_cliques.size());
  for (const auto& clique : maximal_cliques) {
    std::vector<int> filtered;
    for (int v : clique) {
      if (v < num_cols) {
        filtered.push_back(v);
      }
    }
    if (!filtered.empty()) {
      primal_cliques.push_back(std::move(filtered));
    }
  }
  auto groups = slc_->GetConstraints(primal_cliques);

  // Build row mapping: global row → (constraint index, local row).
  num_global_rows_ = slc_->A().rows();
  row_map_.resize(num_global_rows_, {-1, -1});

  std::vector<SupernodalAssemblerBase*> result;
  int constraint_index = 0;
  for (auto& group : groups) {
    auto constraint = std::make_unique<LinearConstraint>(group.A, group.b);
    constraint->SetPrimalVariables(group.variables);

    // Record row mapping.
    for (int local = 0; local < static_cast<int>(group.global_rows.size());
         ++local) {
      int global = group.global_rows[local];
      row_map_[global] = {constraint_index, local};
    }

    // Allocate workspace via ArenaAllocatable interface.
    size_t bytes = constraint->RequiredArenaBytes();
    owned_workspace_memory_.emplace_back(bytes / sizeof(double) + 1);
    constraint->BindArenaMemory(owned_workspace_memory_.back().data(), bytes);

    result.push_back(constraint.get());
    owned_constraints_.push_back(std::move(constraint));
    constraint_index++;
  }
  return result;
}

}  // namespace conex
