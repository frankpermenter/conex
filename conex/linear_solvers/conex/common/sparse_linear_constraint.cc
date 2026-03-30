#include "conex/common/sparse_linear_constraint.h"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <set>

#include "conex/common/constraint_manager.h"
#include "conex/tree_solver/kkt_solver_factory.h"
#include "conex/tree_solver/kkt_tree_solver.h"
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

void SparseLinearConstraintAssembler::SetWeights(
    const Eigen::VectorXd& weights) {
  CONEX_DEMAND(weights.size() == num_global_rows_,
               "Weight vector size must match number of rows in A.");
  CONEX_DEMAND(!owned_constraints_.empty(),
               "Decompose must be called before SetWeights.");

  // The Gram evaluator computes (WA)^T (WA) = A^T W^2 A.
  // So to get A^T diag(weights) A, we store sqrt(weights) in W.
  for (int global = 0; global < num_global_rows_; ++global) {
    const auto& m = row_map_[global];
    if (m.constraint_index < 0) continue;
    owned_constraints_[m.constraint_index]->workspace()->W(m.local_row) =
        std::sqrt(weights(global));
  }

  // Update each constraint's Gram evaluator.
  for (auto& c : owned_constraints_) {
    c->GetBlockAssembler();  // Ensure evaluator is bound.
    static_cast<GramEvaluator*>(c->GetBlockAssembler())->update_weights();
  }
}

Eigen::VectorXd SparseLinearConstraintAssembler::ComputeResiduals(
    const Eigen::VectorXd& x) const {
  // Compute per-clique residuals and scatter to global vector.
  // Each constraint holds a dense A_clique and b_clique.

  // Step 1: compute per-constraint residuals.
  std::vector<Eigen::VectorXd> local_residuals(owned_constraints_.size());
  for (size_t ci = 0; ci < owned_constraints_.size(); ++ci) {
    const auto& constraint = owned_constraints_[ci];
    const auto& vars = constraint->primal_variables();
    const int nv = static_cast<int>(vars.size());

    Eigen::VectorXd x_local(nv);
    for (int j = 0; j < nv; ++j) {
      x_local(j) = x(vars[j]);
    }
    local_residuals[ci] = constraint->ComputeResidual(x_local);
  }

  // Step 2: scatter to global using row_map_ (O(m), not O(m * num_constraints)).
  Eigen::VectorXd residuals = Eigen::VectorXd::Zero(num_global_rows_);
  for (int global = 0; global < num_global_rows_; ++global) {
    const auto& m = row_map_[global];
    if (m.constraint_index >= 0) {
      residuals(global) = local_residuals[m.constraint_index](m.local_row);
    }
  }

  return residuals;
}

void SparseLinearConstraintAssembler::BindPartition(
    const SymmetricLinearSystemTreeSolver& solver) {
  block_info_.resize(owned_constraints_.size());
  const auto& perm = solver.perm();
  const auto& partition = solver.raw_partition();

  for (size_t ci = 0; ci < owned_constraints_.size(); ++ci) {
    const auto& constraint = owned_constraints_[ci];
    const auto& vars = constraint->primal_variables();

    auto* evaluator = const_cast<LinearConstraint*>(constraint.get())
                          ->GetBlockAssembler();
    auto* gram = static_cast<GramEvaluator*>(evaluator);

    int sn_count = gram->sn_count();
    block_info_[ci].sn_count = sn_count;

    // Find block: scan blocks to match supernode range containing a
    // constraint variable's elimination position.
    int block_idx = -1;
    if (sn_count > 0 && !vars.empty()) {
      for (int v : vars) {
        int ep = perm(v);
        int cum = 0;
        for (int k = 0; k < solver.num_subsystems(); ++k) {
          int sn_rows = partition.supernode_rows(k);
          if (ep >= cum && ep < cum + sn_rows) {
            block_idx = k;
            break;
          }
          cum += sn_rows;
        }
        if (block_idx >= 0) break;
      }
    }
    block_info_[ci].block_index = block_idx;
  }
}

Eigen::VectorXd SparseLinearConstraintAssembler::ComputeBlockResiduals(
    const KKTSolverBase& solver) const {
  if (!partition_bound()) {
    // Generic path: gather globally and compute residuals.
    const int n = solver.number_of_variables();
    Eigen::VectorXd x_global(n);
    solver.GatherFromBlocks(x_global);
    return ComputeResiduals(x_global);
  }

  // Tree-specific fast path: use supernode/separator blocks directly.
  auto* tree_solver =
      dynamic_cast<const SymmetricLinearSystemTreeSolver*>(&solver);
  if (!tree_solver) {
    // BindPartition was called but solver isn't a tree solver — fall back.
    const int n = solver.number_of_variables();
    Eigen::VectorXd x_global(n);
    solver.GatherFromBlocks(x_global);
    return ComputeResiduals(x_global);
  }

  const auto& partition = tree_solver->raw_partition();

  // Step 1: compute per-constraint residuals from block data.
  std::vector<Eigen::VectorXd> local_residuals(owned_constraints_.size());
  for (size_t ci = 0; ci < owned_constraints_.size(); ++ci) {
    const auto& constraint = owned_constraints_[ci];
    const auto& info = block_info_[ci];

    if (info.block_index < 0) {
      local_residuals[ci] = Eigen::VectorXd::Zero(constraint->num_rows());
      continue;
    }

    int k = info.block_index;
    int ns = info.sn_count;
    int n_vars = constraint->number_of_variables();
    auto full_sn = partition.supernode(k);
    auto full_sep = partition.separator(k);

    // After supernode merging, the partition's supernode/separator
    // sizes may exceed this constraint's sn/sep split.  If dimensions
    // match, use the fast block path; otherwise fall back to global.
    if (full_sn.rows() == ns &&
        full_sep.rows() == n_vars - ns) {
      local_residuals[ci] =
          constraint->ComputeBlockResidual(full_sn, full_sep);
    } else {
      // Gather this constraint's variables globally and compute.
      Eigen::VectorXd x_global(solver.number_of_variables());
      solver.GatherFromBlocks(x_global);
      const auto& vars = constraint->primal_variables();
      Eigen::VectorXd x_local(n_vars);
      for (int j = 0; j < n_vars; ++j) x_local(j) = x_global(vars[j]);
      local_residuals[ci] = constraint->ComputeResidual(x_local);
    }
  }

  // Step 2: scatter to global using row_map_ (O(m)).
  Eigen::VectorXd residuals = Eigen::VectorXd::Zero(num_global_rows_);
  for (int global = 0; global < num_global_rows_; ++global) {
    const auto& m = row_map_[global];
    if (m.constraint_index >= 0) {
      residuals(global) = local_residuals[m.constraint_index](m.local_row);
    }
  }

  return residuals;
}

Eigen::VectorXd SparseLinearConstraintAssembler::ComputeTransposeProduct(
    const Eigen::VectorXd& v) const {
  // Compute A^T * v per-clique using the decomposed dense blocks.
  // Each constraint holds A_clique (m_i x n_i). We compute
  // A_clique^T * v_local and scatter to the global result.
  const int n = slc_ ? slc_->A().cols() : 0;
  Eigen::VectorXd result = Eigen::VectorXd::Zero(n);

  // Step 1: gather v into per-constraint local vectors using row_map_.
  std::vector<Eigen::VectorXd> v_locals(owned_constraints_.size());
  for (size_t ci = 0; ci < owned_constraints_.size(); ++ci) {
    v_locals[ci] = Eigen::VectorXd::Zero(owned_constraints_[ci]->num_rows());
  }
  for (int global = 0; global < num_global_rows_; ++global) {
    const auto& m = row_map_[global];
    if (m.constraint_index >= 0) {
      v_locals[m.constraint_index](m.local_row) = v(global);
    }
  }

  // Step 2: compute A_clique^T * v_local and scatter to result.
  for (size_t ci = 0; ci < owned_constraints_.size(); ++ci) {
    const auto& constraint = owned_constraints_[ci];
    const auto& vars = constraint->primal_variables();
    Eigen::VectorXd atv =
        constraint->constraint_matrix().transpose() * v_locals[ci];
    for (int j = 0; j < static_cast<int>(vars.size()); ++j) {
      result(vars[j]) += atv(j);
    }
  }

  return result;
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
