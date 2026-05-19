#include "conex/common/model.h"

#include <cmath>
#include <numeric>
#include <set>
#include <stdexcept>

#include "conex/common/eja_ops.h"
#include "conex/common/kkt_system.h"
#include "conex/common/structural_rank.h"
#include "conex/linear_solvers/cholesky_solvers.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

namespace conex {

std::vector<int> FindDependentEquations(
    const Eigen::SparseMatrix<double>& C,
    const std::vector<int>& primal_vars,
    double threshold,
    Arena* external_arena) {
  const int m = C.rows();  // number of equations
  if (m == 0) return {};

  // Remove zero columns from C before building the gram model.
  // Zero columns correspond to primal variables not touched by any
  // equation — they can't contribute to row dependencies.
  std::vector<int> col_nnz(C.cols(), 0);
  for (int k = 0; k < C.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(C, k); it; ++it)
      col_nnz[it.col()]++;
  std::vector<int> keep_cols;
  for (int j = 0; j < C.cols(); ++j)
    if (col_nnz[j] > 0) keep_cols.push_back(j);

  // Build C_reduced with only non-zero columns.
  std::vector<int> col_remap(C.cols(), -1);
  for (int i = 0; i < (int)keep_cols.size(); ++i)
    col_remap[keep_cols[i]] = i;
  std::vector<Eigen::Triplet<double>> trips;
  for (int k = 0; k < C.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(C, k); it; ++it) {
      int nc = col_remap[it.col()];
      if (nc >= 0) trips.emplace_back(it.row(), nc, it.value());
    }
  Eigen::SparseMatrix<double> C_nz(m, (int)keep_cols.size());
  C_nz.setFromTriplets(trips.begin(), trips.end());

  // Build a model with m variables and one linear constraint A = C_nz^T.
  // The KKT normal equations form A^T*W*A = C_nz*C_nz^T = C*C^T.
  Eigen::SparseMatrix<double> Ct = C_nz.transpose();
  std::vector<int> eq_vars(m);
  std::iota(eq_vars.begin(), eq_vars.end(), 0);

  Model gram_model;
  gram_model.AddLinearConstraint(Ct, Eigen::VectorXd::Zero(Ct.rows()), eq_vars);

  // Build tree solver with CholeskySkipZero subsystems.
  SolverConfiguration config;
  double thr = threshold;
  config.tree.subsystem_factory = [thr]()
      -> std::unique_ptr<KKTSubsystemBase> {
    auto s = std::make_unique<CholeskySkipZero>();
    s->SetThreshold(thr);
    return s;
  };
  Arena local_arena;
  Arena* arena = external_arena ? external_arena : &local_arena;
  auto cursor = arena->SaveCursor();
  auto system = KKTSystem::Build(gram_model, config, arena);
  auto* kkt = system.kkt();
  auto* ts = system.tree_solver();
  if (!ts) return {};

  // Set W=I and factor via the tree solver.
  auto w = kkt->MakeRowSpace();
  EuclideanJordanAlgebra::setOnes(w);
  kkt->SetScaling(w);
  kkt->AssembleAndFactor();

  // Collect zero pivots from all subsystems and map back to
  // original equation indices via the elimination permutation.
  const auto& perm_inv = ts->perm_inv();
  std::vector<int> dependent;
  int elim_pos = 0;
  for (auto* sub : ts->solve_order()) {
    auto* csz = dynamic_cast<CholeskySkipZero*>(sub);
    if (!csz) {
      auto* ks = dynamic_cast<KKTSubsystem*>(sub);
      if (ks) elim_pos += ks->supernode_submatrix().rows();
      continue;
    }
    for (int local_idx : csz->zero_pivot_positions()) {
      dependent.push_back(perm_inv[elim_pos + local_idx]);
    }
    elim_pos += csz->supernode_submatrix().rows();
  }

  std::sort(dependent.begin(), dependent.end());

  // Release arena memory used by the temporary tree solver.
  if (cursor) arena->RestoreCursor(cursor);
  return dependent;
}

std::pair<Model, Expansion> RemoveStructuralRankDeficiency(
    const Model& problem) {
  const int n = problem.num_variables();

  // Phase 1: Column reduction for linear and PSD constraints.
  std::vector<Eigen::Triplet<double>> trips;
  int total_rows = 0;
  for (const auto& c : problem.constraints()) {
    // Linear and SOC have the same A matrix structure.
    const Eigen::SparseMatrix<double>* A_ptr = nullptr;
    const std::vector<int>* vars_ptr = nullptr;
    if (auto* lc = std::get_if<Model::LinearConstraintData>(&c)) {
      A_ptr = &lc->A;
      vars_ptr = &lc->vars;
    } else if (auto* sc = std::get_if<Model::SOCConstraintData>(&c)) {
      A_ptr = &sc->A;
      vars_ptr = &sc->vars;
    } else if (auto* bc = std::get_if<Model::BarrierConstraintData>(&c)) {
      A_ptr = &bc->A;
      vars_ptr = &bc->vars;
    }
    if (A_ptr) {
      for (int k = 0; k < A_ptr->outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(*A_ptr, k); it; ++it)
          trips.emplace_back(total_rows + it.row(),
                             (*vars_ptr)[it.col()], it.value());
      total_rows += A_ptr->rows();
    } else if (auto* qc = std::get_if<Model::QuadraticCostData>(&c)) {
      // Quadratic cost variables are structurally present (they contribute
      // to the Gram matrix diagonal).  Add one row per variable to prevent
      // them from being dropped as structurally rank-deficient.
      for (int v : qc->vars) {
        trips.emplace_back(total_rows, v, 1.0);
        total_rows++;
      }
    } else if (auto* ec = std::get_if<Model::EqualityConstraintData>(&c)) {
      // Equality constraint primal variables are structurally present.
      for (int k = 0; k < ec->C.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(ec->C, k); it; ++it)
          trips.emplace_back(total_rows + it.row(),
                             ec->primal_vars[it.col()], it.value());
      total_rows += ec->C.rows();
    } else if (auto* pc = std::get_if<Model::PSDConstraintData>(&c)) {
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
  Model reduced;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData> ||
                     std::is_same_v<T, Model::SOCConstraintData> ||
                     std::is_same_v<T, Model::BarrierConstraintData>) {
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
        if constexpr (std::is_same_v<T, Model::SOCConstraintData>)
          reduced.AddSOCConstraint(A_new, data.b, new_vars);
        else if constexpr (std::is_same_v<T, Model::BarrierConstraintData>)
          reduced.AddBarrierConstraint(A_new, data.b, new_vars, data.ops);
        else
          reduced.AddLinearConstraint(A_new, data.b, new_vars);

      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
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

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
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
                                          Model::EqualityConstraintData>) {
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

        // Detect numerically dependent rows via C*C^T tree factorization
        // (CholeskySkipZero detects zero pivots).  Run on the remapped
        // matrix directly — this supersedes structural reduction.
        int p_orig = C_remapped.rows();
        std::vector<int> dep_rows;
        // Run numerical dependency check when the system is
        // underdetermined (more columns than rows) and non-trivial.
        if (p_orig > 1 && new_ncols > p_orig) {
          dep_rows = FindDependentEquations(C_remapped, new_primal);
        }

        // Fall back to structural reduction when numerical check
        // didn't run (overdetermined or degenerate systems).
        if (dep_rows.empty() && p_orig > 1) {
          std::vector<int> struct_row_map;
          auto C_struct = DropStructurallyDependentRows(C_remapped,
                                                        &struct_row_map);
          if (C_struct.rows() < p_orig) {
            // Mark rows NOT in struct_row_map as dependent.
            std::set<int> kept(struct_row_map.begin(),
                               struct_row_map.end());
            for (int r = 0; r < p_orig; ++r)
              if (!kept.count(r)) dep_rows.push_back(r);
          }
        }

        if (!dep_rows.empty()) {
          std::set<int> dep_set(dep_rows.begin(), dep_rows.end());
          // Build kept rows.
          std::vector<int> kept;
          for (int r = 0; r < p_orig; ++r)
            if (!dep_set.count(r)) kept.push_back(r);
          int p_kept = static_cast<int>(kept.size());

          std::vector<int> row_remap(p_orig, -1);
          for (int i = 0; i < p_kept; ++i) row_remap[kept[i]] = i;

          std::vector<Eigen::Triplet<double>> trips;
          for (int k = 0; k < C_remapped.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(C_remapped, k);
                 it; ++it) {
              int nr = row_remap[it.row()];
              if (nr >= 0) trips.emplace_back(nr, it.col(), it.value());
            }
          Eigen::SparseMatrix<double> C_kept(p_kept, new_ncols);
          C_kept.setFromTriplets(trips.begin(), trips.end());
          Eigen::VectorXd d_kept(p_kept);
          for (int i = 0; i < p_kept; ++i)
            d_kept(i) = data.d(kept[i]);

          // Check consistency of dropped rows.
          if (p_kept > 0) {
            Eigen::MatrixXd Ck_dense(C_kept);
            Eigen::VectorXd x_check =
                Ck_dense.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                    .solve(d_kept);
            Eigen::MatrixXd C_dense(C_remapped);
            for (int r : dep_rows) {
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
          reduced.AddEqualityConstraint(C_kept, d_kept, new_primal);
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

std::pair<Model, RowScaling> RowScaleModel(const Model& model) {
  Model scaled;
  RowScaling scaling;
  scaling.row_scale.resize(model.num_constraints());

  for (int i = 0; i < model.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        // SOC constraints cannot be row-scaled independently —
        // per-row scaling destroys cone structure (||λ₁|| ≤ λ₀).
        scaled.AddSOCConstraint(data.A, data.b, data.vars);

      } else if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        int m = data.A.rows();
        Eigen::VectorXd scale(m);
        for (int r = 0; r < m; ++r) {
          // Scale by max(|b_i|, ||A_i||_inf) to avoid division by zero.
          double row_norm = 0;
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, 0);
               it; ++it) {
            // This iterates column 0 only; need all columns.
          }
          // Compute row norm via column iteration.
          row_norm = 0;
          for (int k = 0; k < data.A.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
                 it; ++it)
              if (it.row() == r)
                row_norm = std::max(row_norm, std::abs(it.value()));
          double bi = std::abs(data.b(r));
          scale(r) = std::max(bi, row_norm);
          if (scale(r) < 1e-15) scale(r) = 1.0;  // degenerate row
        }
        scaling.row_scale[i] = scale;

        // Build scaled A and b.
        Eigen::VectorXd inv_scale = scale.cwiseInverse();
        // Scale rows: A_scaled(r,:) = A(r,:) / scale(r).
        std::vector<Eigen::Triplet<double>> trips;
        for (int k = 0; k < data.A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
               it; ++it)
            trips.emplace_back(it.row(), it.col(),
                               it.value() * inv_scale(it.row()));
        Eigen::SparseMatrix<double> A_scaled(m, data.A.cols());
        A_scaled.setFromTriplets(trips.begin(), trips.end());
        Eigen::VectorXd b_scaled = data.b.cwiseProduct(inv_scale);

        scaled.AddLinearConstraint(A_scaled, b_scaled, data.vars);

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        scaled.AddQuadraticCost(data.Q_sparse, data.vars);
      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        scaled.AddPSDConstraint(data.A_list, data.B, data.vars,
                                 data.use_chordal);
      } else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        scaled.AddEqualityConstraint(data.C, data.d, data.primal_vars);
      } else if constexpr (std::is_same_v<T, Model::BarrierConstraintData>) {
        // Barrier constraints: pass through unscaled (row scaling
        // would break the cone structure).
        scaled.AddBarrierConstraint(data.A, data.b, data.vars, data.ops);
      }
    }, model.constraint(i));
  }

  if (model.has_linear_cost()) {
    scaled.SetLinearCost(model.linear_cost());
  }

  return {scaled, scaling};
}

}  // namespace conex
