#include "conex/common/constraint_manager.h"

#include <cmath>
#include <numeric>
#include <set>

#include "conex/common/error_checking_macros.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/structural_rank.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

void ConstraintManager::Preprocess() {
  const int n = max_number_of_variables_;

  // Phase 1: Column reduction for SparseLinearConstraint assemblers.
  std::vector<SparseLinearConstraintAssembler*> slc_assemblers;
  for (auto* assembler : custom_assemblers_) {
    auto* slc = dynamic_cast<SparseLinearConstraintAssembler*>(assembler);
    if (slc) slc_assemblers.push_back(slc);
  }

  if (!slc_assemblers.empty()) {
    int total_rows = 0;
    for (auto* slc : slc_assemblers) total_rows += slc->sparse_matrix().rows();

    std::vector<Eigen::Triplet<double>> trips;
    int row_offset = 0;
    for (auto* slc : slc_assemblers) {
      const auto& A = slc->sparse_matrix();
      for (int k = 0; k < A.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
          trips.emplace_back(row_offset + it.row(), it.col(), it.value());
      row_offset += A.rows();
    }
    Eigen::SparseMatrix<double> A_combined(total_rows, n);
    A_combined.setFromTriplets(trips.begin(), trips.end());

    std::vector<int> col_map;
    DropStructurallyDependentColumns(A_combined, &col_map);

    if (static_cast<int>(col_map.size()) < n) {
      was_reduced_ = true;
      original_num_variables_ = n;
      column_map_ = col_map;
      inverse_col_map_.assign(n, -1);
      for (int i = 0; i < static_cast<int>(col_map.size()); ++i) {
        inverse_col_map_[col_map[i]] = i;
      }

      int n_reduced = static_cast<int>(col_map.size());

      for (size_t ai = 0; ai < custom_assemblers_.size(); ++ai) {
        auto* slc = dynamic_cast<SparseLinearConstraintAssembler*>(
            custom_assemblers_[ai]);
        if (!slc) continue;

        const auto& A_orig = slc->sparse_matrix();
        const auto& b_orig = slc->rhs_vector();

        std::vector<Eigen::Triplet<double>> t;
        for (int k = 0; k < A_orig.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(A_orig, k); it;
               ++it) {
            int new_col = inverse_col_map_[it.col()];
            if (new_col >= 0) t.emplace_back(it.row(), new_col, it.value());
          }
        Eigen::SparseMatrix<double> A_new(A_orig.rows(), n_reduced);
        A_new.setFromTriplets(t.begin(), t.end());

        auto new_slc = std::make_unique<SparseLinearConstraint>(A_new, b_orig);
        std::set<int> var_set;
        for (const auto& sup : new_slc->row_supports())
          var_set.insert(sup.begin(), sup.end());
        std::vector<int> new_vars(var_set.begin(), var_set.end());
        auto new_assembler = std::make_unique<SparseLinearConstraintAssembler>(
            std::move(new_slc), new_vars);

        custom_assemblers_[ai] = new_assembler.get();
        owned_custom_assemblers_.push_back(std::move(new_assembler));
      }

      max_number_of_variables_ = n_reduced;
      new_dual_variable_start_ = n_reduced;
    }
  }

  // Phase 2: Equality constraints — remap columns and drop dependent rows.
  if (equality_constraints_.data.empty()) return;

  new_dual_variable_start_ = max_number_of_variables_;
  equality_constraints_.dual_variables.clear();
  equality_constraints_.assemblers.clear();

  for (size_t ei = 0; ei < equality_constraints_.data.size(); ++ei) {
    auto& eq = equality_constraints_.data[ei];
    auto& vars = equality_constraints_.variables[ei];

    // Remap variables and columns if column reduction was done.
    if (was_reduced_) {
      Eigen::MatrixXd C_old = eq.A_;
      std::vector<int> old_vars = vars;

      std::vector<int> new_vars;
      std::vector<int> kept_cols;
      for (int j = 0; j < static_cast<int>(old_vars.size()); ++j) {
        int new_idx = inverse_col_map_[old_vars[j]];
        if (new_idx >= 0) {
          new_vars.push_back(new_idx);
          kept_cols.push_back(j);
        }
      }

      Eigen::MatrixXd C_new(eq.A_.rows(), static_cast<int>(kept_cols.size()));
      for (int j = 0; j < static_cast<int>(kept_cols.size()); ++j) {
        C_new.col(j) = C_old.col(kept_cols[j]);
      }

      eq.A_ = C_new;
      vars = new_vars;
    }

    // Drop structurally dependent rows.
    int nr = eq.A_.rows();
    Eigen::SparseMatrix<double> C_sparse = eq.A_.sparseView();

    std::vector<int> row_map;
    DropStructurallyDependentRows(C_sparse, &row_map);

    if (static_cast<int>(row_map.size()) < nr) {
      int nr_new = static_cast<int>(row_map.size());

      // Check dropped rows for inconsistency before removing them.
      // A dropped row c_j^T x = d_j is inconsistent if c_j is in the
      // row space of the kept rows C_k but d_j != lambda^T d_k
      // (where c_j = lambda^T C_k).
      std::vector<bool> kept(nr, false);
      for (int idx : row_map) kept[idx] = true;

      Eigen::MatrixXd C_kept(nr_new, eq.A_.cols());
      Eigen::VectorXd d_kept(nr_new);
      for (int i = 0; i < nr_new; ++i) {
        C_kept.row(i) = eq.A_.row(row_map[i]);
        d_kept(i) = eq.b_(row_map[i]);
      }

      // Factorize C_kept^T once for all dropped rows.
      auto qr = C_kept.transpose().colPivHouseholderQr();

      for (int r = 0; r < nr; ++r) {
        if (kept[r]) continue;
        Eigen::VectorXd c_dropped = eq.A_.row(r).transpose();
        double d_dropped = eq.b_(r);

        // Solve C_kept^T * lambda = c_dropped.
        Eigen::VectorXd lambda = qr.solve(c_dropped);
        double c_residual = (C_kept.transpose() * lambda - c_dropped).norm();
        double c_scale = std::max(c_dropped.norm(), 1.0);

        // Only check RHS if the row is numerically in the row space.
        if (c_residual < 1e-10 * c_scale) {
          double d_predicted = lambda.dot(d_kept);
          double d_err = std::abs(d_predicted - d_dropped);
          double d_scale = std::max(std::abs(d_dropped), 1.0);
          CONEX_DEMAND(d_err < 1e-10 * d_scale,
                       "Inconsistent equality constraints: a structurally "
                       "dependent row is in the row space of the kept rows "
                       "but its right-hand side is incompatible.");
        }
      }

      Eigen::MatrixXd C_reduced(nr_new, eq.A_.cols());
      Eigen::MatrixXd b_reduced(nr_new, eq.b_.cols());
      for (int i = 0; i < nr_new; ++i) {
        C_reduced.row(i) = eq.A_.row(row_map[i]);
        b_reduced.row(i) = eq.b_.row(row_map[i]);
      }
      eq.A_ = C_reduced;
      eq.b_ = b_reduced;
    }

    // Allocate dual variables.
    int num_dual = eq.A_.rows();
    std::vector<int> dual_vars(num_dual);
    std::iota(dual_vars.begin(), dual_vars.end(), new_dual_variable_start_);
    new_dual_variable_start_ += num_dual;
    equality_constraints_.dual_variables.push_back(dual_vars);

    // Rebuild assembler.
    equality_constraints_.assemblers.emplace_back(
        eq.A_, Eigen::VectorXd(eq.b_), vars, dual_vars);
  }
}

Eigen::VectorXd ConstraintManager::ExpandSolution(
    const Eigen::VectorXd& x_reduced) const {
  if (!was_reduced_) return x_reduced;
  Eigen::VectorXd x(original_num_variables_);
  x.setZero();
  for (int i = 0; i < static_cast<int>(column_map_.size()); ++i) {
    x(column_map_[i]) = x_reduced(i);
  }
  return x;
}

Eigen::VectorXd ConstraintManager::ReduceVector(
    const Eigen::VectorXd& v_original) const {
  if (!was_reduced_) return v_original;
  Eigen::VectorXd v(column_map_.size());
  for (int i = 0; i < static_cast<int>(column_map_.size()); ++i) {
    v(i) = v_original(column_map_[i]);
  }
  return v;
}

}  // namespace conex
