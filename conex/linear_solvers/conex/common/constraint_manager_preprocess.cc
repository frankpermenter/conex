#include "conex/common/constraint_manager.h"

#include <set>

#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/structural_rank.h"
#include <Eigen/Sparse>

namespace conex {

void ConstraintManager::Preprocess() {
  // Collect all SparseLinearConstraintAssembler pointers.
  std::vector<SparseLinearConstraintAssembler*> slc_assemblers;
  for (auto* assembler : custom_assemblers_) {
    auto* slc = dynamic_cast<SparseLinearConstraintAssembler*>(assembler);
    if (slc) slc_assemblers.push_back(slc);
  }
  if (slc_assemblers.empty()) return;

  const int n = max_number_of_variables_;

  // Stack all SLC matrices vertically for a combined structural rank check.
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

  // Check structural rank.
  std::vector<int> col_map;
  Eigen::SparseMatrix<double> A_reduced =
      DropStructurallyDependentColumns(A_combined, &col_map);

  if (static_cast<int>(col_map.size()) == n) return;  // full rank

  // Build reduction maps.
  was_reduced_ = true;
  original_num_variables_ = n;
  column_map_ = col_map;
  inverse_col_map_.assign(n, -1);
  for (int i = 0; i < static_cast<int>(col_map.size()); ++i) {
    inverse_col_map_[col_map[i]] = i;
  }

  int n_reduced = static_cast<int>(col_map.size());

  // Rebuild each SLC assembler with reduced columns.
  for (size_t ai = 0; ai < custom_assemblers_.size(); ++ai) {
    auto* slc = dynamic_cast<SparseLinearConstraintAssembler*>(
        custom_assemblers_[ai]);
    if (!slc) continue;

    const auto& A_orig = slc->sparse_matrix();
    const auto& b_orig = slc->rhs_vector();

    // Build reduced A: remap columns via inverse_col_map_.
    std::vector<Eigen::Triplet<double>> t;
    for (int k = 0; k < A_orig.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A_orig, k); it; ++it) {
        int new_col = inverse_col_map_[it.col()];
        if (new_col >= 0) t.emplace_back(it.row(), new_col, it.value());
      }
    }
    Eigen::SparseMatrix<double> A_new(A_orig.rows(), n_reduced);
    A_new.setFromTriplets(t.begin(), t.end());

    // Build new variable set.
    auto new_slc = std::make_unique<SparseLinearConstraint>(A_new, b_orig);
    std::set<int> var_set;
    for (const auto& sup : new_slc->row_supports())
      var_set.insert(sup.begin(), sup.end());
    std::vector<int> new_vars(var_set.begin(), var_set.end());
    auto new_assembler = std::make_unique<SparseLinearConstraintAssembler>(
        std::move(new_slc), new_vars);

    // Update pointer in custom_assemblers_.
    custom_assemblers_[ai] = new_assembler.get();
    owned_custom_assemblers_.push_back(std::move(new_assembler));
  }

  // Update variable count.
  max_number_of_variables_ = n_reduced;
  new_dual_variable_start_ = n_reduced;
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
