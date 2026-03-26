#pragma once
#include <list>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/constraint.h"
#include "conex/common/linear_constraint.h"

namespace conex {

// Decomposes a sparse matrix A into dense sub-blocks grouped by column
// support.  The class holds the sparse matrix and precomputed per-row
// supports.  Callers choose a grouping strategy (containment merging,
// maximal-clique assignment, etc.) and call GetConstraints() with the
// desired target supports to extract dense sub-blocks.
class SparseLinearConstraint {
 public:
  SparseLinearConstraint(const Eigen::SparseMatrix<double>& A,
                         const Eigen::VectorXd& b);

  struct RowGroup {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::vector<int> variables;
    std::vector<int> global_rows;  // row indices in the original A
  };

  // The unique row supports of A (sorted, deduplicated).
  const std::vector<std::vector<int>>& row_supports() const {
    return unique_supports_;
  }

  const Eigen::SparseMatrix<double>& A() const { return A_; }

  // Given a list of target supports (e.g. maximal cliques), assign each
  // row to the smallest target that contains its support, then build
  // dense sub-blocks.
  std::vector<RowGroup> GetConstraints(
      const std::vector<std::vector<int>>& target_supports) const;

 private:
  Eigen::SparseMatrix<double> A_;
  Eigen::VectorXd b_;

  struct SupportGroup {
    std::vector<int> support;
    std::vector<int> rows;
  };
  std::vector<SupportGroup> support_groups_;
  std::vector<std::vector<int>> unique_supports_;
};

// Assembler that wraps a SparseLinearConstraint.  Its variables() returns
// the union of all row supports.  Decompose() splits it into per-clique
// LinearConstraint assemblers.
class SparseLinearConstraintAssembler : public SupernodalAssemblerBase {
 public:
  SparseLinearConstraintAssembler(
      std::unique_ptr<SparseLinearConstraint> slc,
      const std::vector<int>& all_variables);

  std::vector<std::vector<int>> get_cliques() const override {
    return {slc_->row_supports().begin(), slc_->row_supports().end()};
  }

  std::vector<SupernodalAssemblerBase*> Decompose(
      const std::vector<std::vector<int>>& maximal_cliques) override;

  // Decompose into raw (A, b, variables) blocks without creating
  // LinearConstraint objects or allocating workspace.  Each returned
  // RowGroup corresponds to one maximal clique.
  std::vector<SparseLinearConstraint::RowGroup> DecomposeRaw(
      const std::vector<std::vector<int>>& maximal_cliques) const;

  // Access decomposed constraints (available after Decompose()).
  const std::vector<std::unique_ptr<LinearConstraint>>& constraints() const {
    return owned_constraints_;
  }

  void set_precompute_gram(bool v) override {
    for (auto& c : owned_constraints_) {
      c->set_precompute_gram(v);
    }
  }

  // Row mapping: global row index → (constraint index, local row).
  // Available after Decompose().
  struct RowMapping {
    int constraint_index;
    int local_row;
  };
  const std::vector<RowMapping>& row_map() const { return row_map_; }
  int num_global_rows() const { return num_global_rows_; }

  // Set per-row weights from a global weight vector (size = A.rows()).
  // Distributes to per-clique W vectors and updates each GramEvaluator.
  void SetWeights(const Eigen::VectorXd& weights);

  // Compute residuals r = A x - b per-clique, returned as a global vector.
  // x is the solution in original variable order (size = num columns of A).
  // Returns a vector of size num_global_rows_.
  Eigen::VectorXd ComputeResiduals(const Eigen::VectorXd& x) const;

  // Compute residuals using the block partition from a solved system.
  // The solver must have been used to solve (so the partition is populated).
  // Uses A_perm_ directly on contiguous supernode/separator blocks — no gather.
  Eigen::VectorXd ComputeBlockResiduals(
      const class SymmetricLinearSystemTreeSolver& solver) const;

  // Compute A^T * v per-clique, returned as a global vector of size n.
  // v is a per-row vector (size = num_global_rows_).
  Eigen::VectorXd ComputeTransposeProduct(const Eigen::VectorXd& v) const;

 private:
  std::unique_ptr<SparseLinearConstraint> slc_;

  // Owned storage for decomposed constraints.
  std::vector<std::unique_ptr<LinearConstraint>> owned_constraints_;
  // Persistent workspace memory for each LinearConstraint's WorkspaceLinear.
  std::list<Eigen::VectorXd> owned_workspace_memory_;
  // Global row → per-clique mapping (indexed by global row).
  std::vector<RowMapping> row_map_;
  int num_global_rows_ = 0;
};

struct SparseLeastSquaresResult {
  Eigen::VectorXd x;
  double construction_time_us;
  double assemble_and_factor_time_us;
  double solve_time_us;

  // Sub-phase breakdown of construction_time_us:
  double grouping_us;
  double add_constraints_us;
  double init_workspace_us;
  double clique_extraction_us;
  double finalize_us;
};

SparseLeastSquaresResult SparseLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

}  // namespace conex
