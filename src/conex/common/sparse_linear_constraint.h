#pragma once
#include <list>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

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
  const Eigen::VectorXd& b() const { return b_; }

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
class SparseLinearConstraintAssembler : public CliqueProvider {
 public:
  SparseLinearConstraintAssembler(
      std::unique_ptr<SparseLinearConstraint> slc,
      const std::vector<int>& all_variables);

  std::vector<std::vector<int>> get_cliques() const override;

  // Access underlying data (e.g. for Preprocess).
  const Eigen::SparseMatrix<double>& sparse_matrix() const { return slc_->A(); }
  const Eigen::VectorXd& rhs_vector() const { return slc_->b(); }

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

  // Gather a RowSpace back to a dense vector in original constraint row order.
  // segments[i] is the RowSpace segment for the i-th sub-constraint.
  // segments_offset is the index of the first sub-constraint's segment
  // in the RowSpace.
  Eigen::VectorXd GatherRows(const RowSpace& rs, int segment_offset) const;

  // Number of rows in the original (pre-decomposition) constraint.
  int num_global_rows() const { return num_global_rows_; }

 protected:
  // Factory for creating per-clique constraints.  Subclasses override to
  // create specialized constraint types (e.g. PSDLinearConstraint).
  virtual std::unique_ptr<LinearConstraint> MakeConstraint(
      const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

 private:
  std::unique_ptr<SparseLinearConstraint> slc_;

  // Owned storage for decomposed constraints.
  std::vector<std::unique_ptr<LinearConstraint>> owned_constraints_;
  // Persistent workspace memory for each LinearConstraint's WorkspaceLinear.
  std::list<Eigen::VectorXd> owned_workspace_memory_;
  // Global row → per-clique mapping (indexed by global row).
  struct RowMapping {
    int constraint_index;
    int local_row;
  };
  std::vector<RowMapping> row_map_;
  int num_global_rows_ = 0;
};

}  // namespace conex

// For backward compatibility — types moved to algorithms/least_squares.h.
#include "conex/algorithms/least_squares.h"
