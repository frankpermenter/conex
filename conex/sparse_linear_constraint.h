#pragma once
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

class Program;

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
  };

  // The unique row supports of A (sorted, deduplicated).
  // Each entry is a sorted vector of column indices.
  const std::vector<std::vector<int>>& row_supports() const {
    return unique_supports_;
  }

  // Given a list of target supports (e.g. maximal cliques), assign each
  // row to the smallest target that contains its support, then build
  // dense sub-blocks.  Each target support becomes one RowGroup (skipped
  // if no rows map to it).
  std::vector<RowGroup> GetConstraints(
      const std::vector<std::vector<int>>& target_supports) const;

  // Add all sub-constraints to the program using containment-merged groups.
  // (Legacy interface — uses GetConstraints with containment merging.)
  std::vector<int> AddToProgram(Program& prog);

  int num_groups() const { return groups_.size(); }
  const std::vector<RowGroup>& groups() const { return groups_; }

 private:
  const Eigen::SparseMatrix<double>& A_;
  Eigen::VectorXd b_;

  // Per unique support: the support itself and the original row indices.
  struct SupportGroup {
    std::vector<int> support;
    std::vector<int> rows;
  };
  std::vector<SupportGroup> support_groups_;
  std::vector<std::vector<int>> unique_supports_;

  // Legacy containment-merged groups (built in constructor).
  std::vector<RowGroup> groups_;
};

struct SparseLeastSquaresResult {
  Eigen::VectorXd x;
  double construction_time_us;
  double assemble_and_factor_time_us;
  double solve_time_us;

  // Sub-phase breakdown of construction_time_us:
  double grouping_us;          // SparseLinearConstraint constructor
  double add_constraints_us;   // ConstraintManager::AddConstraint calls
  double init_workspace_us;    // InitializeWorkspace + SetIdentity
  double clique_extraction_us; // get_cliques + MakeCliqueTree
  double finalize_us;          // push adapters + Finalize + mode setup
};

// Containment-grouping path.
SparseLeastSquaresResult SparseLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

// Maximal-clique grouping path.
SparseLeastSquaresResult SparseLeastSquaresMaximalClique(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

}  // namespace conex
