#pragma once
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

class Program;

// Decomposes a sparse linear inequality Ax <= b into groups of rows
// with identical column support.  Each group becomes a separate
// LinearConstraint with its own variable subset, enabling the tree
// solver to exploit sparsity.
class SparseLinearConstraint {
 public:
  SparseLinearConstraint(const Eigen::SparseMatrix<double>& A,
                         const Eigen::VectorXd& b);

  // Add all sub-constraints to the program.
  // Returns the constraint IDs.
  std::vector<int> AddToProgram(Program& prog);

  int num_groups() const { return groups_.size(); }

  struct RowGroup {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::vector<int> variables;
  };

  const std::vector<RowGroup>& groups() const { return groups_; }

 private:
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

SparseLeastSquaresResult SparseLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

}  // namespace conex
