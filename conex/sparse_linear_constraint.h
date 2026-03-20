#pragma once
#include <list>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/constraint.h"
#include "conex/linear_constraint.h"

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
  const std::vector<std::vector<int>>& row_supports() const {
    return unique_supports_;
  }

  // Given a list of target supports (e.g. maximal cliques), assign each
  // row to the smallest target that contains its support, then build
  // dense sub-blocks.
  std::vector<RowGroup> GetConstraints(
      const std::vector<std::vector<int>>& target_supports) const;

  // Add all sub-constraints to the program using containment-merged groups.
  std::vector<int> AddToProgram(Program& prog);

  int num_groups() const { return groups().size(); }
  const std::vector<RowGroup>& groups() const;

 private:
  const Eigen::SparseMatrix<double>& A_;
  Eigen::VectorXd b_;

  struct SupportGroup {
    std::vector<int> support;
    std::vector<int> rows;
  };
  std::vector<SupportGroup> support_groups_;
  std::vector<std::vector<int>> unique_supports_;

  // Containment-merged groups (lazily computed on first access).
  mutable std::vector<RowGroup> groups_;
};

// Assembler that wraps a SparseLinearConstraint.  Its variables() returns
// the union of all row supports.  Decompose() splits it into per-clique
// LinearConstraint assemblers.
class SparseLinearConstraintAssembler : public SupernodalAssemblerBase {
 public:
  SparseLinearConstraintAssembler(
      std::unique_ptr<SparseLinearConstraint> slc,
      const std::vector<int>& all_variables);

  void SetDenseData() override {
    throw std::runtime_error(
        "SparseLinearConstraintAssembler requires the tree solver path "
        "(CONEX_KKT_SOLVER_TREE). Use Decompose() instead of SetDenseData().");
  }

  std::vector<std::vector<int>> get_cliques() const override {
    return {slc_->row_supports().begin(), slc_->row_supports().end()};
  }

  std::vector<SupernodalAssemblerBase*> Decompose() override;

  std::vector<SupernodalAssemblerBase*> Decompose(
      const std::vector<std::vector<int>>& maximal_cliques) override;

  void RegisterDecomposedConeInequalities(ConstraintManager* cm) override;

  void set_precompute_gram(bool v) override {
    for (auto& c : owned_constraints_) {
      c->set_precompute_gram(v);
    }
  }

 private:
  std::unique_ptr<SparseLinearConstraint> slc_;

  // Owned storage for decomposed constraints and their assemblers.
  std::vector<std::unique_ptr<LinearConstraint>> owned_constraints_;
  std::list<SupernodalAssemblerConstraint> owned_assemblers_;
  // Persistent workspace memory for each LinearConstraint's WorkspaceLinear.
  std::list<Eigen::VectorXd> owned_workspace_memory_;
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

// Containment-grouping path.
SparseLeastSquaresResult SparseLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

// Maximal-clique grouping path.
SparseLeastSquaresResult SparseLeastSquaresMaximalClique(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

// MakeTreeSolver path: uses SparseLinearConstraintAssembler + Decompose.
SparseLeastSquaresResult SparseLeastSquaresMakeTreeSolver(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

}  // namespace conex
