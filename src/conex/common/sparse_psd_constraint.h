// Sparse PSD constraint assembler with chordal decomposition.
//
// Given Σ A_i x_i + B ≽ 0 with sparse n×n matrices, computes the
// aggregate sparsity pattern, finds a chordal covering (maximal cliques
// via min-degree elimination), and creates one PSD sub-constraint per
// matrix-space clique.  Each sub-block reports its optimization-variable
// footprint to the KKT tree builder (like SparseQuadraticTermAssembler).

#pragma once
#include <list>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "conex/common/linear_constraint.h"
#include "conex/common/supernodal_assembler_base.h"

namespace conex {

class SparsePSDConstraintAssembler : public CliqueProvider {
 public:
  // A_list[k] is n×n, B is n×n, vars[k] is the optimization variable
  // for A_list[k].  The constraint is Σ A_k x_{vars[k]} + B ≽ 0.
  // If use_chordal is false, no chordal decomposition is performed:
  // the full n×n matrix is treated as a single PSD constraint.
  SparsePSDConstraintAssembler(
      const std::vector<Eigen::SparseMatrix<double>>& A_list,
      const Eigen::SparseMatrix<double>& B,
      const std::vector<int>& vars,
      bool use_chordal = true);

  // Returns one clique per matrix sub-block: the set of optimization
  // variables whose A_i has nonzero entries in that sub-block.
  std::vector<std::vector<int>> get_cliques() const override;

  std::vector<SupernodalAssemblerBase*> Decompose(
      const std::vector<std::vector<int>>& maximal_cliques) override;

  // Access decomposed constraints (available after Decompose()).
  const std::vector<std::unique_ptr<ConeConstraint>>& constraints() const {
    return owned_constraints_;
  }

 private:
  // A matrix-space clique from the chordal decomposition.
  struct MatrixClique {
    std::vector<int> indices;       // subset of {0,...,n-1}
    std::vector<int> var_indices;   // which A_list entries are active
    std::vector<int> opt_vars;      // global optimization variable indices
  };

  std::vector<Eigen::SparseMatrix<double>> A_list_;
  Eigen::SparseMatrix<double> B_;
  int n_;  // matrix dimension
  std::vector<MatrixClique> matrix_cliques_;

  // Owned storage for decomposed constraints.
  std::vector<std::unique_ptr<ConeConstraint>> owned_constraints_;
  std::list<Eigen::VectorXd> owned_workspace_memory_;
};

}  // namespace conex
