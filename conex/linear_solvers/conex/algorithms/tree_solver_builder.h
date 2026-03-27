#pragma once
#include <Eigen/Dense>
#include <list>
#include <memory>
#include <set>
#include <vector>

#include "conex/common/equality_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {

// Builder for constructing a tree solver with a known clique tree structure,
// bypassing automatic clique ordering.
//
// Usage:
//   TreeSolverBuilder b;
//   int root = b.AddClique();              // root (parent = -1)
//   int child = b.AddClique(root);         // child of root
//   b.AddCost(child, Q, {0, 1, 2});       // PD cost block
//   b.AddEquality(child, C, d, {0,1,3}, {4,5});  // indefinite block
//   b.AddCost(root, Qf, {3});
//   auto result = b.Build();
//   result.solver->AssembleAndFactor();
//   auto x = result.solver->Solve(rhs);
class TreeSolverBuilder {
 public:
  // Add a clique.  parent = -1 for root, otherwise a valid clique id.
  // Returns the new clique's id.
  int AddClique(int parent = -1);

  // Add a positive-definite cost block Q to a clique.
  // vars = variable indices that Q operates on.
  void AddCost(int clique, const Eigen::MatrixXd& Q,
               const std::vector<int>& vars);

  // Add an indefinite equality constraint block to a clique.
  // Assembles [0, C'; C, 0] on (primal_vars, dual_vars).
  void AddEquality(int clique, const Eigen::MatrixXd& C,
                   const Eigen::VectorXd& d,
                   const std::vector<int>& primal_vars,
                   const std::vector<int>& dual_vars);

  struct Result {
    std::unique_ptr<SymmetricLinearSystemTreeSolver> solver;
    int num_variables;
  };

  // Validate and build the tree solver.
  // Computes supernodes/separators, creates adapters, calls Finalize.
  Result Build();

 private:
  struct CliqueInfo {
    int parent;
    std::vector<int> children;
    std::set<int> all_vars;
  };
  std::vector<CliqueInfo> cliques_;

  struct PendingAdapter {
    SupernodalAssemblerBase* assembler;
    int clique;
    ContributionType type;
  };
  std::vector<PendingAdapter> pending_;

  // Owned assembler storage (std::list for pointer stability).
  std::list<DenseQuadraticTermSubAssembler> cost_assemblers_;
  std::list<SupernodalAssemblerEqualities> eq_assemblers_;
};

}  // namespace conex
