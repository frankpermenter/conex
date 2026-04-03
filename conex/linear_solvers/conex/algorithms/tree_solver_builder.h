// TODO: Merge TreeSolverBuilder and ConstraintManager into a single class.
// Both collect assemblers and produce a SymmetricLinearSystemTreeSolver.
// ConstraintManager adds Preprocess (structural rank reduction, row dropping)
// and automatic dual variable allocation.  TreeSolverBuilder adds explicit
// tree specification, quotient AMD, per-clique block registration, and
// arena allocation.  A unified class would support both the automatic path
// (sparse matrices in, clique ordering discovers structure) and the
// structured path (user specifies tree topology) through a single API.
#pragma once
#include <Eigen/Dense>
#include <list>
#include <memory>
#include <set>
#include <vector>

#include <Eigen/Sparse>

#include "conex/common/clique_ordering.h"
#include "conex/common/equality_constraint.h"
#include "conex/common/linear_constraint.h"
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

  // Add a dense linear constraint A x = b to a clique.
  // Assembles A'A (positive definite) on vars.
  void AddLinearConstraint(int clique, const Eigen::MatrixXd& A,
                           const Eigen::VectorXd& b,
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
    int num_cliques;
    int max_clique_size;  // max(|sn| + |sep|) over all cliques
    long long fill;       // Σ (|sn| + |sep|)^2
    // Opaque storage for assemblers/matrices that must outlive the solver.
    struct Storage;
    std::unique_ptr<Storage> storage_;
    Result();
    ~Result();
    Result(Result&&) noexcept;
    Result& operator=(Result&&) noexcept;
  };

  // Enable running intersection property (RIP) validation in Build().
  // When enabled, Build() checks that every variable appearing in a clique
  // and any ancestor also appears in every intermediate clique on the path.
  // Off by default (O(V * depth) cost).
  void EnableRIPCheck(bool enable = true) { check_rip_ = enable; }

  // NOTE: Auto-tree mode (all parents = -1) uses weighted AMD on the
  // quotient graph of clique intersections to compute an elimination tree.
  // This works well for positive-definite systems but may produce suboptimal
  // orderings for indefinite KKT systems where the AMD can eliminate "hub"
  // cliques too early, placing cost data in separator blocks.  For
  // structured problems (LQR, stochastic opt), prefer the explicit tree.

  // Validate and build the tree solver.
  // Computes supernodes/separators, creates adapters, calls FinalizeStructure.
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
  bool check_rip_ = false;

  // Owned assembler storage (std::list for pointer stability).
  std::list<QuadraticConstraint> cost_assemblers_;
  std::list<LinearConstraint> linear_assemblers_;
  std::list<SupernodalAssemblerEqualities> eq_assemblers_;

  // Arena memory for LinearConstraint workspaces (allocated in Build).
  std::vector<double> workspace_arena_;

  // If all parents are -1, compute an elimination ordering via weighted
  // AMD on the quotient graph, then run symbolic elimination on the
  // variable graph to get proper fill and maximal cliques.
  // Returns the EliminationOrdering for use with
  // MakeCliqueTreeFromEliminationOrdering.
  EliminationOrdering ComputeQuotientAMDOrdering();
};

}  // namespace conex
