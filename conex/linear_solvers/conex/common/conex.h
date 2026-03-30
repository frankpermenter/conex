#pragma once

namespace conex {

enum : int {
  CONEX_KKT_SOLVER_TREE = 3,
};

struct TreeSolverOptions {
  bool left_looking = true;
  // Use DynamicSubsystem for all cliques (LLT+RLDLT/LU runtime dispatch).
  // When false, positive-definite cliques use LLTSolver (in-place LLT).
  bool use_generic_factorization = false;
  // Use LU instead of RLDLT for indefinite cliques.
  bool use_lu_for_indefinite = false;
  int supernode_reorder_method = 0;  // SUPERNODE_REORDER_BFS_GREEDY
  int max_merge_supernode_size = 5;
};

struct SolverConfiguration {
  // Number of CPU threads used by compatible KKT solvers.
  // Set to 1 for single-threaded execution.
  int num_threads = 1;

  // Number of RHS columns to pre-allocate solve workspace for.
  // Set to the expected column count to avoid reallocation on first Solve().
  int rhs_cols = 1;

  // Use quotient AMD: run weighted min-degree on the constraint graph
  // (one node per constraint, edge weight = shared variables) instead of
  // variable-level AMD on the full KKT matrix.  Faster for problems
  // with known block structure.
  bool use_quotient_amd = false;

  TreeSolverOptions tree;
};

}  // namespace conex
