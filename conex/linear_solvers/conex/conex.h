#pragma once

namespace conex {

enum : int {
  CONEX_KKT_SOLVER_TREE = 3,
};

struct TreeSolverOptions {
  bool precompute_gram = false;
  bool left_looking = true;
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

  TreeSolverOptions tree;
};

}  // namespace conex
