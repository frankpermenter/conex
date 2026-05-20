#pragma once
#include <functional>
#include <memory>

namespace conex {

class KKTSubsystemBase;

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
  // Use LAPACK dsytrf (Bunch-Kaufman) for indefinite cliques.
  // Returns zero-pivot index on failure for targeted demotion.
  bool use_lapack_for_indefinite = false;
  int supernode_reorder_method = 0;  // SUPERNODE_REORDER_BFS_GREEDY
  int max_merge_supernode_size = 5;
  // Optional factory for custom subsystem types (e.g., CholeskySkipZero).
  // If set, all non-indefinite cliques use this instead of LLT.
  std::function<std::unique_ptr<KKTSubsystemBase>()> subsystem_factory;
};

struct SolverConfiguration {
  // Number of CPU threads used by compatible KKT solvers.
  // Set to 1 for single-threaded execution.
  int num_threads = 1;

  // Number of RHS columns to pre-allocate solve workspace for.
  // Default is 3 (needed by ComputeFullDecomposition's 3-column solve).
  int rhs_cols = 3;

  // Use quotient AMD: run weighted min-degree on the constraint graph
  // (one node per constraint, edge weight = shared variables) instead of
  // variable-level AMD on the full KKT matrix.  Faster for problems
  // with known block structure.
  bool use_quotient_amd = false;

  // Row-scale linear constraints so that b_i ≈ 1.  Improves conditioning
  // for problems with large b values (e.g., bound constraints with big bounds).
  bool row_scale = false;

  // If > 0, lift equality constraints Cx=d into a quadratic penalty
  // alpha*|Cx-d|^2 in the objective, removing all equality constraints.
  // Post-solve, equality duals are reconstructed as nu = alpha*(Cx-d).
  // Use with HSDE algorithm (solve_raw("hsde")) for best results.
  double penalty_alpha = 0;

  TreeSolverOptions tree;
};

}  // namespace conex
