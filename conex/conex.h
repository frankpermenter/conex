#pragma once
#include <string>

namespace conex {

enum : int {
  CONEX_INITIALIZATION_MODE_COLDSTART = 0,
  CONEX_INITIALIZATION_MODE_WARMSTART = 1,
  CONEX_KKT_SOLVER_SUPERNODAL = 0,
  CONEX_KKT_SOLVER_SUPERNODAL_QR = 1,
  CONEX_KKT_SOLVER_CG = 2,
  CONEX_KKT_SOLVER_TREE = 3,
  CONEX_KKT_SOLVER_SPARSE_QR = 4,
  CONEX_CLIQUE_TREE_METHOD_WEIGHTED_DFS = 0,
  CONEX_CLIQUE_TREE_METHOD_AMD = 1,
  CONEX_ALGORITHM_INFEASIBLE_START = 0,
  CONEX_ALGORITHM_SELF_DUAL_EMBEDDING = 1,
  CONEX_STEP_TYPE_DUAL_BARRIER = 0,
  CONEX_STEP_TYPE_PRIMAL_BARRIER = 1,
  CONEX_STEP_TYPE_GEODESIC = 2,
};

struct SupernodalSolverOptions {
  int iterative_refinement_iterations = 0;
};

struct TreeSolverOptions {
  bool precompute_gram = false;
  bool left_looking = true;
};

struct SolverConfiguration {
  int prepare_dual_variables = 0;
  int initialization_mode = 0;
  // TODO(FrankPermenter): Remove inv_sqrt_mu_max
  double inv_sqrt_mu_max = 1000;
  double minimum_mu = 1e-15;
  double maximum_mu = 1e4;
  double divergence_upper_bound = 1;
  int enable_line_search = 0;
  double dinf_upper_bound = 1;
  int final_centering_steps = 5;
  double final_centering_tolerance = 1;
  int initial_centering_steps_warmstart = 0;
  int initial_centering_steps_coldstart = 0;
  double warmstart_abort_threshold = 2;
  int max_iterations = 25;
  double infeasibility_threshold = 1e5;
  double kkt_error_tolerance = 1e10;
  int kkt_solver = CONEX_KKT_SOLVER_SUPERNODAL;
  int clique_tree_method = CONEX_CLIQUE_TREE_METHOD_AMD;
  int enable_rescaling = 1;
  // Number of CPU threads used by compatible KKT solvers.
  // Set to 1 for single-threaded execution.
  int num_threads = 1;
  int verbose = 1;
  bool enable_logging = false;
  bool enable_scale_correction = false;
  int algorithm = CONEX_ALGORITHM_INFEASIBLE_START;
  int step_type = CONEX_STEP_TYPE_GEODESIC;
  std::string log_file = "conex_log.json";

  bool record_kkt_timings = false;

  SupernodalSolverOptions supernodal;
  TreeSolverOptions tree;
};

struct ConexStatus {
  int solved = 0;
  int num_iterations;
  int primal_infeasible = 0;
  int dual_infeasible = 0;
  double dual_objective_value = 0;
  double primal_objective_value = 0;
  double complementarity = 0;
  double dinf = 0;
};

}  // namespace conex
