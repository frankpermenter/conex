#pragma once
namespace conex {

enum : int {
  CONEX_INITIALIZATION_MODE_COLDSTART = 0,
  CONEX_INITIALIZATION_MODE_WARMSTART = 1,
  CONEX_KKT_SOLVER_LLT = 0,
  CONEX_KKT_SOLVER_LDLT = 1,
  CONEX_KKT_SOLVER_QR = 2,
  CONEX_KKT_SOLVER_CG = 3,
  CONEX_KKT_SOLVER_TREE = 4,
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
  double final_centering_tolerance = .01;
  int initial_centering_steps_warmstart = 0;
  int initial_centering_steps_coldstart = 0;
  double warmstart_abort_threshold = 2;
  int max_iterations = 25;
  double infeasibility_threshold = 1e5;
  double kkt_error_tolerance = 1e10;
  int kkt_solver = CONEX_KKT_SOLVER_TREE;
  int enable_rescaling = 1;
  int iterative_refinement_iterations = 0;
  int verbose;
};

struct ConexStatus {
  int solved = 0;
  int num_iterations;
  int primal_infeasible = 0;
  int dual_infeasible = 0;
};

}  // namespace conex
