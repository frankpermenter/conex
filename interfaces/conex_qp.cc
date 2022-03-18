#include "conex.h"

#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "conex/cone_program.h"
#include "conex/linear_constraint.h"

#include "conex/error_checking_macros.h"

#include "conex/quadratic_programs/logspace_interior_point_method.h"

// TODO(FrankPermenter): check for null pointers.
#define SAFER_CAST_TO_Program(x, prog)                                     \
  CONEX_DEMAND(x, "Program pointer is null.");                             \
  prog = static_cast<Program*>(x);                                         \
  if (prog->is_initialized) {                                              \
    if (prog->NumberOfConstraints() + 2 !=                                 \
        static_cast<int>(prog->workspaces.size())) {                       \
      CONEX_DEMAND(false, "Program corrupted or invalid pointer.");        \
    }                                                                      \
  } else {                                                                 \
    if (prog->workspaces.size() != 0 || prog->NumberOfConstraints() < 0) { \
      CONEX_DEMAND(false, "Program corrupted or invalid pointer.");        \
    }                                                                      \
  }                                                                        \
  CONEX_DEMAND(prog, "Program corrupted or invalid pointer.");

using DenseMatrix = Eigen::MatrixXd;
using conex::Program;
using conex::SolverConfiguration;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

namespace {

void NonZeroSubMat(const Eigen::MatrixXd& M, std::vector<int>* vars,
                   Eigen::MatrixXd* Y) {
  vars->clear();
  for (int i = 0; i < M.rows(); i++) {
    if (M(i, i) > 0) {
      vars->push_back(i);
    }
  }

  int row = 0;
  Y->resize(vars->size(), vars->size());
  for (auto& i : *vars) {
    int col = 0;
    for (auto& j : *vars) {
      (*Y)(row, col) = (M(i, j) + M(j, i)) / 2.0;
      (*Y)(col, row) = (*Y)(row, col);
      col++;
    }
    row++;
  }
}
SolverConfiguration APIConvertSolverConfiguration(
    const CONEX_SolverConfiguration* config) {
  SolverConfiguration c;
  c.prepare_dual_variables = config->prepare_dual_variables;
  c.initialization_mode = config->initialization_mode;
  c.inv_sqrt_mu_max = config->inv_sqrt_mu_max;
  c.minimum_mu = config->minimum_mu;
  c.maximum_mu = config->maximum_mu;
  c.divergence_upper_bound = config->divergence_upper_bound;
  c.enable_line_search = config->enable_line_search;
  c.dinf_upper_bound = config->dinf_upper_bound;
  c.final_centering_steps = config->final_centering_steps;
  c.final_centering_tolerance = config->final_centering_tolerance;
  c.initial_centering_steps_warmstart =
      config->initial_centering_steps_warmstart;
  c.initial_centering_steps_coldstart =
      config->initial_centering_steps_coldstart;
  c.warmstart_abort_threshold = config->warmstart_abort_threshold;
  c.max_iterations = config->max_iterations;
  c.iterative_refinement_iterations = config->iterative_refinement_iterations;
  c.infeasibility_threshold = config->infeasibility_threshold;
  c.kkt_error_tolerance = config->kkt_error_tolerance;
  c.enable_rescaling = config->enable_rescaling;
  c.kkt_solver = config->kkt_solver;
  return c;
}
}  // namespace

int CONEX_QP_GetCanonicalProblemData(const double* quadratic_cost_matrix,
                    int num_row,
                    int num_col,
                    const double* cost_vector, int num_row_cost_vector,
                    const double* inequality_matrix, int num_row_ineq,
                    int num_col_ineq, const double* inequality_upper_bound,
                    int num_row_ineq_ub, const double* inequality_lower_bound, int num_row_ineq_lb,
                    int *num_ineq,
                    int *num_eq,
                    double* matrix_A, int num_row_A, int num_col_A,
                    double* vector_b, int num_row_b,
                    double* matrix_B, int num_row_B, int num_col_B,
                    double* vector_d, int num_row_d) {

  CONEX_DEMAND(num_col == num_row, "Cost matrix must be square.");
  CONEX_DEMAND(num_row_cost_vector == num_col,
               "Cost vector and cost matrix must have same number of rows.");
  CONEX_DEMAND(num_row_ineq == num_row_ineq_ub,
               "Inequality matrix and upper bound vector must have same number "
               "of rows.");
  CONEX_DEMAND(num_row_ineq == num_row_ineq_lb,
               "Inequality matrix and lower bound vector must have same number "
               "of rows.");
  CONEX_DEMAND(
      num_col_ineq == num_col,
      "Inequality matrix and cost matrix must have same number of columns.");

  using Map = Eigen::Map<const MatrixXd>;
  int num_vars = num_col;
  conex::quadratic_programs::ProblemData data;
  data.W = Map(quadratic_cost_matrix, num_row, num_col);
  data.c = Map(cost_vector, num_vars, 1);

  MatrixXd affine_term_ineq;
  MatrixXd affine_term_eq;
  MatrixXd ineq_mat = Map(inequality_matrix, num_row_ineq, num_vars);
  conex::PreprocessLinearInequality(
      Map(inequality_matrix, num_row_ineq, num_vars),
      Map(inequality_lower_bound, num_row_ineq_lb, 1),
      Map(inequality_upper_bound, num_row_ineq_ub, 1), &data.A,
      &affine_term_ineq, &data.B, &affine_term_eq,
      false);

  if (affine_term_ineq.rows() > 0) {
    data.b = affine_term_ineq;
    data.A.array() *= -1;
  }

  if (affine_term_eq.rows() > 0) {
    data.d = affine_term_eq;
  }
  using MapOutput = Eigen::Map<MatrixXd>;
  if (data.A.rows() > 0) {
    MapOutput Aout(matrix_A, num_row_A, num_col_A); 
    Aout.topLeftCorner(data.A.rows(), data.A.cols()) = data.A;
    Eigen::Map<VectorXd> bout(vector_b, num_row_b); 
    bout.head(data.b.rows()) = data.b;
  }
  if (data.B.rows() > 0) {
    MapOutput Bout(matrix_B, num_row_B, num_col_B);
    Bout.topLeftCorner(data.B.rows(), data.B.cols()) = data.B;
    Eigen::Map<VectorXd> dout(vector_d, num_row_d); 
    dout.head(data.d.rows()) = data.d;
  }

  *num_eq = data.B.rows();
  *num_ineq = data.A.rows();
  return 0;
}

int CONEX_QP_Solver(const double* quadratic_cost_matrix, int num_row,
                    int num_col, const double* cost_vector, int num_row_cost,
                    const double* inequality_matrix, int num_row_ineq,
                    int num_col_ineq, const double* inequality_upper_bound,
                    int num_row_ineq_ub, const double* inequality_lower_bound,
                    int num_row_ineq_lb,
                    const CONEX_SolverConfiguration* config_input,
                    double* solution, int num_row_solution,
                    CONEX_SolutionStats* stats) {
  CONEX_DEMAND(num_col == num_row, "Cost matrix must be square.");
  CONEX_DEMAND(num_col == num_row_solution,
               "Output dimension does not equal number of variables.");
  CONEX_DEMAND(num_row_cost == num_col,
               "Cost vector and cost matrix must have same number of rows.");
  CONEX_DEMAND(num_row_ineq == num_row_ineq_ub,
               "Inequality matrix and upper bound vector must have same number "
               "of rows.");
  CONEX_DEMAND(num_row_ineq == num_row_ineq_lb,
               "Inequality matrix and lower bound vector must have same number "
               "of rows.");
  CONEX_DEMAND(
      num_col_ineq == num_col,
      "Inequality matrix and cost matrix must have same number of columns.");

  using Map = Eigen::Map<const MatrixXd>;
  int num_vars = num_col;
  conex::quadratic_programs::ProblemData data;
  data.W = Map(quadratic_cost_matrix, num_row, num_col);
  data.c = Map(cost_vector, num_vars, 1);

  MatrixXd affine_term_ineq;
  MatrixXd affine_term_eq;
  MatrixXd ineq_mat = Map(inequality_matrix, num_row_ineq, num_vars);
  conex::PreprocessLinearInequality(
      Map(inequality_matrix, num_row_ineq, num_vars),
      Map(inequality_lower_bound, num_row_ineq_lb, 1),
      Map(inequality_upper_bound, num_row_ineq_ub, 1), &data.A,
      &affine_term_ineq, &data.B, &affine_term_eq,
      config_input->enable_rescaling);


  if (affine_term_ineq.rows() > 0) {
    data.b = affine_term_ineq;
    data.A.array() *= -1;
  }

  if (affine_term_eq.rows() > 0) {
    data.d = affine_term_eq;
  }

  conex::quadratic_programs::SolverOptions config;
  config.enable_dynamic_regularization = config_input->enable_line_search;
  config.maximum_iterations = config_input->max_iterations;
  config.theta_weight = config_input->theta_weight;
  config.sqrt_mu_weight = config_input->sqrt_mu_weight;
  config.target_duality_gap = config_input->target_duality_gap;
  config.dinf_limit = config_input->dinf_upper_bound;
  config.minimum_mu = config_input->minimum_mu;
  config.enable_rescaling = config_input->enable_rescaling;
  config.theta_truncation_threshold = config_input->theta_truncation_threshold;
  config.endgame_rescaling_threshold = config_input->endgame_rescaling_threshold;
  config.endgame_rescaling_factor = config_input->endgame_rescaling_factor;
  config.verbosity = config_input->verbosity;

  auto sol = LogspaceIPM(data, config);
  Eigen::Map<MatrixXd> sol_map(solution, num_vars, 1);
  sol_map = sol.x.x;
  if (stats) {
    stats->iterations = sol.iterations;
  }
  return sol.status;
}
