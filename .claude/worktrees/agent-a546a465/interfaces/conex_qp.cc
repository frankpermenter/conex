#include <iostream>
#include <memory>
#include <vector>

#include "conex.h"
#include "conex/cone_program.h"
#include "conex/error_checking_macros.h"
#include "conex/linear_constraint.h"
#include "conex/self_dual_embedding.h"
#include "conex/soc_constraint.h"
#include <Eigen/Dense>

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

void AddSqrtQuadraticCostEpigraph(conex::Program* conex_prog,
                                  const Eigen::MatrixXd& Q,
                                  const std::vector<int>& z, int epigraph) {
  double eps = 0;
  Eigen::LLT<Eigen::MatrixXd> llt(
      Q + eps * Eigen::MatrixXd::Identity(Q.rows(), Q.rows()));
  Eigen::MatrixXd Qsqrt_transpose = llt.matrixL();
  Eigen::MatrixXd Qsqrt = Qsqrt_transpose.transpose();
  // Build (A, b) satisfying b - A(x, t) \in L <=> t >= 1/2 x^T Q x.
  int num_vars = z.size();
  Eigen::MatrixXd Ai(Qsqrt.rows() + 2, num_vars + 1);
  Eigen::MatrixXd b(Qsqrt.rows() + 2, 1);
  Ai.setZero();
  b.setZero();

  // (a t + k)^2 >= (a t - k)^2 + c * x^T Q x.
  // => 4 a k t >= c * x^T Q x.
  // => 2 a k / c t >= 1.0/2.0  x^T Q x.
  double c = 1.0 / (Qsqrt.transpose() * Qsqrt).squaredNorm();
  double a = std::sqrt(c);
  double k = 1.5;

  c = 1;
  a = 1;
  k = 0.5;

  Ai.topRightCorner(2, 1) << -a, -a;
  Ai.bottomLeftCorner(Qsqrt.rows(), Qsqrt.cols()) = Qsqrt * std::sqrt(c);
  b(0) = k;
  b(1) = -k;

  Eigen::VectorXd linear_cost(1);
  linear_cost << 2 * (a * k) / c;
  conex_prog->AddLinearCost(linear_cost, {epigraph});
  auto z_indices = z;
  z_indices.push_back(epigraph);
  conex_prog->AddConstraint(conex::SOCConstraint(Ai, b), z_indices);
}

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

struct ProblemData {
  Eigen::MatrixXd W;
  Eigen::VectorXd c;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::MatrixXd B;
  Eigen::VectorXd d;
  ProblemData() = default;
  ProblemData(int m, int n) : A(m, n), c(n), b(m), W(n, n) {}
};

}  // namespace

int CONEX_QP_GetCanonicalProblemData(
    const double* quadratic_cost_matrix, int num_row, int num_col,
    const double* cost_vector, int num_row_cost_vector,
    const double* inequality_matrix, int num_row_ineq, int num_col_ineq,
    const double* inequality_upper_bound, int num_row_ineq_ub,
    const double* inequality_lower_bound, int num_row_ineq_lb, int* num_ineq,
    int* num_eq, double* matrix_A, int num_row_A, int num_col_A,
    double* vector_b, int num_row_b, double* matrix_B, int num_row_B,
    int num_col_B, double* vector_d, int num_row_d) {
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
  ProblemData data;
  data.W = Map(quadratic_cost_matrix, num_row, num_col);
  data.c = Map(cost_vector, num_vars, 1);

  MatrixXd affine_term_ineq;
  MatrixXd affine_term_eq;
  MatrixXd ineq_mat = Map(inequality_matrix, num_row_ineq, num_vars);
  conex::PreprocessLinearInequality(
      Map(inequality_matrix, num_row_ineq, num_vars),
      Map(inequality_lower_bound, num_row_ineq_lb, 1),
      Map(inequality_upper_bound, num_row_ineq_ub, 1), &data.A,
      &affine_term_ineq, &data.B, &affine_term_eq, false);

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
  ProblemData data;
  data.W = Map(quadratic_cost_matrix, num_row, num_col);
  data.c = Map(cost_vector, num_vars, 1);

  MatrixXd affine_term_ineq;
  MatrixXd affine_term_eq;
  MatrixXd ineq_mat = Map(inequality_matrix, num_row_ineq, num_vars);
  conex::PreprocessLinearInequality(
      Map(inequality_matrix, num_row_ineq, num_vars),
      Map(inequality_lower_bound, num_row_ineq_lb, 1),
      Map(inequality_upper_bound, num_row_ineq_ub, 1), &data.A,
      &affine_term_ineq, &data.B, &affine_term_eq, 0);

  if (affine_term_ineq.rows() > 0) {
    data.b = affine_term_ineq;
    data.A.array() *= -1;
  }

  if (affine_term_eq.rows() > 0) {
    data.d = affine_term_eq;
  }

  bool use_epigraph = false;
  std::vector<int> vars(num_vars);
  std::iota(vars.begin(), vars.end(), 0);
  int num_vars_prog = num_vars;
  if (use_epigraph) {
    num_vars_prog++;
  }
  Program prg(num_vars_prog);

  if (data.A.rows() > 0) {
    prg.AddConstraint(conex::LinearConstraint(-data.A, data.b), vars);
  }
  if (data.B.rows() > 0) {
    prg.AddConstraint(conex::EqualityConstraints(data.B, data.d), vars);
  }

  if (use_epigraph) {
    AddSqrtQuadraticCostEpigraph(&prg, 0.5 * (data.W + data.W.transpose()),
                                 vars, num_vars_prog - 1);
  } else {
    prg.AddQuadraticCost(data.W, vars);
  }

  prg.AddLinearCost(data.c, vars);
  Eigen::Map<MatrixXd> sol_map(solution, num_vars, 1);
  SolverConfiguration config;
  config.enable_line_search = true;
  config.enable_rescaling = config_input->enable_rescaling;
  config.enable_scale_correction = config_input->enable_scale_correction;
  config.final_centering_tolerance = 1;
  config.final_centering_steps = 0;
  config.dinf_upper_bound = config_input->dinf_upper_bound;
  config.inv_sqrt_mu_max = config_input->inv_sqrt_mu_max;
  config.max_iterations = config_input->max_iterations;
  //    config.kkt_solver = conex::CONEX_KKT_SOLVER_SUPERNODAL_QR;
  config.enable_logging = true;
  sol_map = Solve(prg, config);

  stats->iterations = prg.statistics().num_iter;
  stats->duality_gap = prg.Status().complementarity;

  return CONEX_SUCCESS;
}
