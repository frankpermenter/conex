#include "conex/cone_program.h"
#include "conex/conex.h"
#include "conex/debug_macros.h"
#include "conex/linear_constraint.h"
#include "conex/quadratic_cone_constraint.h"
#include "conex/soc_constraint.h"

namespace conex {

void AddSqrtQuadraticCostEpigraph(conex::Program* conex_prog,
                                  const Eigen::MatrixXd& Qsqrt,
                                  const std::vector<int>& z, int epigraph) {
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

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct ProblemData {
  Eigen::MatrixXd W;
  Eigen::MatrixXd Wsqrt;
  Eigen::VectorXd c;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};

MatrixXd NormalizeRows(const MatrixXd& X) {
  MatrixXd Y = X;
  for (int i = 0; i < X.rows(); i++) {
    Y.row(i) = X.row(i) / X.row(i).norm();
  }
  Y = Y / sqrt((double)X.rows());
  return Y;
}

ProblemData RandomWellPosedProblem(int n, int num_ineqs,
                                   int rank_of_quadratic) {
  if (rank_of_quadratic + num_ineqs < n) {
    throw std::runtime_error(
        "Must have rank of quadratic + num_ineqs >= num_vars");
  }
  if (rank_of_quadratic > n) {
    throw std::runtime_error("Must have rank of quadratic <= num_vars");
  }

  ProblemData data;

  data.A = NormalizeRows(MatrixXd::Random(num_ineqs, n));
  if (rank_of_quadratic > 0) {
    data.Wsqrt = NormalizeRows(MatrixXd::Random(rank_of_quadratic, n));
    data.W = data.Wsqrt.transpose() * data.Wsqrt;
  } else {
    data.Wsqrt = MatrixXd::Random(1, n) * 0;
    data.W = data.Wsqrt.transpose() * data.Wsqrt;
  }

  VectorXd strictly_feasible_slack(num_ineqs);
  strictly_feasible_slack.setConstant(1);
  VectorXd strictly_feasible_lambda(num_ineqs);
  strictly_feasible_lambda.setConstant(1);
  strictly_feasible_slack += VectorXd::Random(num_ineqs) * .1 / sqrt(num_ineqs);
  strictly_feasible_lambda +=
      VectorXd::Random(num_ineqs) * .1 / sqrt(num_ineqs);

  VectorXd feasible_x = VectorXd::Random(n);
  feasible_x = feasible_x / feasible_x.norm();

  data.b = strictly_feasible_slack - (data.A * feasible_x);
  data.b.array();
  data.c = data.A.transpose() * strictly_feasible_lambda - data.W * feasible_x;

  return data;
}

ProblemData ProblemDataFromSolution(int n, int num_ineqs) {
  ProblemData data;
  int size_of_active_set = n;

  VectorXd optimal_slack(num_ineqs);
  optimal_slack.setZero();
  VectorXd optimal_lambda(num_ineqs);
  optimal_lambda.setZero();
  optimal_lambda.head(size_of_active_set) =
      VectorXd::Random(size_of_active_set).array().abs();
  optimal_slack.tail(num_ineqs - size_of_active_set) =
      VectorXd::Random(num_ineqs - size_of_active_set).array().abs();

  optimal_lambda.head(size_of_active_set)
      .setLinSpaced(size_of_active_set, 1, size_of_active_set);
  optimal_slack.tail(num_ineqs - size_of_active_set).setConstant(1);

  data.A = MatrixXd::Random(num_ineqs, n);
  data.b = optimal_slack;
  data.c = data.A.transpose() * optimal_lambda;
  data.W = MatrixXd::Zero(n, n);

  return data;
}

struct Stats {
  int num_iter;
  double complementarity;
};
ConexStatus SolveQPInstance(ProblemData& data, const SolverConfiguration& config,
                    bool use_epigraph, bool print_stats = false) {
  int num_vars = data.A.cols();
  Program prog(num_vars + use_epigraph);
  VectorXd solution(num_vars + use_epigraph);

  std::vector<int> vars;
  for (int i = 0; i < num_vars; i++) {
    vars.push_back(i);
  }

  if (!use_epigraph) {
    prog.AddQuadraticCost(data.W, vars);
    prog.AddLinearCost(data.c);
  } else {
    AddSqrtQuadraticCostEpigraph(&prog, data.Wsqrt, vars, vars.size());
    VectorXd linear_cost(num_vars + 1);
    linear_cost.head(num_vars) = data.c;
    linear_cost(num_vars) = 0;
    prog.AddLinearCost(linear_cost);
  }
  // Ax <= b.
  prog.AddConstraint(LinearConstraint(-data.A, data.b), vars);

  bool solved = Solve(prog, config, solution.data());
  if (!solved) {
    throw std::runtime_error("Failed to solve");
  }
  VectorXd optimal_x = solution.head(num_vars);
  return prog.Status();
}

struct Statistics {
  int average_iter_epigraph;
  int average_iter_qp;
};

}  // namespace conex

struct ExperimentalSetup {
  int num_trial;
  int num_vars;
  int num_ineqs;
  int rank_of_quadratic;
  bool use_geodesic_updates = true;
};

conex::Statistics DoCompare(const ExperimentalSetup& setup) {
  int num_trial = setup.num_trial;
  int num_vars = setup.num_vars;
  int num_ineqs = setup.num_ineqs;
  int rank_of_quadratic = setup.rank_of_quadratic;

  std::cout << "\n";
  std::cout << " Num instances: " << num_trial;
  std::cout << " Num Vars: " << num_vars;
  std::cout << " Num Ineq: " << num_ineqs;
  std::cout << " Rank: " << rank_of_quadratic;
  double average_iter_no_rescaling = 0;
  double average_iter_with_rescaling = 0;

  double average_gap_no_rescaling = 0;
  double average_gap_with_rescaling = 0;

  conex::SolverConfiguration config;
  config.enable_line_search = true;
  config.enable_rescaling = false;
  config.inv_sqrt_mu_max = 1e5;
  config.maximum_mu = 1e9;
  config.final_centering_tolerance = 1;
  config.max_iterations = 50;
  config.kkt_error_tolerance = 1e30;
  config.verbose = false;
  config.dinf_upper_bound = 1;
  config.enable_scale_correction = true;

  conex::Statistics stats;
  conex::ConexStatus conex_stats;
  for (int i = 0; i < num_trial; i++) {
    config.step_type = conex::CONEX_STEP_TYPE_GEODESIC;
    srand(i);
    int num_iter = 0;
    conex::ProblemData data =
        conex::RandomWellPosedProblem(num_vars, num_ineqs, rank_of_quadratic);


    config.enable_scale_correction = true;
    conex_stats = conex::SolveQPInstance(data, config, false /*use_epigraph*/);
    average_iter_with_rescaling += 1.0 / (1 + i) * (conex_stats.num_iterations 
    - average_iter_with_rescaling);

    average_gap_with_rescaling += 1.0 / (1 + i) * (conex_stats.complementarity
    - average_gap_with_rescaling);


    config.enable_scale_correction = false;
    conex_stats = conex::SolveQPInstance(data, config, false /*use_epigraph*/);
    average_iter_no_rescaling += 1.0 / (1 + i) * (conex_stats.num_iterations - average_iter_no_rescaling);
    average_gap_no_rescaling += 1.0 / (1 + i) * (conex_stats.complementarity - average_gap_no_rescaling);
  }
  std::cout << " Avg Iter QP no rescaling: " << average_iter_no_rescaling;
  std::cout << " Avg Iter QP with rescaling: " << average_iter_with_rescaling;
  std::cout << " Avg Gap QP no rescaling: " << average_gap_no_rescaling;
  std::cout << " Avg Gap QP with rescaling: " << average_gap_with_rescaling;
  std::cout << "\n";
  return stats;
}

void CompareWithGeodesicIPM() {
  ExperimentalSetup setup;
  setup.num_trial = 1;
  setup.num_vars = 100;

  // Increase rank
  setup.num_ineqs = 75;
  setup.rank_of_quadratic = 25;
  DoCompare(setup);
  setup.rank_of_quadratic = 50;
  DoCompare(setup);
  setup.rank_of_quadratic = 75;
  DoCompare(setup);
  setup.rank_of_quadratic = 100;
  DoCompare(setup);

  //// Increase ineqs
  setup.rank_of_quadratic = 75;
  setup.num_ineqs = 25;
  DoCompare(setup);
  setup.num_ineqs = 50;
  DoCompare(setup);
  setup.num_ineqs = 75;
  DoCompare(setup);
  setup.num_ineqs = 100;
  DoCompare(setup);
}

int main() {
  CompareWithGeodesicIPM();
  return 0;
}
