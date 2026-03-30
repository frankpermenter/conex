#include "conex/algorithms/irls.h"

#include <chrono>
#include <cmath>
#include <numeric>

#include "conex/common/problem.h"
#include "conex/common/solver.h"

namespace conex {

IRLSResult SolveIRLS(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    int max_iterations,
    double epsilon,
    double tolerance) {
  using clock = std::chrono::high_resolution_clock;
  IRLSResult result;
  const int m = A.rows();
  const int n = A.cols();

  // Build problem.
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  auto c = problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(m), vars);

  // Preprocess (drop structurally dependent columns).
  auto [reduced, expansion] = Preprocess(problem);
  const int n_solve = reduced.num_variables();

  // Build solver once.
  auto solver = Solver::Build(reduced);
  solver.AssembleAndFactor();

  auto t0 = clock::now();

  Eigen::VectorXd weights = Eigen::VectorXd::Ones(m);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n_solve);
  double prev_obj = std::numeric_limits<double>::max();

  // Dense A in reduced space for residual/transpose computation.
  Eigen::MatrixXd A_reduced(A);
  if (expansion.was_reduced()) {
    A_reduced.resize(m, n_solve);
    for (int j = 0; j < n_solve; ++j)
      A_reduced.col(j) = Eigen::MatrixXd(A).col(expansion.col_map[j]);
  }

  for (int iter = 0; iter < max_iterations; ++iter) {
    solver.SetWeights(c, weights);
    if (!solver.AssembleAndFactor()) break;

    // RHS = A^T W b.
    Eigen::VectorXd rhs = A_reduced.transpose() * (weights.asDiagonal() * b);
    x = solver.Solve(rhs);

    // Residual: r = A x - b.
    Eigen::VectorXd r = A_reduced * x - b;
    double obj = r.lpNorm<1>();

    if (std::abs(prev_obj - obj) < tolerance * std::abs(obj) + 1e-15) {
      result.iterations = iter + 1;
      break;
    }
    prev_obj = obj;
    result.iterations = iter + 1;

    // IRLS weight update.
    for (int i = 0; i < m; ++i)
      weights(i) = 1.0 / std::max(std::abs(r(i)), epsilon);
  }

  auto t1 = clock::now();
  result.x = expansion.Expand(x);
  result.l1_objective = (A_reduced * x - b).lpNorm<1>();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  return result;
}

}  // namespace conex
