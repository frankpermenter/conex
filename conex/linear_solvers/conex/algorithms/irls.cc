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

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  auto c = problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(m), vars);

  auto [reduced, expansion] = Preprocess(problem);
  const int n_solve = reduced.num_variables();

  auto solver = Solver::Build(reduced);
  solver.AssembleAndFactor();

  // BlockVariables for solve (no dense vectors in the solve path).
  auto x = solver.MakeBlockVariable();
  auto rhs_bv = solver.MakeBlockVariable();

  auto t0 = clock::now();

  Eigen::VectorXd weights = Eigen::VectorXd::Ones(m);
  double prev_obj = std::numeric_limits<double>::max();

  // Dense A in reduced space for rhs/residual computation.
  Eigen::MatrixXd A_reduced(A);
  if (expansion.was_reduced()) {
    A_reduced.resize(m, n_solve);
    for (int j = 0; j < n_solve; ++j)
      A_reduced.col(j) = Eigen::MatrixXd(A).col(expansion.col_map[j]);
  }

  for (int iter = 0; iter < max_iterations; ++iter) {
    solver.SetWeights(c, weights);
    if (!solver.AssembleAndFactor()) break;

    // RHS = A'Wb → scatter into BlockVariable.
    Eigen::VectorXd rhs = A_reduced.transpose() * (weights.asDiagonal() * b);
    rhs_bv.ScatterFrom(rhs);

    // Solve directly into x's blocks.
    solver.SolveInto(rhs_bv, x);

    // Residual: r = Ax - b (dense — lives in measurement space).
    Eigen::VectorXd x_dense = x.Gather();
    Eigen::VectorXd r = A_reduced * x_dense - b;
    double obj = r.lpNorm<1>();

    if (std::abs(prev_obj - obj) < tolerance * std::abs(obj) + 1e-15) {
      result.iterations = iter + 1;
      break;
    }
    prev_obj = obj;
    result.iterations = iter + 1;

    for (int i = 0; i < m; ++i)
      weights(i) = 1.0 / std::max(std::abs(r(i)), epsilon);
  }

  auto t1 = clock::now();
  Eigen::VectorXd x_final = x.Gather();
  result.x = expansion.Expand(x_final);
  result.l1_objective = (A_reduced * x_final - b).lpNorm<1>();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  return result;
}

}  // namespace conex
