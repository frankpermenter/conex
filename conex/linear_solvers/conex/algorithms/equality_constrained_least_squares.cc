#include "conex/algorithms/equality_constrained_least_squares.h"

#include <chrono>
#include <numeric>

#include "conex/common/problem.h"
#include "conex/common/solver.h"

namespace conex {

EqualityConstrainedLeastSquaresResult EqualityConstrainedLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double>& C,
    const Eigen::VectorXd& d) {
  using clock = std::chrono::high_resolution_clock;
  EqualityConstrainedLeastSquaresResult result;
  const int n = A.cols();

  auto t0 = clock::now();

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(A.rows()), vars);
  auto c_eq = problem.AddEqualityConstraint(C, d, vars);

  // Preprocess: drops dependent columns AND rows (with consistency check).
  auto [reduced, expansion] = Preprocess(problem);

  auto solver = Solver::Build(reduced);

  auto t1 = clock::now();

  bool ok = solver.AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  // Build RHS = [A'b; d_reduced] in reduced space.
  const auto& duals = solver.dual_variables(c_eq);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(solver.num_variables());
  rhs.head(reduced.num_variables()) =
      expansion.Reduce(Eigen::VectorXd(A.transpose() * b));

  // Get the reduced equality RHS.
  auto* eq = std::get_if<Problem::EqualityConstraintData>(
      &reduced.constraint(c_eq));
  if (eq) {
    for (int i = 0; i < static_cast<int>(duals.size()); ++i)
      rhs(duals[i]) = eq->d(i);
  }

  Eigen::VectorXd sol = solver.Solve(rhs);

  auto t3 = clock::now();

  result.x = expansion.Expand(sol.head(reduced.num_variables()));
  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.assemble_and_factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();

  return result;
}

}  // namespace conex
