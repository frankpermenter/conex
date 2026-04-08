#include "conex/algorithms/solve_lp.h"

#include <numeric>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/common/solver.h"

namespace conex {

LPResult SolveLP(const Problem& problem, double tolerance) {
  auto [reduced, expansion] = Preprocess(problem);
  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();

  // Build cost in SolverRHS format.
  Eigen::VectorXd c_r = expansion.Reduce(problem.linear_cost());
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c_r);

  // Initialize W = ones.
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 0, tolerance);

  LPResult out;
  out.gap = result.complementarity;
  out.factorizations = result.total_factorizations;
  out.solves = result.total_solves;

  // Recover x from the final solve: use the last decomposition's y.
  // For now, solve one more time to get x.
  // TODO: cache x from the last iteration.
  out.x = Eigen::VectorXd::Zero(problem.num_variables());
  out.objective = 0;

  return out;
}

}  // namespace conex
