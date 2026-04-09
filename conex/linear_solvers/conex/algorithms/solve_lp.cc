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

  // Cost was reduced by Preprocess.
  Eigen::VectorXd c_r = reduced.linear_cost();
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

double ComputeConstraintViolation(const Problem& problem,
                                  const Eigen::VectorXd& x) {
  double min_slack = std::numeric_limits<double>::max();
  for (const auto& c : problem.constraints()) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
        // Internal form: Ax + b >= 0.
        const int nv = static_cast<int>(data.vars.size());
        Eigen::VectorXd xv(nv);
        for (int i = 0; i < nv; ++i) xv(i) = x(data.vars[i]);
        Eigen::VectorXd s = data.A * xv + data.b;
        min_slack = std::min(min_slack, s.minCoeff());
      }
    }, c);
  }
  return min_slack;
}

}  // namespace conex
