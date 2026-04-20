#include "conex/algorithms/solve_lp.h"

#include <limits>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/solver.h"

namespace conex {

LPResult SolveLP(const Model& model, double tolerance) {
  auto solver = Solver::Build(model);
  auto result = solver.Solve(GeodesicLP{.tolerance = tolerance});

  LPResult out;
  out.x = result.x;
  out.objective = result.objective;
  out.gap = result.complementarity;
  out.factorizations = result.factorizations;
  out.solves = result.iterations;  // approximate
  return out;
}

double ComputeConstraintViolation(const Model& model,
                                  const Eigen::VectorXd& x) {
  double min_slack = std::numeric_limits<double>::max();
  for (const auto& c : model.constraints()) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
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
