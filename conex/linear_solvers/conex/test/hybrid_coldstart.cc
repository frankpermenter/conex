// Cold-start hybrid benchmark on QPS files.
// Rebuilds the problem with b=ones so that W=I is on the central path,
// then sets cost = A^T I so that (W=I, k=1) → d=0.
//
// Usage: hybrid_coldstart <file.QPS> [max_iters]

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
#include "conex/common/qps_reader.h"
#include "conex/common/solver.h"

using namespace conex;
using Clock = std::chrono::high_resolution_clock;

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s <file.QPS> [max_iters]\n", argv[0]);
    return 1;
  }
  const char* path = argv[1];
  int max_iters = (argc > 2) ? std::atoi(argv[2]) : 500;

  auto [problem, info] = ReadQPS(path);
  printf("=== %s ===\n", path);
  printf("  vars=%d, eq=%d, ineq=%d, bounds=%d, quad=%d\n",
         info.num_variables, info.num_equality_rows,
         info.num_inequality_rows, info.num_bounded_vars,
         info.num_quadratic_entries);

  // Rebuild problem with b = ones for inequality constraints so that
  // W=I is on the central path.  Keep equalities and Q unchanged.
  Model centered;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        Eigen::VectorXd b_id = Eigen::VectorXd::Ones(data.A.rows());
        centered.AddLinearConstraint(data.A, b_id, data.vars);
      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        centered.AddQuadraticCost(data.Q_sparse, data.vars);
      } else if constexpr (std::is_same_v<T,
                                          Model::EqualityConstraintData>) {
        centered.AddEqualityConstraint(data.C, data.d, data.primal_vars);
      }
    }, problem.constraint(i));
  }

  auto solver = Solver::Build(centered);
  auto* kkt = solver.kkt();
  printf("  KKT vars=%d, constraints=%d\n",
         kkt->number_of_variables(), centered.num_constraints());

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // Use the original problem's linear cost.
  auto cost_rhs = kkt->MakeSolverRHS();
  if (problem.has_linear_cost()) {
    cost_rhs = kkt->MakeBlockVariable(problem.linear_cost());
  } else {
    cost_rhs.SetZero();
  }

  CompiledModel cm(*kkt, cost_rhs);
  auto t0 = Clock::now();
  // Cold start: initial_k = -1 (auto), tau = 1.0.
  auto result = SolveGeodesicHybrid(cm, W,
                                     max_iters, 1e-8, /*verbose=*/true);
  auto t1 = Clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  printf("\n  Result: %d iters, %d fac, %d sol, mu=%.2e, d_inf=%.2e\n",
         result.total_factorizations + result.total_solves,
         result.total_factorizations, result.total_solves,
         result.mu, result.d_inf_norm);
  printf("  Time: %.1f ms\n", ms);

  return 0;
}
