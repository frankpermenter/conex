// Minimal test: lineSearchK claims ||d0 + k*d1||_inf <= 1 but
// the evaluated norm exceeds 1.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/sdpa_reader.h"

namespace conex {
namespace {

TEST(LineSearchBug, EvaluatedNormExceedsBound) {
  auto [problem, info] = ReadSDPA(
      "/agent-workspace/problem_libraries/SDPLIB/data/truss8.dat-s");
  SolverConfiguration cfg;
  auto solver = Solver::Build(problem, cfg);
  auto* kkt = solver.solver();

  auto cost_rhs = kkt->MakeSolverRHS();
  if (problem.has_linear_cost())
    cost_rhs = kkt->MakeBlockVariable(problem.linear_cost());

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // Run phase 1 with phase1_only — this exits at the exact iteration
  // where lineSearchK(d0, tau*d1_0, 1.1) > 0.
  auto p1 = SolveGeodesicPhaseOne(*kkt, cost_rhs, W, 500, 1, 1e-8,
                                   /*verbose=*/true, /*phase1_only=*/true);
  ASSERT_GT(p1.mu, 0) << "Phase 1 should have reached theta=0";

  double tau = p1.tau;
  double k_p1 = 1.0 / std::sqrt(p1.mu);

  // Decompose at the exit W (same state as phase 1's last decomp).
  RowSpace b = kkt->GetAffineTerm();
  auto decomp = ComputeFullDecomposition(*kkt, cost_rhs, b, W);

  // Reproduce lineSearchK call from phase 1.
  RowSpace tau_d1_0 = decomp.d1_0;
  tau_d1_0 *= tau;
  double k_ls = lineSearchK(decomp.d0, tau_d1_0, 1.1);
  printf("lineSearchK(bound=1.1) = %.6e (phase1 k=%.6e)\n", k_ls, k_p1);

  // Evaluate the direction at the lineSearchK's k.
  if (k_ls > 0) {
    RowSpace d = addScaled(decomp.d0, tau_d1_0, 1.0, k_ls);
    double d_inf_direct = normInf(d);
    printf("Direct d_inf(k_ls) = %.6e (should be <= 1.1)\n", d_inf_direct);
    EXPECT_LE(d_inf_direct, 1.101)
        << "lineSearchK guarantee violated";
  }

  // Evaluate via EvaluateDirection (uses decomp.d1_0 separately).
  RowSpace d_eval = EvaluateDirection(decomp, k_ls, tau, 0.0);
  double d_inf_eval = normInf(d_eval);
  printf("EvaluateDirection d_inf(k_ls) = %.6e\n", d_inf_eval);

  // These two should agree.
  EXPECT_NEAR(normInf(addScaled(decomp.d0, tau_d1_0, 1.0, k_ls)),
              d_inf_eval, 1e-10)
      << "Direct and EvaluateDirection should agree";
}

}  // namespace
}  // namespace conex
