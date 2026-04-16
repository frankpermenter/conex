// Test: when cost = A^T e and b = e, the initial point (W=I) is exactly
// centered.  Phase 1 should immediately find theta=0 feasible (d=0 at k=1)
// and the hybrid should start with d_inf ≈ 0.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"
#include "conex/algorithms/geodesic_ipm.h"

namespace conex {
namespace {

TEST(HybridTransition, TrivialCentering) {
  // Build a problem with b = e (ones) and c = A^T e.
  const int m = 10;  // rows (constraints)
  const int n = 5;   // cols (variables)
  std::srand(42);
  Eigen::MatrixXd A_dense = Eigen::MatrixXd::Random(m, n);
  Eigen::SparseMatrix<double> A = A_dense.sparseView();
  Eigen::VectorXd b = Eigen::VectorXd::Ones(m);
  Eigen::VectorXd c = A_dense.transpose() * Eigen::VectorXd::Ones(m);

  Problem problem;
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  problem.AddLinearConstraint(A, b, vars);
  problem.SetLinearCost(c);

  SolverConfiguration cfg;
  auto solver = Solver::Build(problem, cfg);
  auto* kkt = solver.solver();

  // Prepare cost RHS.
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  // Phase 1 with phase1_only: should transition immediately.
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  auto p1 = SolveGeodesicPhaseOne(*kkt, cost_rhs, W, 500, 1, 1e-8,
                                   /*verbose=*/true, /*phase1_only=*/true);
  printf("Phase1: %d iters, mu=%.2e, tau=%.4f\n",
         p1.iterations, p1.mu, p1.tau);

  // Should have transitioned at iter 0 (theta=0 immediately feasible).
  EXPECT_LE(p1.iterations, 1)
      << "Phase 1 should transition immediately for trivial centering";
  EXPECT_GT(p1.tau, 0.5) << "tau should be near 1";

  // At k=1, tau=1, theta=0, the trivially centered point has d = 0
  // exactly (d0 + d1_0 = 0 when c = A^T e and b = e at W = I).
  // Verify by computing one hybrid direction at k=1 before any steps.
  double k = 1.0;
  printf("Hybrid handoff: k=%.4f, tau=%.4f\n", k, p1.tau);

  // Manually compute one hybrid direction to check the initial state.
  kkt->SetScaling(W);
  kkt->AssembleAndFactor();
  RowSpace b_scaled = kkt->GetAffineTerm();  // b = e (ones)
  // tau=1 so no scaling needed.
  RowSpace r_check = kkt->MakeRowSpace();
  setOnes(r_check);
  r_check *= (1.0 / k);  // r = (1/k) * e = e at k=1
  RowSpace d_check = kkt->MakeRowSpace();
  RowSpace delta_check = kkt->MakeRowSpace();
  auto info0 = ComputeHybridDirection(*kkt, cost_rhs, b_scaled,
                                       W, r_check, d_check, delta_check);
  printf("Initial hybrid direction: d_inf=%.2e, gap=%.2e\n",
         info0.d_inf, info0.gap);
  EXPECT_LT(info0.d_inf, 1e-10)
      << "Hybrid should start exactly centered (d = 0) at k=1";
  // gap = <r^2, 1-d^2> = m at the central path (mu=1, k=1). Not 0.
  EXPECT_NEAR(info0.gap, m, 1e-10)
      << "Gap should equal m (= mu * rank) at the trivially centered point";
}

}  // namespace
}  // namespace conex
