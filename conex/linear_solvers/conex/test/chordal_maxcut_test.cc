#include <cstdio>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/tree_solver/kkt_tree_solver.h"
#include "conex/common/equality_constraint.h"
using namespace conex;

int main() {
  Problem p;
  Eigen::SparseMatrix<double> A(2, 2);
  A.insert(0, 0) = 1; A.insert(1, 1) = 1; A.makeCompressed();
  std::vector<int> vars = {0, 1};
  Eigen::VectorXd b0 = Eigen::VectorXd::Zero(2);
  p.AddLinearConstraint(A, b0, vars);
  Eigen::SparseMatrix<double> C(1, 2);
  C.insert(0, 0) = 1; C.insert(0, 1) = 1; C.makeCompressed();
  Eigen::VectorXd d(1); d(0) = 1.0;
  p.AddEqualityConstraint(C, d, vars);
  Eigen::VectorXd cost(2); cost << 1, 1;
  p.SetLinearCost(cost);

  auto solver = Solver::Build(p);
  auto* kkt = solver.solver();
  auto* ts = solver.tree_solver();

  printf("n_total = %d\n", kkt->number_of_variables());
  printf("n_eq_assemblers = %zu\n", ts->equality_sub_assemblers().size());
  for (const auto* ec : ts->equality_sub_assemblers()) {
    printf("  dual_vars:");
    for (int dv : ec->dual_variables()) printf(" %d", dv);
    printf("  d:");
    for (int i = 0; i < ec->affine_term().size(); ++i)
      printf(" %.2f", ec->affine_term()(i));
    printf("\n");
  }

  // Solve with dense interface first (known to work).
  kkt->AssembleAndFactor();
  int nv = kkt->number_of_variables();
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nv);
  rhs.head(2) = cost;
  for (const auto* ec : ts->equality_sub_assemblers()) {
    const auto& dv = ec->dual_variables();
    const auto& dd = ec->affine_term();
    for (int i = 0; i < (int)dv.size(); ++i) rhs(dv[i]) = dd(i);
  }
  Eigen::VectorXd sol = kkt->Solve(rhs);
  printf("Dense solve: x = [%.6f, %.6f], cost = %.6f\n",
         sol(0), sol(1), cost.dot(sol.head(2)));

  // Now solve with geodesic IPM.
  // Cost at primal, +d at dual (scaled by k*tau in the IPM's parameterization).
  Eigen::VectorXd cost_full = Eigen::VectorXd::Zero(nv);
  cost_full.head(cost.size()) = cost;
  for (const auto* ec : ts->equality_sub_assemblers()) {
    const auto& dv = ec->dual_variables();
    const auto& dd = ec->affine_term();
    for (int i = 0; i < (int)dv.size(); ++i)
      cost_full(dv[i]) = dd(i);
  }
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(cost_full);
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  // Manual first decomp to check.
  kkt->SetScaling(W);
  printf("Factor: %s\n", kkt->AssembleAndFactor() ? "ok" : "FAIL");
  printf("n_vars=%d\n", kkt->number_of_variables());
  auto r = SolveGeodesicThetaContinuation(*kkt, cost_rhs, W, 500, 1, 1e-8, true);
  printf("IPM: x.size=%d, mu=%.2e, iters=%d\n",
         (int)r.x.size(), r.mu, r.iterations);
  if (r.x.size() >= 2) {
    printf("IPM solve: x=[%.6f, %.6f], cost=%.6f\n",
           r.x(0), r.x(1), cost.dot(r.x.head(2)));
  }
}
