#include <gtest/gtest.h>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_hybrid_r.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/common/kkt_system.h"
#include "conex/common/model.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace conex;

namespace {

// Helper: build a dense LP model centered at W=I.
KKTSystem MakeCenteredLP(int n, int m, int seed) {
  srand(seed);
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A_dense.transpose() * VectorXd::Ones(m);

  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.SetLinearCost(c);
  return KKTSystem::Build(model);
}

// =====================================================================
// Test: direction satisfies defining equations.
// =====================================================================

TEST(GeodesicHybridR, DirectionEquations) {
  // At (W=I+pert, r=e, theta=0), verify the direction satisfies
  // the Newton equations.
  const int n = 5, m = 8;
  auto system = MakeCenteredLP(n, m, 42);
  auto* kkt = system.kkt();

  VectorXd c = VectorXd::Zero(n);
  // c = A'e (centered).
  {
    RowSpace ones = kkt->MakeRowSpace();
    setOnes(ones);
    auto at_e = kkt->MakeSolverRHS();
    at_e.SetZero();
    kkt->AccumulateAtranspose(ones, at_e);
    c.resize(kkt->number_of_variables());
    at_e.supernodes->GatherInto(c);
  }
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  // Small perturbation.
  setFromVector(W, VectorXd::Ones(m) + 0.1 * VectorXd::Random(m).cwiseAbs());

  RowSpace r = kkt->MakeRowSpace();
  setOnes(r);
  RowSpace b = kkt->GetAffineTerm();

  kkt->SetScaling(W);
  kkt->AssembleAndFactor();

  // Test at several theta values.
  for (double theta : {0.0, 0.1, 0.5, 1.0}) {
    RowSpace d = kkt->MakeRowSpace();
    RowSpace delta = kkt->MakeRowSpace();
    auto info = ComputeHybridRDirection(*kkt, cost_rhs, b, W, r, theta,
                                         d, delta);
    // Extract y by re-solving (same RHS).
    RowSpace b_theta = kkt->MakeRowSpace();
    if (theta == 0.0) { b_theta = b; }
    else if (theta == 1.0) { setOnes(b_theta); }
    else {
      RowSpace ones = kkt->MakeRowSpace(); setOnes(ones);
      b_theta = addScaled(ones, b, theta, 1.0 - theta);
    }
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

    auto y_rhs = kkt->MakeSolverRHS();
    y_rhs = cost_rhs;
    RowSpace v = quadraticRepresentation(W, b_theta);
    kkt->AccumulateAtranspose(v, y_rhs);
    y_rhs *= -1;
    v = quadraticRepresentation(sqrtW, r);
    v *= 2.0;
    kkt->AccumulateAtranspose(v, y_rhs);
    kkt->SolveSolverRHS(y_rhs);
    int nv = kkt->number_of_variables();
    VectorXd y(nv);
    y_rhs.supernodes->GatherInto(y);

    auto [p_res, d_res] = VerifyHybridREquations(
        *kkt, cost_rhs, b, W, r, theta, d, delta, y);
    printf("  theta=%.1f: gap=%.2e d_inf=%.4f primal_res=%.2e dual_res=%.2e\n",
           theta, info.gap, info.d_inf, p_res, d_res);
    EXPECT_LT(p_res, 1e-10);
    EXPECT_LT(d_res, 1e-10);
  }
}

TEST(GeodesicHybridR, DirectionEquationsNonUniformR) {
  // Same but with non-uniform r.
  const int n = 5, m = 8;
  auto system = MakeCenteredLP(n, m, 77);
  auto* kkt = system.kkt();

  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs.SetZero();  // c = 0.

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  RowSpace r = kkt->MakeRowSpace();
  // Non-uniform r.
  setFromVector(r, VectorXd::Ones(m) * 0.5 +
                   0.3 * VectorXd::Random(m).cwiseAbs());

  RowSpace b = kkt->GetAffineTerm();
  kkt->SetScaling(W);
  kkt->AssembleAndFactor();

  for (double theta : {0.0, 0.3, 1.0}) {
    RowSpace d = kkt->MakeRowSpace();
    RowSpace delta = kkt->MakeRowSpace();
    ComputeHybridRDirection(*kkt, cost_rhs, b, W, r, theta, d, delta);

    // Extract y.
    RowSpace b_theta = kkt->MakeRowSpace();
    if (theta == 0.0) { b_theta = b; }
    else if (theta == 1.0) { setOnes(b_theta); }
    else {
      RowSpace ones = kkt->MakeRowSpace(); setOnes(ones);
      b_theta = addScaled(ones, b, theta, 1.0 - theta);
    }
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    auto y_rhs = kkt->MakeSolverRHS();
    y_rhs = cost_rhs;
    RowSpace v = quadraticRepresentation(W, b_theta);
    kkt->AccumulateAtranspose(v, y_rhs);
    y_rhs *= -1;
    v = quadraticRepresentation(sqrtW, r);
    v *= 2.0;
    kkt->AccumulateAtranspose(v, y_rhs);
    kkt->SolveSolverRHS(y_rhs);
    int nv = kkt->number_of_variables();
    VectorXd y(nv);
    y_rhs.supernodes->GatherInto(y);

    auto [p_res, d_res] = VerifyHybridREquations(
        *kkt, cost_rhs, b, W, r, theta, d, delta, y);
    printf("  theta=%.1f: primal_res=%.2e dual_res=%.2e\n",
           theta, p_res, d_res);
    EXPECT_LT(p_res, 1e-10);
    EXPECT_LT(d_res, 1e-10);
  }
}

// =====================================================================
// Test: at W=I, r=e, theta=1 (identity problem), d should be ~0.
// =====================================================================

TEST(GeodesicHybridR, CenteredFixedPoint) {
  const int n = 5, m = 8;
  auto system = MakeCenteredLP(n, m, 42);
  auto* kkt = system.kkt();

  // c = A'e (centered cost).
  RowSpace ones = kkt->MakeRowSpace();
  setOnes(ones);
  auto at_e = kkt->MakeSolverRHS();
  at_e.SetZero();
  kkt->AccumulateAtranspose(ones, at_e);
  VectorXd c(kkt->number_of_variables());
  at_e.supernodes->GatherInto(c);
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  RowSpace r = kkt->MakeRowSpace();
  setOnes(r);
  RowSpace b = kkt->GetAffineTerm();

  kkt->SetScaling(W);
  kkt->AssembleAndFactor();

  // At theta=1, b_theta = e, and c = A'e. The direction d should be 0.
  RowSpace d = kkt->MakeRowSpace();
  RowSpace delta = kkt->MakeRowSpace();
  auto info = ComputeHybridRDirection(*kkt, cost_rhs, b, W, r, 1.0,
                                       d, delta);
  printf("  theta=1: d_inf=%.2e gap=%.2e\n", info.d_inf, info.gap);
  EXPECT_LT(info.d_inf, 1e-10);
}

// =====================================================================
// Test: full solve on a small LP.
// =====================================================================

TEST(GeodesicHybridR, SolveSmallLP) {
  const int n = 5, m = 8;
  auto system = MakeCenteredLP(n, m, 42);
  auto* kkt = system.kkt();

  RowSpace ones = kkt->MakeRowSpace();
  setOnes(ones);
  auto at_e = kkt->MakeSolverRHS();
  at_e.SetZero();
  kkt->AccumulateAtranspose(ones, at_e);
  VectorXd c(kkt->number_of_variables());
  at_e.supernodes->GatherInto(c);
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveGeodesicHybridR(*kkt, cost_rhs, W, 500, 1e-8, true);

  printf("\n  fac=%d sol=%d mu=%.2e gap=%.2e d_inf=%.4f\n",
         result.total_factorizations, result.total_solves,
         result.mu, result.complementarity, result.d_inf_norm);
  printf("  dual_res=%.2e compl=%.2e\n",
         result.optimality.dual_residual,
         result.optimality.complementarity);

  EXPECT_LT(std::abs(result.complementarity), 1e-6);
  EXPECT_LE(result.d_inf_norm, 1.001);
}

}  // namespace
