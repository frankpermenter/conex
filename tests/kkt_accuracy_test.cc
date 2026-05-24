#include <gtest/gtest.h>
#include <cstdio>
#include <Eigen/Dense>

#include "conex/common/eja_ops.h"
#include "conex/common/solver.h"
#include "conex/common/qps_reader.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

namespace conex {

TEST(KKTAccuracy, QADLITTL_SolveResidual) {
  auto [problem, info] = ReadQPS(
      "/agent-workspace/problem_libraries/maros_meszaros/QPS_Files/QADLITTL.QPS");

  SolverConfiguration config;

  // Tree solver
  auto solver = Solver::Build(problem, config);
  auto* kkt = solver.kkt();
  int n = kkt->number_of_variables();

  // Get the dense KKT matrix
  Eigen::MatrixXd K(n, n);
  kkt->DenseKKTMatrix(K.data(), n, false);

  // Set identity weights (initial point)
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  kkt->SetScaling(W);

  // Get the dense KKT matrix BEFORE factoring
  kkt->Assemble();
  Eigen::MatrixXd K_pre(n, n);
  kkt->DenseKKTMatrix(K_pre.data(), n, false);

  kkt->Factor();

  // Get the dense KKT matrix AFTER factoring (for reference)
  kkt->DenseKKTMatrix(K.data(), n, false);

  // Create a random RHS and solve
  Eigen::VectorXd b_dense = Eigen::VectorXd::Random(n);

  // Solve via tree solver
  auto rhs_tree = kkt->MakeSolverRHS();
  rhs_tree.ScatterFrom(b_dense.data(), n);
  kkt->SolveSolverRHS(rhs_tree);
  Eigen::VectorXd x_tree(n);
  rhs_tree.supernodes->GatherInto(x_tree);

  // Solve via dense LDLT
  Eigen::VectorXd x_dense = K.ldlt().solve(b_dense);

  // Residuals (use pre-factor matrix for ground truth)
  double res_tree = (K_pre * x_tree - b_dense).norm() / b_dense.norm();
  double res_dense = (K_pre * x_dense - b_dense).norm() / b_dense.norm();
  double diff = (x_tree - x_dense).norm() / x_dense.norm();

  printf("  n=%d\n", n);
  printf("  K cond (approx): %.2e\n",
         K.jacobiSvd().singularValues()(0) /
         K.jacobiSvd().singularValues()(n-1));
  printf("  Tree  solve residual: %.2e\n", res_tree);
  printf("  Dense solve residual: %.2e\n", res_dense);
  printf("  Solution diff:        %.2e\n", diff);

  // Check if tree solver has separator issues
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(kkt);
  if (ts) {
    printf("  Num subsystems: %d\n", ts->num_subsystems());
  }

  // Also check: solve the same system but via DenseSolveInPlace
  Eigen::VectorXd b2 = b_dense;
  kkt->DenseSolveInPlace(b2.data(), n, 1, true);
  double res_dense_via_kkt = (K * b2 - b_dense).norm() / b_dense.norm();
  double diff2 = (b2 - x_dense).norm() / x_dense.norm();
  printf("  DenseSolveInPlace residual: %.2e\n", res_dense_via_kkt);
  printf("  DenseSolveInPlace diff:     %.2e\n", diff2);

  // Check if DenseKKTMatrix matches what we'd build manually.
  // Build a second solver with dense factorization and compare matrices.
  auto solver_d = Solver::BuildDense(problem);
  auto* kkt_d = solver_d.kkt();
  RowSpace Wd = kkt_d->MakeRowSpace();
  EuclideanJordanAlgebra::setOnes(Wd);
  kkt_d->SetScaling(Wd);
  kkt_d->AssembleAndFactor();
  int nd = kkt_d->number_of_variables();
  Eigen::MatrixXd Kd(nd, nd);
  kkt_d->DenseKKTMatrix(Kd.data(), nd, false);

  printf("  Dense solver n=%d, Tree solver n=%d\n", nd, n);
  if (nd == n) {
    Eigen::MatrixXd D = K_pre - Kd;
    printf("  ||K_pre_tree - K_dense|| = %.2e\n", D.norm());
    printf("  ||K_pre_tree|| = %.2e, ||K_dense|| = %.2e\n", K_pre.norm(), Kd.norm());
    // Find the rows/cols with largest differences
    for (int i = 0; i < n; ++i) {
      double row_err = D.row(i).norm();
      if (row_err > 1e-10) {
        printf("  Row %d: err=%.2e (diag_tree=%.2e diag_dense=%.2e)\n",
               i, row_err, K(i,i), Kd(i,i));
      }
    }
  }

  // Solve with dense solver
  Eigen::VectorXd b2d = b_dense.head(nd);
  auto rhs_d = kkt_d->MakeSolverRHS();
  rhs_d.ScatterFrom(b2d.data(), nd);
  kkt_d->SolveSolverRHS(rhs_d);
  Eigen::VectorXd x_d(nd);
  rhs_d.supernodes->GatherInto(x_d);
  double res_d_solve = (Kd * x_d - b2d).norm() / b2d.norm();
  printf("  Dense solver solve residual: %.2e\n", res_d_solve);

  // Compare model.dot(x, Qx) with dense x'*Qx.
  // Test 1: x from ScatterFrom (works)
  {
    auto x_rhs = kkt->MakeSolverRHS();
    x_rhs.ScatterFrom(b_dense.data(), n);

    auto qx_rhs = kkt->MakeSolverRHS();
    qx_rhs.SetZero();
    kkt->AccumulateQx(x_rhs, qx_rhs);

    // Method 1: model.dot (tree solver override)
    double dot_model = kkt->dot(x_rhs, qx_rhs);

    // Method 2: dense gather then Eigen dot
    Eigen::VectorXd x_d(n), qx_d(n);
    x_rhs.supernodes->GatherInto(x_d);
    kkt->GatherInto(qx_rhs, qx_d);
    double dot_dense = x_d.dot(qx_d);

    // Method 3: direct K_pre computation
    double dot_kpre = b_dense.dot(K_pre * b_dense) -
                      b_dense.dot(b_dense);  // subtract identity (barrier weight)
    // Actually, just use the gathered vectors.

    printf("  dot(x, Qx) model=%.6e  dense=%.6e  diff=%.2e\n",
           dot_model, dot_dense, std::abs(dot_model - dot_dense));
    EXPECT_NEAR(dot_model, dot_dense, 1e-6 * std::abs(dot_dense))
        << "model.dot(x, Qx) disagrees with dense computation (ScatterFrom)";
  }

  // Test 2: x from MakeSolverRHS + manual supernode copy (like GeodesicLP y0/y1)
  {
    // Simulate: scatter into supernodes only (no separator data)
    auto x_rhs = kkt->MakeSolverRHS();  // zeroed supernodes + separators
    // Copy b_dense into supernodes via ScatterFrom
    auto x_ref = kkt->MakeSolverRHS();
    x_ref.ScatterFrom(b_dense.data(), n);
    // Now copy only supernodes from x_ref to x_rhs (simulating y0/y1 extraction)
    int nb = x_rhs.supernodes->num_blocks();
    for (int bk = 0; bk < nb; ++bk)
      x_rhs.supernodes->block(bk) = x_ref.supernodes->block(bk);
    // x_rhs has correct supernodes, zero separators, blocks_fully_gathered=true

    auto qx_rhs = kkt->MakeSolverRHS();
    qx_rhs.SetZero();
    kkt->AccumulateQx(x_rhs, qx_rhs);

    double dot_model2 = kkt->dot(x_rhs, qx_rhs);

    Eigen::VectorXd x_d2(n), qx_d2(n);
    x_rhs.supernodes->GatherInto(x_d2);
    kkt->GatherInto(qx_rhs, qx_d2);
    double dot_dense2 = x_d2.dot(qx_d2);

    printf("  dot(x, Qx) model=%.6e  dense=%.6e  diff=%.2e  [supernode-only x]\n",
           dot_model2, dot_dense2, std::abs(dot_model2 - dot_dense2));
    EXPECT_NEAR(dot_model2, dot_dense2, 1e-6 * std::abs(dot_dense2))
        << "model.dot(x, Qx) disagrees with dense computation (supernode-only)";
  }

  EXPECT_LT(res_tree, 1e-8) << "Tree solver residual too large";
}

}  // namespace conex
