// Test KKT solver accuracy: sample random x, compute b = A^T A x via
// MultiplyA and AccumulateAtranspose, solve y = G^{-1} b, report ||y - x||.
//
// At W=e (identity), G = A^T P(W) A = A^T A, so y should equal x exactly.
//
// Usage: gram_solve_test <sdpa_file>

#include <chrono>
#include <cstdio>
#include <random>
#include <string>

#include <Eigen/Dense>

#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/model.h"
#include "conex/common/sdpa_reader.h"
#include "conex/common/solver.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("Usage: %s <sdpa_file.dat-s>\n", argv[0]);
    return 1;
  }
  const std::string filename = argv[1];

  auto [problem, info] = conex::ReadSDPA(filename);
  printf("=== %s: %d constraints, %d blocks, dim=%d ===\n",
         filename.c_str(), info.num_constraints, info.num_blocks,
         info.total_matrix_dim);

  // Add a quadratic cost Q = I to regularize the Gram matrix:
  // G = A^T A + I instead of G = A^T A.
  int m = info.num_constraints;
  Eigen::SparseMatrix<double> Q(m, m);
  Q.setIdentity();
  problem.AddQuadraticCost(Q);

  conex::SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = 100000;  // force single clique
  auto solver = conex::Solver::Build(problem, cfg);
  auto* kkt = solver.solver();
  const int nvars = kkt->number_of_variables();

  // Set scaling W = e (identity) and factor.
  auto W = kkt->MakeRowSpace();
  setOnes(W);
  kkt->SetScaling(W);
  bool ok = kkt->AssembleAndFactor();
  printf("  AssembleAndFactor: %s\n", ok ? "ok" : "FAILED");

  // Sample random x.
  std::mt19937 gen(42);
  std::normal_distribution<double> dist(0.0, 1.0);
  Eigen::VectorXd x(nvars);
  for (int i = 0; i < nvars; ++i) x(i) = dist(gen);

  auto x_rhs = kkt->MakeSolverRHS();
  x_rhs = kkt->MakeBlockVariable(x);

  // Compute Ax (RowSpace).
  auto Ax = kkt->MakeRowSpace();
  kkt->MultiplyA(x_rhs, Ax);

  // Compute b = A^T(Ax) + Q*x (SolverRHS).
  auto b_rhs = kkt->MakeSolverRHS();
  b_rhs.SetZero();
  kkt->AccumulateAtranspose(Ax, b_rhs);
  kkt->AccumulateQx(x_rhs, b_rhs);

  // Solve G y = b.
  auto y_rhs = b_rhs;  // copy
  kkt->SolveSolverRHS(y_rhs);

  // Gather y and compare to x.
  Eigen::VectorXd y(nvars);
  y_rhs.supernodes->GatherInto(y);

  // Check: re-compute G·y (via primitives) and G·x (via primitives),
  // compare them directly. If y = x then ||G·y - G·x|| should be 0.
  auto Ay = kkt->MakeRowSpace();
  kkt->MultiplyA(y_rhs, Ay);
  auto Gy = kkt->MakeSolverRHS();
  Gy.SetZero();
  kkt->AccumulateAtranspose(Ay, Gy);
  kkt->AccumulateQx(y_rhs, Gy);

  auto Ax2 = kkt->MakeRowSpace();
  kkt->MultiplyA(x_rhs, Ax2);
  auto Gx = kkt->MakeSolverRHS();
  Gx.SetZero();
  kkt->AccumulateAtranspose(Ax2, Gx);
  kkt->AccumulateQx(x_rhs, Gx);

  Eigen::VectorXd v_gy(nvars), v_gx(nvars);
  Gy.supernodes->GatherInto(v_gy);
  Gx.supernodes->GatherInto(v_gx);
  printf("  ||G·y - G·x|| = %.6e\n", (v_gy - v_gx).norm());

  // Per-column check: for each i, compute b_i = G·e_i via primitives,
  // solve y = G^{-1} b_i, and check that y ≈ e_i. Report which columns fail.
  int n_bad = 0;
  Eigen::VectorXd err_per_col(nvars);
  for (int i = 0; i < nvars; ++i) {
    Eigen::VectorXd ei = Eigen::VectorXd::Zero(nvars);
    ei(i) = 1.0;
    auto ei_rhs = kkt->MakeSolverRHS();
    ei_rhs = kkt->MakeBlockVariable(ei);

    auto Aei = kkt->MakeRowSpace();
    kkt->MultiplyA(ei_rhs, Aei);

    auto bi = kkt->MakeSolverRHS();
    bi.SetZero();
    kkt->AccumulateAtranspose(Aei, bi);
    kkt->AccumulateQx(ei_rhs, bi);

    auto yi = bi;
    kkt->SolveSolverRHS(yi);
    Eigen::VectorXd yv(nvars);
    yi.supernodes->GatherInto(yv);
    err_per_col(i) = (yv - ei).cwiseAbs().maxCoeff();
    if (err_per_col(i) > 1e-8) n_bad++;
  }
  printf("  per-column: %d / %d columns have err > 1e-8, max err = %.4e\n",
         n_bad, nvars, err_per_col.maxCoeff());
  // Print indices of the first few bad columns.
  if (n_bad > 0) {
    printf("  first bad columns: ");
    int shown = 0;
    for (int i = 0; i < nvars && shown < 10; ++i) {
      if (err_per_col(i) > 1e-8) {
        printf("%d(err=%.2e) ", i, err_per_col(i));
        shown++;
      }
    }
    printf("\n");
  }

  Eigen::VectorXd err = y - x;
  double err_inf = err.cwiseAbs().maxCoeff();
  double err_2 = err.norm();
  double x_inf = x.cwiseAbs().maxCoeff();
  double x_2 = x.norm();

  printf("  ||x||_inf      = %.6e\n", x_inf);
  printf("  ||y - x||_inf  = %.6e\n", err_inf);
  printf("  ||y - x||_2    = %.6e\n", err_2);
  printf("  ||y - x||_inf / ||x||_inf = %.6e\n", err_inf / x_inf);
  printf("  ||y - x||_2 / ||x||_2     = %.6e\n", err_2 / x_2);

  return 0;
}
