#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

TEST(GeodesicBarrierQP, CentralPathConvergence) {
  // Test: min c^T x  s.t. Ax >= b,  with b = ones(m), c = A^T ones(m).
  //
  // Central path at x=0: s = Ax - b = -b, but with the >= convention
  // s = Ax - b.  At x=0, s = -b = -1 (infeasible).
  //
  // Actually, for >= constraint with slack s = Ax - b >= 0, we need
  // Ax >= b.  At x=0 with b=1 that fails.  So the central path is at
  // some x_* where s = A x_* - 1 = 1.  But x doesn't matter because
  // the geodesic iteration only tracks W.
  //
  // At W = ones, k = 1 (in the API's stored -A, -b):
  //   RHS = cost + (-A)^T(-1 + 2) = A^T 1 - A^T 1 = 0  =>  y = 0
  //   d   = 1 + 1*(-1 - 0) = 0  for every row
  //
  // So W = ones is a fixed point for any A.  Perturb and verify d -> 0.
  srand(42);
  const int n = 5, m = 8;

  // Arbitrary A (rectangular, need not be square).
  std::vector<Eigen::Triplet<double>> trips;
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  // Ax + b >= 0 with b = ones.  Central path at W=1, k=1, x=0: d = 1-(0+1) = 0.
  // Cost: c = A^T ones  (from RHS = -c + A^T(-b+2) = 0 at W=1, b=1).
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, b, vars);
  auto [reduced, expansion] = Preprocess(problem);
  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();

  auto cost_rhs = kkt->MakeSolverRHS();
  VectorXd c_r = expansion.Reduce(c);
  cost_rhs = kkt->MakeBlockVariable(c_r);

  // Initialize W = ones + small perturbation.
  RowSpace W = kkt->MakeRowSpace();
  setFromVector(W, VectorXd::Ones(m) + 0.01 * VectorXd::Random(m));

  // Phase 1: center at k = 1.
  double k = 1.0;
  auto result = GeodesicCenter(*kkt, cost_rhs, W, k, 100, 1e-10);

  printf("k=%.2f: %d iters, ||d||_inf=%.2e\n",
         k, result.iterations, result.d_inf_norm);
  EXPECT_LT(result.d_inf_norm, 1e-8);
  RowSpace W_ones = kkt->MakeRowSpace();
  setOnes(W_ones);
  EXPECT_NEAR(normInf(addScaled(W, W_ones, 1.0, -1.0)), 0.0, 1e-6);

  // Phase 2: line-search for k, then re-center.
  const int num_updates = 6;
  for (int step = 0; step < num_updates; ++step) {
    double k_new = GeodesicLineSearch(*kkt, cost_rhs, W);
    EXPECT_GE(k_new, k);

    k = k_new;
    result = GeodesicCenter(*kkt, cost_rhs, W, k, 100, 1e-10);
    printf("k=%.4f  mu=%.2e: %d iters, d_inf=%.2e, d_sqr=%.2e, "
           "s_dot_x=%.2e\n",
           k, result.mu, result.iterations, result.d_inf_norm,
           result.d_sq_norm, result.complementarity);
    EXPECT_LT(result.d_inf_norm, 1e-8);
  }

  // After increasing k, W should have moved away from ones.
  EXPECT_GT(normInf(addScaled(W, W_ones, 1.0, -1.0)), 0.01);
}

TEST(GeodesicBarrierQP, MultipleConstraints) {
  // Geodesic IPM on a problem with two separate AddLinearConstraint calls.
  // min c^T x s.t. A1*x >= b1, A2*x >= b2.
  // Central path at W=ones when b=ones and c = (A1^T + A2^T) * ones.
  srand(99);
  const int n = 6, m1 = 10, m2 = 8;

  MatrixXd A1_dense = MatrixXd::Random(m1, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m1, n);
  MatrixXd A2_dense = MatrixXd::Random(m2, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m2, n);

  auto toSparse = [](const MatrixXd& M) {
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < M.rows(); ++i)
      for (int j = 0; j < M.cols(); ++j)
        trips.emplace_back(i, j, M(i, j));
    Eigen::SparseMatrix<double> S(M.rows(), M.cols());
    S.setFromTriplets(trips.begin(), trips.end());
    return S;
  };

  Eigen::SparseMatrix<double> A1 = toSparse(A1_dense);
  Eigen::SparseMatrix<double> A2 = toSparse(A2_dense);
  VectorXd b1 = VectorXd::Ones(m1);
  VectorXd b2 = VectorXd::Ones(m2);
  // Ax + b >= 0.  Central path cost: c = (A1^T + A2^T) ones.
  VectorXd c = A1.transpose() * VectorXd::Ones(m1) +
               A2.transpose() * VectorXd::Ones(m2);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A1, b1, vars);
  problem.AddLinearConstraint(A2, b2, vars);
  auto [reduced, expansion] = Preprocess(problem);
  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();

  auto cost_rhs = kkt->MakeSolverRHS();
  VectorXd c_r = expansion.Reduce(c);
  cost_rhs = kkt->MakeBlockVariable(c_r);

  const int m = m1 + m2;

  // --- Test 1: GeodesicCenter at k=1, W=ones is fixed point ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);
    // Small perturbation.
    setFromVector(W, VectorXd::Ones(m) + 0.01 * VectorXd::Random(m));

    auto result = GeodesicCenter(*kkt, cost_rhs, W, 1.0, 100, 1e-10);
    EXPECT_LT(result.d_inf_norm, 1e-8);

    // W should return to ones.
    RowSpace W_ones = kkt->MakeRowSpace();
    setOnes(W_ones);
    EXPECT_NEAR(normInf(addScaled(W, W_ones, 1.0, -1.0)), 0.0, 1e-6);

    printf("MultipleConstraints center: %d iters, d_inf=%.2e\n",
           result.iterations, result.d_inf_norm);
  }

  // --- Test 2: SolveGeodesicLP (0 centering) ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);

    auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 0, 1e-8, true);
    EXPECT_LT(result.complementarity, 1e-7);

    printf("MultipleConstraints geodesic: %d fac, %d sol, gap=%.2e\n",
           result.total_factorizations, result.total_solves,
           result.complementarity);
  }

  // --- Test 3: SolveGeodesicHybrid ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);

    auto result = SolveGeodesicHybrid(*kkt, cost_rhs, W, 50, 1e-8);
    EXPECT_LT(std::abs(result.complementarity), 1e-7);

    printf("MultipleConstraints hybrid: %d fac, %d sol, gap=%.2e\n",
           result.total_factorizations, result.total_solves,
           result.complementarity);
  }

  // --- Test 4: Verify RowSpace has correct number of segments ---
  {
    RowSpace W = kkt->MakeRowSpace();
    // With 2 constraints decomposed across cliques, num_constraints >= 2.
    EXPECT_GE(W.num_constraints(), 2);
    // Total rows = m1 + m2.
    EXPECT_EQ(W.total_rows(), m);
    // Every segment has ops.
    for (int i = 0; i < W.num_constraints(); ++i) {
      EXPECT_NE(W.ops[i], nullptr);
    }

    printf("MultipleConstraints segments: %d (from %d + %d rows)\n",
           W.num_constraints(), m1, m2);
  }
}

// =====================================================================
// PSD (SDP) tests for the geodesic IPM.
// =====================================================================

Eigen::SparseMatrix<double> toSparse(const MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i, j)) > 1e-14)
        trips.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(trips.begin(), trips.end());
  return S;
}

// Toy SDP: min c^T x  s.t. B + x1*A1 + x2*A2 ≽ 0
// with B = I, symmetric A_i chosen so central path at W=I, k=1 is x=0.
//
// At x=0 the slack is S = B = I.  For geodesic centering at k=1 with
// W=I, d=0 requires the RHS to vanish:
//   RHS_j = -c_j + A^T_j(-P(W)b + 2W)
// With W=I, P(W)b = b, so RHS_j = -c_j + A^T_j(-b + 2I).
// For b = vec(I): RHS_j = -c_j + <A_j, 2I - I> = -c_j + <A_j, I> = -c_j + trace(A_j).
// So c_j = trace(A_j) makes x=0 the central path point.
TEST(GeodesicSDP, CenterConvergence) {
  srand(42);
  const int n = 3;
  const int p = 2;

  // Symmetric A_i.
  MatrixXd A1 = MatrixXd::Random(n, n);
  A1 = 0.5 * (A1 + A1.transpose());
  MatrixXd A2 = MatrixXd::Random(n, n);
  A2 = 0.5 * (A2 + A2.transpose());
  MatrixXd B = MatrixXd::Identity(n, n);

  std::vector<Eigen::SparseMatrix<double>> A_list = {toSparse(A1), toSparse(A2)};
  std::vector<int> vars = {0, 1};

  // c_j = trace(A_j) so W=I at k=1 is fixed point.
  VectorXd c(p);
  c(0) = A1.trace();
  c(1) = A2.trace();

  Problem problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, /*use_chordal=*/false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.solver();

  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  // Initialize W = I + small perturbation (symmetric).
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  {
    int n2 = n * n;
    MatrixXd pert = 0.05 * MatrixXd::Random(n, n);
    pert = 0.5 * (pert + pert.transpose());
    MatrixXd W0 = MatrixXd::Identity(n, n) + pert;
    for (int i = 0; i < n2; ++i) W.segment_ptr(0)[i] = W0.data()[i];
  }

  auto result = GeodesicCenter(*kkt, cost_rhs, W, 1.0, 100, 1e-10);
  printf("SDP Center: %d iters, d_inf=%.2e\n",
         result.iterations, result.d_inf_norm);
  EXPECT_LT(result.d_inf_norm, 1e-6);

  // W should return to I.
  RowSpace W_ones = kkt->MakeRowSpace();
  setOnes(W_ones);
  double w_err = normInf(addScaled(W, W_ones, 1.0, -1.0));
  printf("  ||W - I||_inf = %.2e\n", w_err);
  EXPECT_LT(w_err, 1e-4);
}

// Sanity check: encode a working LP as a diagonal SDP and verify
// identical centering iterations (fixed k=1).
TEST(GeodesicSDP, DiagonalMatchesLP) {
  srand(42);
  const int n = 5, m = 8;

  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A_dense.transpose() * VectorXd::Ones(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // Same perturbation for both.
  VectorXd pert = 0.1 * VectorXd::Random(m);

  // --- LP path ---
  Problem lp_problem;
  {
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < m; ++i)
      for (int j = 0; j < n; ++j)
        trips.emplace_back(i, j, A_dense(i, j));
    Eigen::SparseMatrix<double> A(m, n);
    A.setFromTriplets(trips.begin(), trips.end());
    lp_problem.AddLinearConstraint(A, b, vars);
  }

  auto lp_solver = Solver::Build(lp_problem);
  auto* lp_kkt = lp_solver.solver();
  auto lp_cost = lp_kkt->MakeSolverRHS();
  lp_cost = lp_kkt->MakeBlockVariable(c);
  RowSpace lp_W = lp_kkt->MakeRowSpace();
  setFromVector(lp_W, VectorXd::Ones(m) + pert);
  auto lp_result = GeodesicCenter(*lp_kkt, lp_cost, lp_W, 1.0, 20, 1e-12, true);

  // --- SDP path (diagonal matrices, no chordal) ---
  Problem sdp_problem;
  {
    std::vector<Eigen::SparseMatrix<double>> A_list;
    for (int j = 0; j < n; ++j) {
      MatrixXd Aj = MatrixXd::Zero(m, m);
      for (int i = 0; i < m; ++i) Aj(i, i) = A_dense(i, j);
      A_list.push_back(toSparse(Aj));
    }
    MatrixXd B = MatrixXd::Zero(m, m);
    for (int i = 0; i < m; ++i) B(i, i) = b(i);
    sdp_problem.AddPSDConstraint(A_list, toSparse(B), vars,
                                  /*use_chordal=*/false);
  }

  auto sdp_solver = Solver::Build(sdp_problem);
  auto* sdp_kkt = sdp_solver.solver();
  auto sdp_cost = sdp_kkt->MakeSolverRHS();
  sdp_cost = sdp_kkt->MakeBlockVariable(c);
  RowSpace sdp_W = sdp_kkt->MakeRowSpace();
  // Set W = diag(ones + pert) as an m×m matrix.
  {
    int m2 = m * m;
    for (int i = 0; i < m2; ++i) sdp_W.segment_ptr(0)[i] = 0;
    for (int i = 0; i < m; ++i) sdp_W.segment_ptr(0)[i * m + i] = 1.0 + pert(i);
  }
  auto sdp_result = GeodesicCenter(*sdp_kkt, sdp_cost, sdp_W, 1.0, 20, 1e-12, true);

  printf("\nCentering:\n");
  printf("  LP:  %d iters, d_inf=%.2e\n", lp_result.iterations, lp_result.d_inf_norm);
  printf("  SDP: %d iters, d_inf=%.2e\n", sdp_result.iterations, sdp_result.d_inf_norm);
  EXPECT_EQ(lp_result.iterations, sdp_result.iterations);
  EXPECT_NEAR(lp_result.d_inf_norm, sdp_result.d_inf_norm, 1e-8);

  // --- Full LP path: decomposition + line search + geodesic step ---
  // Reset W to ones for both.
  setOnes(lp_W);
  {
    int m2 = m * m;
    for (int i = 0; i < m2; ++i) sdp_W.segment_ptr(0)[i] = 0;
    for (int i = 0; i < m; ++i) sdp_W.segment_ptr(0)[i * m + i] = 1.0;
  }

  const int lp_iters = 12;
  auto lp_lp = SolveGeodesicLP(*lp_kkt, lp_cost, lp_W, lp_iters, 0, 1e-12, true);
  auto sdp_lp = SolveGeodesicLP(*sdp_kkt, sdp_cost, sdp_W, lp_iters, 0, 1e-12, true);

  printf("\nLP solve:\n");
  printf("  %3s  %12s  %12s  %12s  %12s\n", "it", "LP_dinf", "SDP_dinf", "LP_mu", "SDP_mu");
  printf("  %s\n", std::string(55, '-').c_str());
  int iters = std::min(lp_lp.iterations, sdp_lp.iterations);
  for (int i = 0; i < iters; ++i) {
    printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e\n", i,
           lp_lp.iter_stats[i].d_inf, sdp_lp.iter_stats[i].d_inf,
           lp_lp.iter_stats[i].mu, sdp_lp.iter_stats[i].mu);
  }
  EXPECT_EQ(lp_lp.iterations, sdp_lp.iterations);
  for (int i = 0; i < iters; ++i) {
    EXPECT_NEAR(lp_lp.iter_stats[i].d_inf,
                sdp_lp.iter_stats[i].d_inf, 1e-8);
    EXPECT_NEAR(lp_lp.iter_stats[i].mu,
                sdp_lp.iter_stats[i].mu,
                1e-6 * lp_lp.iter_stats[i].mu + 1e-14);
  }
}

// Non-diagonal SDP: centering at k=1 with dense symmetric A_i.
// B = I, c_j = trace(A_j) → W=I at k=1 is centered.
// Perturb W and verify centering recovers d → 0.
TEST(GeodesicSDP, NonDiagonalCenter) {
  srand(42);
  const int n = 3;
  const int p = 4;

  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars;
  for (int k = 0; k < p; ++k) {
    MatrixXd Ak = MatrixXd::Random(n, n);
    Ak = 0.5 * (Ak + Ak.transpose());
    A_list.push_back(toSparse(Ak));
    vars.push_back(k);
  }
  MatrixXd B = MatrixXd::Identity(n, n);
  VectorXd c(p);
  for (int j = 0; j < p; ++j) c(j) = A_list[j].toDense().trace();

  Problem problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, /*use_chordal=*/false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.solver();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  // Center at k=1 from W=I (should be 1 iter since already centered).
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  auto r0 = GeodesicCenter(*kkt, cost_rhs, W, 1.0, 50, 1e-10);
  printf("k=1 (W=I): %d iters, d_inf=%.2e\n", r0.iterations, r0.d_inf_norm);
  EXPECT_LT(r0.d_inf_norm, 1e-8);
  EXPECT_LE(r0.iterations, 2);

  // Center at k=1 from perturbed W.
  {
    int n2 = n * n;
    MatrixXd pert = 0.05 * MatrixXd::Random(n, n);
    pert = 0.5 * (pert + pert.transpose());
    MatrixXd W0 = MatrixXd::Identity(n, n) + pert;
    for (int i = 0; i < n2; ++i) W.segment_ptr(0)[i] = W0.data()[i];
  }
  auto r1 = GeodesicCenter(*kkt, cost_rhs, W, 1.0, 50, 1e-10);
  printf("k=1 (perturbed): %d iters, d_inf=%.2e\n",
         r1.iterations, r1.d_inf_norm);
  EXPECT_LT(r1.d_inf_norm, 1e-8);

  // One line search + re-center should work.
  double k = 1.0;
  double k_new = GeodesicLineSearch(*kkt, cost_rhs, W);
  printf("line search: k_new=%.4f\n", k_new);
  EXPECT_GT(k_new, k);
  k = k_new;
  auto r2 = GeodesicCenter(*kkt, cost_rhs, W, k, 50, 1e-10);
  printf("k=%.4f: %d iters, d_inf=%.2e\n", k, r2.iterations, r2.d_inf_norm);
  EXPECT_LT(r2.d_inf_norm, 1e-8);
}

// Non-diagonal SDP: SolveGeodesicLP makes progress (k increases,
// d_inf stays near 1).  Full convergence is limited by the
// d0/d1 decomposition having large individual terms for PSD.
TEST(GeodesicSDP, NonDiagonalLP) {
  srand(42);
  const int n = 3;
  const int p = 4;

  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars;
  for (int k = 0; k < p; ++k) {
    MatrixXd Ak = MatrixXd::Random(n, n);
    Ak = 0.5 * (Ak + Ak.transpose());
    A_list.push_back(toSparse(Ak));
    vars.push_back(k);
  }
  MatrixXd B = MatrixXd::Identity(n, n);
  VectorXd c(p);
  for (int j = 0; j < p; ++j) c(j) = A_list[j].toDense().trace();

  Problem problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, /*use_chordal=*/false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.solver();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 0, 1e-6, true);
  printf("SDP LP: %d fac, mu=%.2e, d_inf=%.2e\n",
         result.total_factorizations, result.mu, result.d_inf_norm);

  // Invariant: k is nondecreasing, and d_inf ≈ 1 when k increases.
  for (int i = 1; i < result.iterations; ++i) {
    EXPECT_GE(result.iter_stats[i].mu, 0);  // gap nonneg
    // mu = 1/k^2, so mu decreasing ↔ k increasing.
    if (result.iter_stats[i].mu < result.iter_stats[i-1].mu) {
      // k increased — line search hit the boundary.
      EXPECT_NEAR(result.iter_stats[i].d_inf, 1.0, 0.01);
    }
  }
  // k should increase beyond 1.
  EXPECT_GT(1.0 / std::sqrt(result.mu), 1.2);
}

// Test the hybrid's W-update loop in isolation.
// Start centered (W=I, r=ones → d=0, Δ=0), perturb r, then
// repeatedly take geodesic steps on W (no r-shrinking) and check
// if Δ → 0.
TEST(GeodesicSDP, HybridCenteringLoop) {
  srand(42);
  const int n = 3, p = 4;

  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars;
  for (int k = 0; k < p; ++k) {
    MatrixXd Ak = MatrixXd::Random(n, n);
    Ak = 0.5 * (Ak + Ak.transpose());
    A_list.push_back(toSparse(Ak));
    vars.push_back(k);
  }
  MatrixXd B = MatrixXd::Identity(n, n);
  VectorXd c(p);
  for (int j = 0; j < p; ++j) c(j) = A_list[j].toDense().trace();

  Problem problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.solver();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // Perturb r from identity.
  RowSpace r = kkt->MakeRowSpace();
  setOnes(r);
  {
    int n2 = n * n;
    MatrixXd pert = 0.1 * MatrixXd::Random(n, n);
    pert = 0.5 * (pert + pert.transpose());
    MatrixXd R0 = MatrixXd::Identity(n, n) + pert;
    for (int i = 0; i < n2; ++i) r.segment_ptr(0)[i] = R0.data()[i];
  }

  printf("  %3s  %12s  %12s  %12s\n",
         "iter", "gap", "d_inf", "d_sqr");
  printf("  %s\n", std::string(42, '-').c_str());

  // Centering loop: uses the same HybridCenteringStep as the algorithm.
  for (int iter = 0; iter < 20; ++iter) {
    auto info = HybridCenteringStep(*kkt, cost_rhs, W, r);
    printf("  %3d  %12.4e  %12.4e  %12.4e\n",
           iter, info.gap, info.d_inf, info.d_sq);
    if (info.d_inf < 1e-10) break;
  }

  // Verify convergence: one more direction computation (no step).
  {
    RowSpace weights = cwiseProduct(W, W);
    kkt->SetWeights(weights);
    kkt->AssembleAndFactor();
    RowSpace d = kkt->MakeRowSpace();
    RowSpace delta = kkt->MakeRowSpace();
    auto info = ComputeHybridDirection(*kkt, cost_rhs, W, r, d, delta);
    printf("\nFinal: d_inf=%.2e, gap=%.2e\n", info.d_inf, info.gap);
    EXPECT_LT(info.d_inf, 1e-6);
  }
}

}  // namespace
}  // namespace conex
