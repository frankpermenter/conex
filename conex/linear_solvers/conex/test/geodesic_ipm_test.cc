#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/compiled_model.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
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

  Model problem;
  problem.AddLinearConstraint(A, b, vars);
  problem.SetLinearCost(c);
  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto cost_rhs = solver.MakeCostRHS();
  CompiledModel cm(*kkt, cost_rhs);

  // Initialize W = ones + small perturbation.
  RowSpace W = kkt->MakeRowSpace();
  setFromVector(W, VectorXd::Ones(m) + 0.01 * VectorXd::Random(m));

  // Phase 1: center at k = 1.
  double k = 1.0;
  auto result = GeodesicCenter(cm, W, k, 100, 1e-10);

  printf("k=%.2f: %d iters, ||d||_inf=%.2e\n",
         k, result.iterations, result.d_inf_norm);
  EXPECT_LT(result.d_inf_norm, 1e-8);
  RowSpace W_ones = kkt->MakeRowSpace();
  setOnes(W_ones);
  EXPECT_NEAR(normInf(addScaled(W, W_ones, 1.0, -1.0)), 0.0, 1e-6);

  // Phase 2: line-search for k, then re-center.
  const int num_updates = 6;
  for (int step = 0; step < num_updates; ++step) {
    double k_new = GeodesicLineSearch(cm, W);
    EXPECT_GE(k_new, k);

    k = k_new;
    result = GeodesicCenter(cm, W, k, 100, 1e-10);
    printf("k=%.4f  mu=%.2e: %d iters, d_inf=%.2e, d_sqr=%.2e, "
           "s_dot_x=%.2e\n",
           k, result.mu, result.iterations, result.d_inf_norm,
           result.d_sq_norm, result.complementarity);
    EXPECT_LT(result.d_inf_norm, 1e-8);
  }

  // After increasing k, W should have moved away from ones.
  EXPECT_GT(normInf(addScaled(W, W_ones, 1.0, -1.0)), 0.01);
}

TEST(GeodesicBarrierQP, FullDecomposition) {
  // Verify ComputeFullDecomposition + EvaluateDirection against
  // VerifyNewtonEquations for several (k, theta) pairs.
  srand(77);
  const int n = 5, m = 8;

  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddLinearConstraint(A, b, vars);
  problem.SetLinearCost(c);
  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto cost_rhs = solver.MakeCostRHS();
  CompiledModel cm(*kkt, cost_rhs);

  // Perturb W away from identity.
  RowSpace W = kkt->MakeRowSpace();
  setFromVector(W, VectorXd::Ones(m) + 0.3 * VectorXd::Random(m));

  const RowSpace b_rs = kkt->GetAffineTerm();

  auto decomp = ComputeFullDecomposition(cm, b_rs, W);

  // Test several (k, theta) pairs.
  double test_ks[] = {0.5, 1.0, 2.0, 5.0};
  double test_thetas[] = {0.0, 0.1, 0.5, 1.0};

  double test_taus[] = {0.0, 0.5, 1.0};
  for (double k : test_ks) {
    for (double tau : test_taus) {
      for (double theta : test_thetas) {
        RowSpace d = EvaluateDirection(decomp, k, tau, theta);

        // Reconstruct y = y0 + k * (tau * y1_0 + theta * y1_theta).
        Eigen::VectorXd y = decomp.y0 + k * (tau * decomp.y1_0 + theta * decomp.y1_theta);

        // VerifyNewtonEquations uses a single blend parameter theta
        // (b = theta*e + (1-theta)*b_0), which only matches the decoupled
        // (tau, theta) when tau=1, theta=0 (original problem, no blend).
        if (tau == 1.0 && theta == 0.0) {
          auto [p_res, d_res] = VerifyNewtonEquations(
              cm, b_rs, W, d, y, k, 0.0);
          printf("  k=%.1f tau=%.1f theta=%.1f: primal=%.2e  dual=%.2e\n",
                 k, tau, theta, p_res, d_res);
          EXPECT_LT(p_res, 1e-10);
          EXPECT_LT(d_res, 1e-10);
        }
      }
    }
  }

  // Verify MinNormK: at k*, ||d||^2 should be minimal.
  for (double theta : test_thetas) {
    double tau = 1.0;
    double k_star = MinNormK(decomp, tau, theta);
    RowSpace d_star = EvaluateDirection(decomp, k_star, tau, theta);
    double dsq_star = squaredNorm(d_star);

    // Perturbing k in either direction should increase ||d||^2.
    double eps = 1e-6;
    double dsq_plus = squaredNorm(EvaluateDirection(decomp, k_star + eps, tau, theta));
    double dsq_minus = squaredNorm(EvaluateDirection(decomp, k_star - eps, tau, theta));

    printf("  theta=%.1f: k*=%.4f  ||d||^2=%.4e  (d+)=%.4e  (d-)=%.4e\n",
           theta, k_star, dsq_star, dsq_plus, dsq_minus);
    EXPECT_LE(dsq_star, dsq_plus + 1e-12);
    EXPECT_LE(dsq_star, dsq_minus + 1e-12);
  }
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

  Model problem;
  problem.AddLinearConstraint(A1, b1, vars);
  problem.AddLinearConstraint(A2, b2, vars);
  problem.SetLinearCost(c);
  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto cost_rhs = solver.MakeCostRHS();
  CompiledModel cm(*kkt, cost_rhs);

  const int m = m1 + m2;

  // --- Test 1: GeodesicCenter at k=1, W=ones is fixed point ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);
    // Small perturbation.
    setFromVector(W, VectorXd::Ones(m) + 0.01 * VectorXd::Random(m));

    auto result = GeodesicCenter(cm, W, 1.0, 100, 1e-10);
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

    auto result = SolveGeodesicLP(cm, W, 30, 0, 1e-8, true);
    EXPECT_LT(result.complementarity, 1e-7);

    printf("MultipleConstraints geodesic: %d fac, %d sol, gap=%.2e\n",
           result.total_factorizations, result.total_solves,
           result.complementarity);
  }

  // --- Test 3: SolveGeodesicHybrid ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);

    auto result = SolveGeodesicHybrid(cm, W, 50, 1e-8);
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

// Nonneg hybrid centering: perturb r, run HybridCenteringStep loop,
// verify d → 0.  Exercises NonnegOrthantOps::updateAutomorphism.
TEST(GeodesicBarrierQP, HybridCenteringLoop) {
  srand(42);
  const int n = 5, m = 8;

  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A_dense.transpose() * VectorXd::Ones(m);

  auto toSparseLoc = [](const MatrixXd& M) {
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < M.rows(); ++i)
      for (int j = 0; j < M.cols(); ++j)
        trips.emplace_back(i, j, M(i, j));
    Eigen::SparseMatrix<double> S(M.rows(), M.cols());
    S.setFromTriplets(trips.begin(), trips.end());
    return S;
  };

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddLinearConstraint(toSparseLoc(A_dense), b, vars);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // Perturb r from ones.
  RowSpace r = kkt->MakeRowSpace();
  setFromVector(r, VectorXd::Ones(m) + 0.1 * VectorXd::Random(m));

  CompiledModel cm(*kkt, cost_rhs);
  for (int iter = 0; iter < 20; ++iter) {
    auto info = HybridCenteringStep(cm, W, r);
    if (info.d_inf < 1e-10) break;
  }

  // Verify convergence.
  cm.SetScaling(W);
  cm.AssembleAndFactor();
  RowSpace d = cm.MakeRowSpace();
  RowSpace delta = cm.MakeRowSpace();
  const RowSpace b_aff = cm.GetAffineTerm();
  auto info = ComputeHybridDirection(cm, b_aff, W, r, d, delta);
  printf("Nonneg HybridCentering: d_inf=%.2e\n", info.d_inf);
  EXPECT_LT(info.d_inf, 1e-6);
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

  Model problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, /*use_chordal=*/false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);
  CompiledModel cm(*kkt, cost_rhs);

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

  auto result = GeodesicCenter(cm, W, 1.0, 100, 1e-10);
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
  Model lp_problem;
  {
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < m; ++i)
      for (int j = 0; j < n; ++j)
        trips.emplace_back(i, j, A_dense(i, j));
    Eigen::SparseMatrix<double> A(m, n);
    A.setFromTriplets(trips.begin(), trips.end());
    lp_problem.AddLinearConstraint(A, b, vars);
  }

  lp_problem.SetLinearCost(c);
  auto lp_solver = Solver::Build(lp_problem);
  auto* lp_kkt = lp_solver.kkt();
  auto lp_cost = lp_solver.MakeCostRHS();
  CompiledModel lp_cm(*lp_kkt, lp_cost);
  RowSpace lp_W = lp_kkt->MakeRowSpace();
  setFromVector(lp_W, VectorXd::Ones(m) + pert);
  auto lp_result = GeodesicCenter(lp_cm, lp_W, 1.0, 20, 1e-12, true);

  // --- SDP path (diagonal matrices, no chordal) ---
  Model sdp_problem;
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
  sdp_problem.SetLinearCost(c);

  auto sdp_solver = Solver::Build(sdp_problem);
  auto* sdp_kkt = sdp_solver.kkt();
  auto sdp_cost = sdp_solver.MakeCostRHS();
  CompiledModel sdp_cm(*sdp_kkt, sdp_cost);
  RowSpace sdp_W = sdp_kkt->MakeRowSpace();
  // Set W = diag(ones + pert) as an m×m matrix.
  {
    int m2 = m * m;
    for (int i = 0; i < m2; ++i) sdp_W.segment_ptr(0)[i] = 0;
    for (int i = 0; i < m; ++i) sdp_W.segment_ptr(0)[i * m + i] = 1.0 + pert(i);
  }
  auto sdp_result = GeodesicCenter(sdp_cm, sdp_W, 1.0, 20, 1e-12, true);

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
  auto lp_lp = SolveGeodesicLP(lp_cm, lp_W, lp_iters, 0, 1e-12, true);
  auto sdp_lp = SolveGeodesicLP(sdp_cm, sdp_W, lp_iters, 0, 1e-12, true);

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

  Model problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, /*use_chordal=*/false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);
  CompiledModel cm(*kkt, cost_rhs);

  // Center at k=1 from W=I (should be 1 iter since already centered).
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  auto r0 = GeodesicCenter(cm, W, 1.0, 50, 1e-10);
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
  auto r1 = GeodesicCenter(cm, W, 1.0, 50, 1e-10);
  printf("k=1 (perturbed): %d iters, d_inf=%.2e\n",
         r1.iterations, r1.d_inf_norm);
  EXPECT_LT(r1.d_inf_norm, 1e-8);

  // One line search + re-center should work.
  double k = 1.0;
  double k_new = GeodesicLineSearch(cm, W);
  printf("line search: k_new=%.4f\n", k_new);
  EXPECT_GT(k_new, k);
  k = k_new;
  auto r2 = GeodesicCenter(cm, W, k, 50, 1e-10);
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

  Model problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, /*use_chordal=*/false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);
  CompiledModel cm(*kkt, cost_rhs);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveGeodesicLP(cm, W, 30, 0, 1e-6, true);
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

  Model problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
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
  CompiledModel cm(*kkt, cost_rhs);
  for (int iter = 0; iter < 20; ++iter) {
    auto info = HybridCenteringStep(cm, W, r);
    printf("  %3d  %12.4e  %12.4e  %12.4e\n",
           iter, info.gap, info.d_inf, info.d_sq);
    if (info.d_inf < 1e-10) break;
  }

  // Verify convergence: one more direction computation (no step).
  {
    cm.SetScaling(W);
    cm.AssembleAndFactor();
    RowSpace d = cm.MakeRowSpace();
    RowSpace delta = cm.MakeRowSpace();
    const RowSpace b_aff = cm.GetAffineTerm();
    auto info = ComputeHybridDirection(cm, b_aff, W, r, d, delta);
    printf("\nFinal: d_inf=%.2e, gap=%.2e\n", info.d_inf, info.gap);
    EXPECT_LT(info.d_inf, 1e-6);
  }
}

// =====================================================================
// SOC tests for the geodesic IPM.
// =====================================================================

// Helper: build a SOC problem with identity on the central path.
// Constraint: A x + b in SOC, where b = (1, 0, ..., 0) = identity.
// Cost c_j = A_{0,j} (scalar row) so W=identity at k=1 is centered.
struct SOCTestProblem {
  Model problem;
  VectorXd c;
  int n_soc;  // 1 + vec_dim
  int p;      // variables
};

// Note: p must be <= vec_dim + 1 (SOC dimension) to avoid a
// pre-existing crash in the tree solver with underdetermined dense constraints.
SOCTestProblem MakeSOCTestProblem(int vec_dim, int p, int seed) {
  srand(seed);
  int n_soc = 1 + vec_dim;
  MatrixXd A_dense = MatrixXd::Random(n_soc, p);

  // b = identity SOC element (1, 0, ..., 0).
  VectorXd b = VectorXd::Zero(n_soc);
  b(0) = 1.0;

  // c_j = A_{0,j} (central path at W=I, k=1).
  // Tests that need a nontrivial LP should add their own perturbation.
  VectorXd c = A_dense.row(0).transpose();

  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < n_soc; ++i)
    for (int j = 0; j < p; ++j)
      if (std::abs(A_dense(i, j)) > 1e-14)
        trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(n_soc, p);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddSOCConstraint(A, b, vars);
  problem.SetLinearCost(c);

  return {std::move(problem), c, n_soc, p};
}

// Centering at k=1: W=identity is the fixed point.
TEST(GeodesicSOC, CenterConvergence) {
  auto tp = MakeSOCTestProblem(4, 5, 42);
  auto solver = Solver::Build(tp.problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(tp.c);
  CompiledModel cm(*kkt, cost_rhs);

  // Perturb W from identity.
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  {
    VectorXd w0(tp.n_soc);
    w0(0) = 1.2;
    w0.tail(tp.n_soc - 1) = 0.1 * VectorXd::Random(tp.n_soc - 1);
    setFromVector(W, w0);
  }

  auto result = GeodesicCenter(cm, W, 1.0, 50, 1e-10, true);
  printf("SOC Center: %d iters, d_inf=%.2e\n",
         result.iterations, result.d_inf_norm);
  EXPECT_LT(result.d_inf_norm, 1e-8);
}

// Full LP path: centering + line search.
// Uses perturbed cost so the LP is nontrivial.
TEST(GeodesicSOC, LP) {
  auto tp = MakeSOCTestProblem(4, 5, 42);
  // Perturb cost from the central-path value to create a nontrivial LP.
  srand(77);
  VectorXd c_lp = tp.c + 0.5 * VectorXd::Random(tp.p);
  tp.problem.SetLinearCost(c_lp);
  auto solver = Solver::Build(tp.problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c_lp);
  CompiledModel cm(*kkt, cost_rhs);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveGeodesicLP(cm, W, 20, 0, 1e-6, true);
  printf("SOC LP: %d fac, mu=%.2e, d_inf=%.2e\n",
         result.total_factorizations, result.mu, result.d_inf_norm);

  // k should increase and d_inf ≈ 1 when k changes.
  for (int i = 1; i < result.iterations; ++i) {
    if (result.iter_stats[i].mu < result.iter_stats[i-1].mu)
      EXPECT_NEAR(result.iter_stats[i].d_inf, 1.0, 0.01);
  }
  EXPECT_GT(1.0 / std::sqrt(result.mu), 1.5);
}

// Hybrid centering: perturb r, verify d → 0.
TEST(GeodesicSOC, HybridCenteringLoop) {
  auto tp = MakeSOCTestProblem(4, 5, 42);
  auto solver = Solver::Build(tp.problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(tp.c);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // Perturb r from identity.
  RowSpace r = kkt->MakeRowSpace();
  setOnes(r);
  {
    VectorXd r0(tp.n_soc);
    r0(0) = 0.8;
    r0.tail(tp.n_soc - 1) = 0.1 * VectorXd::Random(tp.n_soc - 1);
    setFromVector(r, r0);
  }

  printf("  %3s  %12s  %12s\n", "iter", "gap", "d_inf");
  printf("  %s\n", std::string(30, '-').c_str());
  CompiledModel cm(*kkt, cost_rhs);
  for (int iter = 0; iter < 20; ++iter) {
    auto info = HybridCenteringStep(cm, W, r);
    printf("  %3d  %12.4e  %12.4e\n", iter, info.gap, info.d_inf);
    if (info.d_inf < 1e-10) break;
  }

  cm.SetScaling(W);
  cm.AssembleAndFactor();
  RowSpace d = cm.MakeRowSpace();
  RowSpace delta = cm.MakeRowSpace();
  const RowSpace b_aff = cm.GetAffineTerm();
  auto info = ComputeHybridDirection(cm, b_aff, W, r, d, delta);
  printf("Final: d_inf=%.2e\n", info.d_inf);
  EXPECT_LT(info.d_inf, 1e-6);
}

// Full hybrid solve.
TEST(GeodesicSOC, Hybrid) {
  auto tp = MakeSOCTestProblem(4, 5, 42);
  auto solver = Solver::Build(tp.problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(tp.c);
  CompiledModel cm(*kkt, cost_rhs);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveGeodesicHybrid(cm, W, 50, 1e-8);
  printf("SOC Hybrid: %d fac, %d sol, gap=%.2e\n",
         result.total_factorizations, result.total_solves,
         result.complementarity);
  EXPECT_LT(std::abs(result.complementarity), 1e-4);
}

// Two PSD constraints on interleaved variables: vars {1,3,5} and {2,4,6}.
// Tests that the permutation logic in PSDBlockAssembler correctly maps
// between global variable indices and internal column ordering.
TEST(GeodesicSDP, InterleavedVariablesRandom) {
  srand(123);
  const int n = 2;
  const int p = 3;

  auto make_sym = [&](int dim) -> MatrixXd {
    MatrixXd M = MatrixXd::Random(dim, dim);
    return MatrixXd(0.5 * (M + M.transpose()));
  };

  std::vector<Eigen::SparseMatrix<double>> A_list1, A_list2;
  for (int k = 0; k < p; ++k) {
    A_list1.push_back(toSparse(make_sym(n)));
    A_list2.push_back(toSparse(make_sym(n)));
  }
  Eigen::SparseMatrix<double> I2 = toSparse(MatrixXd::Identity(n, n));

  std::vector<int> vars1 = {1, 3, 5};
  std::vector<int> vars2 = {2, 4, 6};

  VectorXd c = VectorXd::Zero(7);
  for (int k = 0; k < p; ++k) {
    c(vars1[k]) = MatrixXd(A_list1[k]).trace();
    c(vars2[k]) = MatrixXd(A_list2[k]).trace();
  }

  Model problem;
  problem.AddPSDConstraint(A_list1, I2, vars1, /*use_chordal=*/false);
  problem.AddPSDConstraint(A_list2, I2, vars2, /*use_chordal=*/false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto cost_rhs = solver.MakeCostRHS();
  CompiledModel cm(*kkt, cost_rhs);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // At W=I, k=1 with cost = A^T(I), d should be ~0.
  auto result = GeodesicCenter(cm, W, 1.0, 10, 1e-10, true);
  printf("Interleaved PSD: %d iters, d_inf=%.2e\n",
         result.iterations, result.d_inf_norm);
  EXPECT_LT(result.d_inf_norm, 1e-8);

  // W should stay at I.
  RowSpace W_ones = kkt->MakeRowSpace();
  setOnes(W_ones);
  double w_err = normInf(addScaled(W, W_ones, 1.0, -1.0));
  printf("  ||W - I||_inf = %.2e\n", w_err);
  EXPECT_LT(w_err, 1e-6);

  // Now perturb and center at k=sqrt(2) (mu=0.5).
  setOnes(W);
  {
    int n2 = n * n;
    MatrixXd pert = MatrixXd::Zero(n, n);
    pert(0, 1) = pert(1, 0) = 0.03;
    MatrixXd W0 = MatrixXd::Identity(n, n) + pert;
    // Write to both PSD segments.
    for (int seg = 0; seg < W.num_constraints(); ++seg) {
      if (W.sizes[seg] == n2) {
        for (int i = 0; i < n2; ++i) W.segment_ptr(seg)[i] = W0.data()[i];
      }
    }
  }

  auto result2 = GeodesicCenter(cm, W, std::sqrt(2.0),
                                 20, 1e-10, true);
  printf("Interleaved center mu=0.5: %d iters, d_inf=%.2e\n",
         result2.iterations, result2.d_inf_norm);
  EXPECT_LT(result2.d_inf_norm, 1e-6);
  EXPECT_LE(result2.iterations, 10);
}

// Same as above but with chordal decomposition enabled.
TEST(GeodesicSDP, InterleavedVariablesChordal) {
  srand(456);
  const int n = 3;
  const int p = 3;

  auto make_sym = [&](int dim) -> MatrixXd {
    MatrixXd M = MatrixXd::Random(dim, dim);
    return MatrixXd(0.5 * (M + M.transpose()));
  };

  std::vector<Eigen::SparseMatrix<double>> A_list1, A_list2;
  for (int k = 0; k < p; ++k) {
    A_list1.push_back(toSparse(make_sym(n)));
    A_list2.push_back(toSparse(make_sym(n)));
  }
  Eigen::SparseMatrix<double> In = toSparse(MatrixXd::Identity(n, n));

  std::vector<int> vars1 = {1, 3, 5};
  std::vector<int> vars2 = {2, 4, 6};

  VectorXd c = VectorXd::Zero(7);
  for (int k = 0; k < p; ++k) {
    c(vars1[k]) = MatrixXd(A_list1[k]).trace();
    c(vars2[k]) = MatrixXd(A_list2[k]).trace();
  }

  Model problem;
  problem.AddPSDConstraint(A_list1, In, vars1, /*use_chordal=*/true);
  problem.AddPSDConstraint(A_list2, In, vars2, /*use_chordal=*/true);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto cost_rhs = solver.MakeCostRHS();
  CompiledModel cm(*kkt, cost_rhs);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = GeodesicCenter(cm, W, 1.0, 10, 1e-10);
  printf("Interleaved chordal: %d iters, d_inf=%.2e\n",
         result.iterations, result.d_inf_norm);
  EXPECT_LT(result.d_inf_norm, 1e-8);

  // Perturb and center at mu=0.5.
  setOnes(W);
  auto result2 = GeodesicCenter(cm, W, std::sqrt(2.0),
                                 20, 1e-10);
  printf("Interleaved chordal center mu=0.5: %d iters, d_inf=%.2e\n",
         result2.iterations, result2.d_inf_norm);
  EXPECT_LT(result2.d_inf_norm, 1e-6);
  EXPECT_LE(result2.iterations, 10);
}

// Mixed PSD + nonneg constraints sharing variables.
// Mimics the structure of buck3/trto3/vibra3 (PSD blocks + diagonal block).
TEST(GeodesicSDP, MixedPSDNonneg) {
  const int n_psd = 2;  // 2×2 PSD block
  const int n_vars = 3;
  const int m_nn = 2;   // 2 nonneg constraints

  // PSD block: A_k on vars {0,1,2}, B = I.
  MatrixXd E00 = MatrixXd::Zero(n_psd, n_psd); E00(0, 0) = 1;
  MatrixXd E01 = MatrixXd::Zero(n_psd, n_psd); E01(0, 1) = E01(1, 0) = 1;
  MatrixXd E11 = MatrixXd::Zero(n_psd, n_psd); E11(1, 1) = 1;
  std::vector<Eigen::SparseMatrix<double>> A_psd = {
      toSparse(E00), toSparse(E01), toSparse(E11)};
  Eigen::SparseMatrix<double> I2 = toSparse(MatrixXd::Identity(n_psd, n_psd));

  // Nonneg block: 2 constraints on vars {0,1,2}, b = ones.
  MatrixXd A_nn_dense(m_nn, n_vars);
  A_nn_dense << 0.5, 0.3, 0.2,
                0.1, 0.4, 0.5;
  Eigen::SparseMatrix<double> A_nn = toSparse(A_nn_dense);
  VectorXd b_nn = VectorXd::Ones(m_nn);

  // Cost = A_psd^T(I) + A_nn^T(ones) so that (W=I, k=1) is on the central path.
  // A_psd^T(I)_j = trace(A_j · I) = trace(A_j).
  // A_nn^T(ones)_j = sum of column j of A_nn.
  VectorXd c = VectorXd::Zero(n_vars);
  for (int k = 0; k < n_vars; ++k)
    c(k) = MatrixXd(A_psd[k]).trace();
  c += A_nn.transpose() * VectorXd::Ones(m_nn);

  std::vector<int> vars(n_vars);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddPSDConstraint(A_psd, I2, vars, /*use_chordal=*/false);
  problem.AddLinearConstraint(A_nn, b_nn, vars);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);
  CompiledModel cm(*kkt, cost_rhs);

  // At W=I, k=1: d should be ~0.
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  printf("Mixed PSD+nonneg: W segments=%d, total_rows=%d\n",
         W.num_constraints(), W.total_rows());
  for (int i = 0; i < W.num_constraints(); ++i)
    printf("  seg %d: size=%d\n", i, W.sizes[i]);

  auto result = GeodesicCenter(cm, W, 1.0, 10, 1e-10, true);
  printf("Center at k=1: %d iters, d_inf=%.2e\n",
         result.iterations, result.d_inf_norm);
  EXPECT_LT(result.d_inf_norm, 1e-8);

  // Center at mu=0.5.
  setOnes(W);
  auto result2 = GeodesicCenter(cm, W, std::sqrt(2.0),
                                 20, 1e-10, true);
  printf("Center at mu=0.5: %d iters, d_inf=%.2e\n",
         result2.iterations, result2.d_inf_norm);
  EXPECT_LT(result2.d_inf_norm, 1e-6);
  EXPECT_LE(result2.iterations, 10);
}

// Same as above but with PSD on a subset of variables.
TEST(GeodesicSDP, MixedPSDNonnegDisjoint) {
  const int n_psd = 2;

  // PSD on vars {0,1}, nonneg on vars {2,3}.
  MatrixXd E00 = MatrixXd::Zero(n_psd, n_psd); E00(0, 0) = 1;
  MatrixXd E11 = MatrixXd::Zero(n_psd, n_psd); E11(1, 1) = 1;
  std::vector<Eigen::SparseMatrix<double>> A_psd = {
      toSparse(E00), toSparse(E11)};
  Eigen::SparseMatrix<double> I2 = toSparse(MatrixXd::Identity(n_psd, n_psd));

  // Nonneg: 3 constraints on vars {2,3}.
  MatrixXd A_nn_dense(3, 2);
  A_nn_dense << 1.0, 0.5,
                0.5, 1.0,
                0.3, 0.7;
  Eigen::SparseMatrix<double> A_nn = toSparse(A_nn_dense);
  VectorXd b_nn = VectorXd::Ones(3);

  // Cost = A^T(I) for each constraint type.
  VectorXd c = VectorXd::Zero(4);
  c(0) = MatrixXd(A_psd[0]).trace();  // trace(E00) = 1
  c(1) = MatrixXd(A_psd[1]).trace();  // trace(E11) = 1
  c.tail(2) = A_nn.transpose() * VectorXd::Ones(3);

  Model problem;
  std::vector<int> psd_vars = {0, 1};
  std::vector<int> nn_vars = {2, 3};
  problem.AddPSDConstraint(A_psd, I2, psd_vars, /*use_chordal=*/false);
  problem.AddLinearConstraint(A_nn, b_nn, nn_vars);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);
  CompiledModel cm(*kkt, cost_rhs);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = GeodesicCenter(cm, W, 1.0, 10, 1e-10, true);
  printf("Mixed disjoint center k=1: %d iters, d_inf=%.2e\n",
         result.iterations, result.d_inf_norm);
  EXPECT_LT(result.d_inf_norm, 1e-8);

  // Center at mu=0.5.
  setOnes(W);
  auto result2 = GeodesicCenter(cm, W, std::sqrt(2.0),
                                 20, 1e-10, true);
  printf("Mixed disjoint center mu=0.5: %d iters, d_inf=%.2e\n",
         result2.iterations, result2.d_inf_norm);
  EXPECT_LT(result2.d_inf_norm, 1e-6);
  EXPECT_LE(result2.iterations, 10);
}

// =====================================================================
// Spin factor test: SOC(1+n) ≅ face of PSD(k) via Clifford embedding.
// Build the same problem as SOC and SDP, verify iterations are isomorphic.
// =====================================================================

// Embed SOC element s = (s0, s1, ..., sn) into Sym(k) via gamma matrices.
// For n=2, k=2: phi(s0, s1, s2) = s0*I + s1*gamma1 + s2*gamma2
//   gamma1 = [1,0;0,-1], gamma2 = [0,1;1,0]
//   phi(s) = [s0+s1, s2; s2, s0-s1]
static Eigen::SparseMatrix<double> ToSparseMat(const MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i, j)) > 1e-15)
        trips.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(trips.begin(), trips.end());
  return S;
}

// Gamma matrices for spin factor V_{1,2} embedded in Sym(2).
static MatrixXd Gamma1_2x2() {
  MatrixXd g(2, 2);
  g << 1, 0, 0, -1;
  return g;
}

static MatrixXd Gamma2_2x2() {
  MatrixXd g(2, 2);
  g << 0, 1, 1, 0;
  return g;
}

// phi: R^3 (SOC element) -> Sym(2): phi(t, x1, x2) = t*I + x1*gamma1 + x2*gamma2
static MatrixXd SpinEmbed(double t, double x1, double x2) {
  return t * MatrixXd::Identity(2, 2) + x1 * Gamma1_2x2() + x2 * Gamma2_2x2();
}

// Embed a column of the SOC constraint matrix into a 2x2 PSD constraint matrix.
static MatrixXd SpinEmbedColumn(const VectorXd& col) {
  return SpinEmbed(col(0), col(1), col(2));
}

TEST(SpinFactor, SOCvsSDP_GeodesicLP) {
  // SOC in R^3 (vec_dim=2), p=2 variables.
  const int vec_dim = 2, p = 2, seed = 77;
  srand(seed);
  const int n_soc = 1 + vec_dim;

  MatrixXd A_dense = MatrixXd::Random(n_soc, p);
  VectorXd b_vec = VectorXd::Zero(n_soc);
  b_vec(0) = 1.0;  // SOC identity
  VectorXd c = A_dense.row(0).transpose();  // centered at W=I, k=1

  // --- Build SOC model ---
  Eigen::SparseMatrix<double> A_soc = ToSparseMat(A_dense);
  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model soc_model;
  soc_model.AddSOCConstraint(A_soc, b_vec, vars);
  soc_model.SetLinearCost(c);

  // --- Build SDP model via spin factor embedding ---
  // PSD constraint: B + sum_j x_j * A_j >= 0 where
  //   B = phi(b), A_j = phi(column_j of A)
  const int k = 2;  // Sym(2) for n=2
  MatrixXd B_psd = SpinEmbed(b_vec(0), b_vec(1), b_vec(2));
  std::vector<Eigen::SparseMatrix<double>> A_psd_list;
  for (int j = 0; j < p; ++j) {
    A_psd_list.push_back(ToSparseMat(SpinEmbedColumn(A_dense.col(j))));
  }

  Model sdp_model;
  sdp_model.AddPSDConstraint(A_psd_list, ToSparseMat(B_psd), vars,
                              /*use_chordal=*/false);
  sdp_model.SetLinearCost(c);

  // --- Solve both with GeodesicLP ---
  auto soc_solver = Solver::Build(soc_model);
  auto* soc_kkt = soc_solver.kkt();
  auto soc_cost = soc_kkt->MakeSolverRHS();
  soc_cost = soc_kkt->MakeBlockVariable(c);
  CompiledModel soc_cm(*soc_kkt, soc_cost);
  RowSpace soc_W = soc_cm.MakeRowSpace();
  setOnes(soc_W);
  auto soc_result = SolveGeodesicLP(soc_cm, soc_W, 30, 0, 1e-8, true);

  auto sdp_solver = Solver::Build(sdp_model);
  auto* sdp_kkt = sdp_solver.kkt();
  auto sdp_cost = sdp_kkt->MakeSolverRHS();
  sdp_cost = sdp_kkt->MakeBlockVariable(c);
  CompiledModel sdp_cm(*sdp_kkt, sdp_cost);
  RowSpace sdp_W = sdp_cm.MakeRowSpace();
  setOnes(sdp_W);
  auto sdp_result = SolveGeodesicLP(sdp_cm, sdp_W, 30, 0, 1e-8, true);

  printf("\n=== Spin Factor: SOC vs SDP (GeodesicLP) ===\n");
  printf("SOC: %d fac, %d sol, mu=%.6e, d_inf=%.6e\n",
         soc_result.total_factorizations, soc_result.total_solves,
         soc_result.mu, soc_result.d_inf_norm);
  printf("SDP: %d fac, %d sol, mu=%.6e, d_inf=%.6e\n",
         sdp_result.total_factorizations, sdp_result.total_solves,
         sdp_result.mu, sdp_result.d_inf_norm);

  // Verify same primal solution.
  ASSERT_EQ(soc_result.x.size(), sdp_result.x.size());
  double x_diff = (soc_result.x - sdp_result.x).norm();
  printf("||x_soc - x_sdp|| = %.2e\n", x_diff);
  EXPECT_LT(x_diff, 1e-6);

  // Verify per-iteration d_inf and k match (the Newton steps are isomorphic).
  // The gap/complementarity may differ slightly due to trace form conventions,
  // so one solver may take one extra iteration to cross the tolerance.
  int n_common = std::min(soc_result.iter_stats.size(),
                          sdp_result.iter_stats.size());
  printf("\n  %3s  %12s %12s  %12s %12s\n",
         "it", "soc_d_inf", "sdp_d_inf", "soc_gap", "sdp_gap");
  for (int i = 0; i < n_common; ++i) {
    const auto& ss = soc_result.iter_stats[i];
    const auto& sd = sdp_result.iter_stats[i];
    printf("  %3d  %12.6e %12.6e  %12.6e %12.6e\n",
           i, ss.d_inf, sd.d_inf, ss.complementarity, sd.complementarity);
    EXPECT_NEAR(ss.d_inf, sd.d_inf, 1e-8)
        << "d_inf mismatch at iteration " << i;
  }

  // Factorization counts should be within 1 (stopping criterion sensitivity).
  EXPECT_LE(std::abs(soc_result.total_factorizations -
                     sdp_result.total_factorizations), 1);
}

}  // namespace
}  // namespace conex
