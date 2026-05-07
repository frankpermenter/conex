#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/common/compiled_model.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
#include "conex/common/psd_cone_ops.h"
#include "conex/common/soc_cone_ops.h"
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

// Spin factor embedding: SOC(1+n) -> Sym(k) via Clifford algebra.
// phi(s) = s_0 * I_k + sum_i s_i * gamma_i
// where gamma_i are anti-commuting symmetric involutions.

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

// Build gamma matrices for the spin factor V_{1,n}.
// Returns n anti-commuting symmetric k×k matrices with gamma_i^2 = I.
//   n=1: k=2, gamma1 = diag(1,-1)
//   n=2: k=2, gamma1 = diag(1,-1), gamma2 = [0,1;1,0]
//   n=3: k=4, gamma1 = sigma3 x I2, gamma2 = sigma1 x I2, gamma3 = sigma1 x sigma3
static std::vector<MatrixXd> BuildGammaMatrices(int n) {
  std::vector<MatrixXd> gammas;
  if (n >= 1) {
    // sigma3 (or sigma3 x I for n>=3)
    if (n <= 2) {
      MatrixXd g(2, 2);
      g << 1, 0, 0, -1;
      gammas.push_back(g);
    } else {
      // sigma3 x I2
      MatrixXd g = MatrixXd::Zero(4, 4);
      g(0,0) = 1; g(1,1) = 1; g(2,2) = -1; g(3,3) = -1;
      gammas.push_back(g);
    }
  }
  if (n >= 2) {
    if (n <= 2) {
      MatrixXd g(2, 2);
      g << 0, 1, 1, 0;  // sigma1
      gammas.push_back(g);
    } else {
      // sigma1 x sigma3
      MatrixXd g = MatrixXd::Zero(4, 4);
      g(0,2) = 1; g(1,3) = -1; g(2,0) = 1; g(3,1) = -1;
      gammas.push_back(g);
    }
  }
  if (n >= 3) {
    // sigma1 x sigma1
    MatrixXd g = MatrixXd::Zero(4, 4);
    g(0,3) = 1; g(1,2) = 1; g(2,1) = 1; g(3,0) = 1;
    gammas.push_back(g);
  }
  return gammas;
}

static int SpinEmbedDim(int n) { return (n <= 2) ? 2 : 4; }

// phi(s) = s_0 * I_k + sum_i s_i * gamma_i
static MatrixXd SpinEmbedVec(const VectorXd& s,
                              const std::vector<MatrixXd>& gammas) {
  int k = gammas[0].rows();
  MatrixXd result = s(0) * MatrixXd::Identity(k, k);
  for (int i = 0; i < (int)gammas.size(); ++i)
    result += s(i + 1) * gammas[i];
  return result;
}

// =====================================================================
// Low-level ConeOps isomorphism test: SOC ops vs PSD ops via spin factor.
// For each operation, apply it in the SOC algebra and in the embedded
// PSD subalgebra, verify the results match under phi.
// =====================================================================

// Embed SOC vector into flat PSD storage (column-major k×k).
static VectorXd Embed(const VectorXd& s, const std::vector<MatrixXd>& gammas) {
  MatrixXd M = SpinEmbedVec(s, gammas);
  return Eigen::Map<VectorXd>(M.data(), M.size());
}

// Extract SOC vector from flat PSD storage by projecting onto {I, gamma_i}.
// s_0 = tr(M) / k, s_i = tr(gamma_i * M) / k.
static VectorXd Extract(const VectorXd& flat, int k,
                         const std::vector<MatrixXd>& gammas) {
  Eigen::Map<const MatrixXd> M(flat.data(), k, k);
  int n = 1 + gammas.size();
  VectorXd s(n);
  s(0) = M.trace() / k;
  for (int i = 0; i < (int)gammas.size(); ++i)
    s(i + 1) = (gammas[i].cwiseProduct(M)).sum() / k;
  return s;
}

static void RunConeOpsIsomorphismTest(int vec_dim) {
  const int n_soc = 1 + vec_dim;
  auto gammas = BuildGammaMatrices(vec_dim);
  int k = SpinEmbedDim(vec_dim);
  int psd_size = k * k;
  const auto& soc = EuclideanJordanAlgebra::socConeOps();
  const auto& psd = EuclideanJordanAlgebra::psdConeOps();
  const double tol = 1e-12;

  // Random SOC elements in the interior: a = (t, x) with t > ||x||.
  srand(42 + vec_dim);
  auto make_interior = [&]() {
    VectorXd s(n_soc);
    s.tail(vec_dim) = 0.3 * VectorXd::Random(vec_dim);
    s(0) = 1.0 + s.tail(vec_dim).norm();
    return s;
  };
  VectorXd a = make_interior();
  VectorXd b = make_interior();
  VectorXd a_psd = Embed(a, gammas);
  VectorXd b_psd = Embed(b, gammas);

  printf("  Spin factor ConeOps isomorphism test: n=%d (SOC dim %d, PSD %dx%d)\n",
         vec_dim, n_soc, k, k);

  // --- setIdentity ---
  {
    VectorXd e_soc(n_soc), e_psd(psd_size);
    soc.setIdentity(e_soc.data(), n_soc);
    psd.setIdentity(e_psd.data(), psd_size);
    VectorXd e_soc_via_psd = Extract(e_psd, k, gammas);
    double err = (e_soc - e_soc_via_psd).norm();
    printf("    setIdentity: err=%.2e\n", err);
    EXPECT_LT(err, tol);
  }

  // --- product ---
  {
    VectorXd ab_soc(n_soc), ab_psd(psd_size);
    soc.product(ab_soc.data(), a.data(), b.data(), n_soc);
    psd.product(ab_psd.data(), a_psd.data(), b_psd.data(), psd_size);
    VectorXd ab_via_psd = Extract(ab_psd, k, gammas);
    double err = (ab_soc - ab_via_psd).norm();
    printf("    product: err=%.2e\n", err);
    EXPECT_LT(err, tol);
  }

  // --- quadraticRepresentation ---
  {
    VectorXd qa_soc(n_soc), qa_psd(psd_size);
    soc.quadraticRepresentation(qa_soc.data(), a.data(), b.data(), n_soc);
    psd.quadraticRepresentation(qa_psd.data(), a_psd.data(), b_psd.data(), psd_size);
    VectorXd qa_via_psd = Extract(qa_psd, k, gammas);
    double err = (qa_soc - qa_via_psd).norm();
    printf("    quadraticRepresentation: err=%.2e\n", err);
    EXPECT_LT(err, tol);
  }

  // --- sqrt ---
  {
    VectorXd sa_soc(n_soc), sa_psd(psd_size);
    soc.sqrt(sa_soc.data(), a.data(), n_soc);
    psd.sqrt(sa_psd.data(), a_psd.data(), psd_size);
    VectorXd sa_via_psd = Extract(sa_psd, k, gammas);
    double err = (sa_soc - sa_via_psd).norm();
    printf("    sqrt: err=%.2e\n", err);
    EXPECT_LT(err, tol);
  }

  // --- normInf ---
  {
    double ni_soc = soc.normInf(a.data(), n_soc);
    double ni_psd = psd.normInf(a_psd.data(), psd_size);
    double err = std::abs(ni_soc - ni_psd);
    printf("    normInf: soc=%.6e psd=%.6e err=%.2e\n", ni_soc, ni_psd, err);
    EXPECT_LT(err, tol);
  }

  // --- squaredNorm ---
  // SOC: sum(a_i^2) with trace form 2*(t^2 + ||x||^2).
  // PSD: tr(A^2) = tr(phi(a)^2). For spin factor: tr(phi(a)^2) = k*(t^2+||x||^2).
  // So PSD squaredNorm = (k/2) * SOC squaredNorm.
  {
    double sn_soc = soc.squaredNorm(a.data(), n_soc);
    double sn_psd = psd.squaredNorm(a_psd.data(), psd_size);
    double ratio = sn_psd / sn_soc;
    double expected_ratio = static_cast<double>(k) / 2.0;
    printf("    squaredNorm: soc=%.6e psd=%.6e ratio=%.4f (expected %.1f)\n",
           sn_soc, sn_psd, ratio, expected_ratio);
    EXPECT_NEAR(ratio, expected_ratio, tol);
  }

  // --- dot ---
  // Same scaling as squaredNorm: psd_dot = (k/2) * soc_dot.
  {
    double d_soc = soc.dot(a.data(), b.data(), n_soc);
    double d_psd = psd.dot(a_psd.data(), b_psd.data(), psd_size);
    double ratio = d_psd / d_soc;
    double expected_ratio = static_cast<double>(k) / 2.0;
    printf("    dot: soc=%.6e psd=%.6e ratio=%.4f (expected %.1f)\n",
           d_soc, d_psd, ratio, expected_ratio);
    EXPECT_NEAR(ratio, expected_ratio, tol);
  }

  // --- minEigenvalue ---
  {
    double me_soc = soc.minEigenvalue(a.data(), n_soc);
    double me_psd = psd.minEigenvalue(a_psd.data(), psd_size);
    double err = std::abs(me_soc - me_psd);
    printf("    minEigenvalue: soc=%.6e psd=%.6e err=%.2e\n", me_soc, me_psd, err);
    EXPECT_LT(err, tol);
  }

  // --- abs ---
  {
    // Use an element with a negative eigenvalue.
    VectorXd c_neg(n_soc);
    c_neg(0) = 0.5;
    c_neg.tail(vec_dim).setZero();
    if (vec_dim > 0) c_neg(1) = 0.8;  // eigenvalues: 1.3, -0.3
    VectorXd c_neg_psd = Embed(c_neg, gammas);
    VectorXd abs_soc(n_soc), abs_psd(psd_size);
    soc.abs(abs_soc.data(), c_neg.data(), n_soc);
    psd.abs(abs_psd.data(), c_neg_psd.data(), psd_size);
    VectorXd abs_via_psd = Extract(abs_psd, k, gammas);
    double err = (abs_soc - abs_via_psd).norm();
    printf("    abs: err=%.2e\n", err);
    EXPECT_LT(err, tol);
  }

  // --- solveLyapunovForD ---
  {
    // r must be interior. Use a as r, b as delta.
    VectorXd d_soc(n_soc), d_psd(psd_size);
    soc.solveLyapunovForD(d_soc.data(), a.data(), b.data(), n_soc);
    psd.solveLyapunovForD(d_psd.data(), a_psd.data(), b_psd.data(), psd_size);
    VectorXd d_via_psd = Extract(d_psd, k, gammas);
    double err = (d_soc - d_via_psd).norm();
    printf("    solveLyapunovForD: err=%.2e\n", err);
    EXPECT_LT(err, tol);
  }

  // --- updateAutomorphism ---
  {
    // Start from W = a (interior), r = b (interior), direction d.
    VectorXd d_dir(n_soc);
    d_dir.setZero(); d_dir(0) = 0.1;
    if (vec_dim > 0) d_dir(1) = -0.05;
    double alpha = 0.5;

    VectorXd w_soc = a, r_soc = b;
    VectorXd w_psd = a_psd, r_psd = b_psd;
    VectorXd d_psd_dir = Embed(d_dir, gammas);

    soc.updateAutomorphism(w_soc.data(), r_soc.data(), alpha,
                           d_dir.data(), n_soc);
    psd.updateAutomorphism(w_psd.data(), r_psd.data(), alpha,
                           d_psd_dir.data(), psd_size);
    VectorXd w_via_psd = Extract(w_psd, k, gammas);
    VectorXd r_via_psd = Extract(r_psd, k, gammas);
    double w_err = (w_soc - w_via_psd).norm();
    double r_err = (r_soc - r_via_psd).norm();
    printf("    updateAutomorphism: w_err=%.2e r_err=%.2e\n", w_err, r_err);
    EXPECT_LT(w_err, 1e-10);
    EXPECT_LT(r_err, 1e-10);
  }

  // --- lineSearchK ---
  {
    VectorXd d0(n_soc), d1(n_soc);
    d0.setZero(); d0(0) = 0.3; if (vec_dim > 0) d0(1) = 0.1;
    d1.setZero(); d1(0) = 0.05; if (vec_dim > 0) d1(1) = -0.02;
    VectorXd d0_psd = Embed(d0, gammas), d1_psd = Embed(d1, gammas);
    double k_soc = soc.lineSearchK(d0.data(), d1.data(), n_soc);
    double k_psd = psd.lineSearchK(d0_psd.data(), d1_psd.data(), psd_size);
    double err = std::abs(k_soc - k_psd) / std::max(k_soc, 1.0);
    printf("    lineSearchK: soc=%.6e psd=%.6e err=%.2e\n", k_soc, k_psd, err);
    EXPECT_LT(err, 1e-10);
  }
}

TEST(SpinFactor, ConeOps_n1) { RunConeOpsIsomorphismTest(1); }
TEST(SpinFactor, ConeOps_n2) { RunConeOpsIsomorphismTest(2); }
TEST(SpinFactor, ConeOps_n3) { RunConeOpsIsomorphismTest(3); }

// Helper: build matched SOC and SDP (via spin factor) solvers for isomorphism tests.
struct SpinFactorPair {
  Solver soc_solver;
  Solver sdp_solver;
};

static SpinFactorPair BuildSpinFactorPair(int vec_dim, int p, int seed) {
  srand(seed);
  const int n_soc = 1 + vec_dim;
  auto gammas = BuildGammaMatrices(vec_dim);

  MatrixXd A_dense = MatrixXd::Random(n_soc, p);
  VectorXd b_vec = VectorXd::Zero(n_soc);
  b_vec(0) = 1.0;
  VectorXd c = A_dense.row(0).transpose();
  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model soc_model;
  soc_model.AddSOCConstraint(ToSparseMat(A_dense), b_vec, vars);
  soc_model.SetLinearCost(c);

  MatrixXd B_psd = SpinEmbedVec(b_vec, gammas);
  std::vector<Eigen::SparseMatrix<double>> A_psd_list;
  for (int j = 0; j < p; ++j)
    A_psd_list.push_back(ToSparseMat(SpinEmbedVec(A_dense.col(j), gammas)));
  Model sdp_model;
  sdp_model.AddPSDConstraint(A_psd_list, ToSparseMat(B_psd), vars, false);
  sdp_model.SetLinearCost(c);

  return {Solver::Build(soc_model), Solver::Build(sdp_model)};
}

// Compare per-iteration d_inf between two algorithm runs.
static void CheckIsomorphicIterations(const char* name,
                                       const GeodesicResult& soc_r,
                                       const GeodesicResult& sdp_r) {
  printf("\n=== %s isomorphism: SOC %d fac, SDP %d fac ===\n",
         name, soc_r.total_factorizations, sdp_r.total_factorizations);
  int n_common = std::min(soc_r.iter_stats.size(), sdp_r.iter_stats.size());
  printf("  %3s  %12s %12s %5s  %12s %12s %5s\n",
         "it", "soc_d_inf", "sdp_d_inf", "err", "soc_d_sqr", "sdp_d_sqr", "err");
  for (int i = 0; i < n_common; ++i) {
    double d_inf_err = std::abs(soc_r.iter_stats[i].d_inf -
                                sdp_r.iter_stats[i].d_inf);
    double d_sqr_err = std::abs(soc_r.iter_stats[i].d_sqr -
                                sdp_r.iter_stats[i].d_sqr);
    printf("  %3d  %12.6e %12.6e %5.0e  %12.6e %12.6e %5.0e\n",
           i, soc_r.iter_stats[i].d_inf, sdp_r.iter_stats[i].d_inf, d_inf_err,
           soc_r.iter_stats[i].d_sqr, sdp_r.iter_stats[i].d_sqr, d_sqr_err);
    EXPECT_NEAR(soc_r.iter_stats[i].d_inf, sdp_r.iter_stats[i].d_inf, 1e-8)
        << name << ": d_inf mismatch at iteration " << i;
    EXPECT_NEAR(soc_r.iter_stats[i].d_sqr, sdp_r.iter_stats[i].d_sqr, 1e-6)
        << name << ": d_sqr mismatch at iteration " << i;
  }
  EXPECT_LE(std::abs(soc_r.total_factorizations -
                     sdp_r.total_factorizations), 1)
      << name;
  if (soc_r.x.size() == sdp_r.x.size()) {
    EXPECT_LT((soc_r.x - sdp_r.x).norm(), 1e-6) << name;
  }
}

TEST(SpinFactor, GeodesicLP_Isomorphic) {
  auto [soc_s, sdp_s] = BuildSpinFactorPair(2, 2, 77);
  CompiledModel soc_cm(*soc_s.kkt(), soc_s.MakeCostRHS());
  CompiledModel sdp_cm(*sdp_s.kkt(), sdp_s.MakeCostRHS());
  RowSpace soc_W = soc_cm.MakeRowSpace(); setOnes(soc_W);
  RowSpace sdp_W = sdp_cm.MakeRowSpace(); setOnes(sdp_W);
  bool verbose = true;
  auto soc_r = SolveGeodesicLP(soc_cm, soc_W, 30, 0, 1e-8, verbose);
  auto sdp_r = SolveGeodesicLP(sdp_cm, sdp_W, 30, 0, 1e-8, verbose);
  CheckIsomorphicIterations("GeodesicLP", soc_r, sdp_r);
}

TEST(SpinFactor, GeodesicHybrid_Isomorphic) {
  auto [soc_s, sdp_s] = BuildSpinFactorPair(2, 2, 77);
  CompiledModel soc_cm(*soc_s.kkt(), soc_s.MakeCostRHS());
  CompiledModel sdp_cm(*sdp_s.kkt(), sdp_s.MakeCostRHS());
  RowSpace soc_W = soc_cm.MakeRowSpace(); setOnes(soc_W);
  RowSpace sdp_W = sdp_cm.MakeRowSpace(); setOnes(sdp_W);
  auto soc_r = SolveGeodesicHybrid(soc_cm, soc_W, 50, 1e-8);
  auto sdp_r = SolveGeodesicHybrid(sdp_cm, sdp_W, 50, 1e-8);
  CheckIsomorphicIterations("GeodesicHybrid", soc_r, sdp_r);
}

TEST(SpinFactor, ThetaContinuation_Isomorphic) {
  auto [soc_s, sdp_s] = BuildSpinFactorPair(2, 2, 77);
  CompiledModel soc_cm(*soc_s.kkt(), soc_s.MakeCostRHS());
  CompiledModel sdp_cm(*sdp_s.kkt(), sdp_s.MakeCostRHS());
  RowSpace soc_W = soc_cm.MakeRowSpace(); setOnes(soc_W);
  RowSpace sdp_W = sdp_cm.MakeRowSpace(); setOnes(sdp_W);
  auto soc_r = SolveGeodesicThetaContinuation(soc_cm, soc_W, 50, 0, 1e-8);
  auto sdp_r = SolveGeodesicThetaContinuation(sdp_cm, sdp_W, 50, 0, 1e-8);
  CheckIsomorphicIterations("ThetaContinuation", soc_r, sdp_r);
}

// Compare GeodesicBarrierLP vs GeodesicLP on a given Model.
// Checks iteration count, per-iteration stats, solution, and lambda.
static void CompareBarrierVsClassic(const char* name, const Model& model) {
  printf("\n====== %s ======\n", name);

  // Run both at 1 iteration to verify formulas are bit-identical.
  {
    auto sw = Solver::Build(model);
    auto sz = Solver::Build(model);
    auto cmw = sw.MakeCompiledModel();
    auto cmz = sz.MakeCompiledModel();
    auto rw = GeodesicLP{1e-14, 1, 0, false}.Run(cmw);
    auto rz = GeodesicBarrierLP{1e-14, 1, false}.Run(cmz);
    double lam_diff = 0, lam_norm = 0;
    for (int i = 0; i < rw.lambda.total_rows(); ++i) {
      lam_diff = std::max(lam_diff,
          std::abs(rw.lambda.col()(i) - rz.lambda.col()(i)));
      lam_norm = std::max(lam_norm, std::abs(rw.lambda.col()(i)));
    }
    printf("  1-iter lambda: diff=%.2e, norm=%.2e, rel=%.2e\n",
           lam_diff, lam_norm, lam_diff / (lam_norm + 1e-30));
    EXPECT_LT(lam_diff, 1e-14) << name << ": lambda not bit-identical at iter 1";
  }

  // Full convergence.
  auto sw = Solver::Build(model);
  auto sz = Solver::Build(model);
  auto cmw = sw.MakeCompiledModel();
  auto cmz = sz.MakeCompiledModel();
  auto rw = GeodesicLP{1e-8, 30, 0, true}.Run(cmw);
  auto rz = GeodesicBarrierLP{1e-8, 30, true}.Run(cmz);

  printf("  W-space: %d iters, gap=%.2e\n", rw.iterations, rw.complementarity);
  printf("  z-space: %d iters, gap=%.2e\n", rz.iterations, rz.complementarity);

  EXPECT_EQ(rw.iterations, rz.iterations) << name << ": iteration count";

  for (int i = 0; i < std::min(rw.iterations, rz.iterations); ++i) {
    double rel_mu = std::abs(rw.iter_stats[i].mu - rz.iter_stats[i].mu)
                    / (std::abs(rw.iter_stats[i].mu) + 1e-30);
    EXPECT_LT(rel_mu, 1e-3) << name << ": mu at iter " << i;
    double rel_gap = std::abs(rw.iter_stats[i].complementarity -
                              rz.iter_stats[i].complementarity)
                     / (std::abs(rw.iter_stats[i].complementarity) + 1e-30);
    EXPECT_LT(rel_gap, 5e-3) << name << ": gap at iter " << i;
  }

  EXPECT_LT((rw.x - rz.x).norm(), 1e-5) << name << ": solution";

  double lam_diff = 0, lam_norm = 0;
  for (int i = 0; i < rw.lambda.total_rows(); ++i) {
    lam_diff = std::max(lam_diff,
        std::abs(rw.lambda.col()(i) - rz.lambda.col()(i)));
    lam_norm = std::max(lam_norm, std::abs(rw.lambda.col()(i)));
  }
  printf("  lambda: diff=%.2e, norm=%.2e, rel=%.2e\n",
         lam_diff, lam_norm, lam_diff / (lam_norm + 1e-30));
  EXPECT_LT(lam_diff / (lam_norm + 1e-30), 1e-3) << name << ": lambda";
}

TEST(GeodesicBarrierQP, BarrierLP_LP) {
  srand(99);
  const int n = 6, m1 = 10, m2 = 8;
  MatrixXd A1 = MatrixXd::Random(m1, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m1, n);
  MatrixXd A2 = MatrixXd::Random(m2, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m2, n);
  VectorXd b1 = VectorXd::Ones(m1), b2 = VectorXd::Ones(m2);
  VectorXd c = A1.transpose() * VectorXd::Ones(m1) +
               A2.transpose() * VectorXd::Ones(m2);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A1), b1, vars);
  model.AddLinearConstraint(toSparse(A2), b2, vars);
  model.SetLinearCost(c);
  CompareBarrierVsClassic("LP", model);
}

TEST(GeodesicBarrierQP, BarrierLP_QP) {
  srand(42);
  const int n = 6, m = 12;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  // SPD quadratic cost: Q = R^T R + I.
  MatrixXd R = 0.3 * MatrixXd::Random(n, n);
  Eigen::SparseMatrix<double> Q = toSparse(R.transpose() * R +
                                            MatrixXd::Identity(n, n));
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.AddQuadraticCost(Q, vars);
  model.SetLinearCost(c);
  CompareBarrierVsClassic("QP", model);
}

TEST(GeodesicBarrierQP, BarrierLP_EqualityConstraints) {
  srand(77);
  const int n = 8, m = 15, p = 2;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  // Equality: Cx = d where C is p×n, d = C*0 = 0 (so x=0 is feasible).
  MatrixXd C_dense = MatrixXd::Random(p, n);
  VectorXd d = VectorXd::Zero(p);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.AddEqualityConstraint(toSparse(C_dense), d, vars);
  model.SetLinearCost(c);
  CompareBarrierVsClassic("LP+Equality", model);
}

TEST(GeodesicBarrierQP, BarrierLP_QPWithEquality) {
  srand(55);
  const int n = 8, m = 15, p = 2;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  MatrixXd R = 0.3 * MatrixXd::Random(n, n);
  Eigen::SparseMatrix<double> Q = toSparse(R.transpose() * R +
                                            MatrixXd::Identity(n, n));
  MatrixXd C_dense = MatrixXd::Random(p, n);
  VectorXd d = VectorXd::Zero(p);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.AddQuadraticCost(Q, vars);
  model.AddEqualityConstraint(toSparse(C_dense), d, vars);
  model.SetLinearCost(c);
  CompareBarrierVsClassic("QP+Equality", model);
}

// Compare z-space BarrierThetaContinuation vs W-space ThetaContinuation.
static void CompareThetaContBarrierVsClassic(const char* name,
                                              const Model& model) {
  printf("\n====== ThetaCont %s ======\n", name);

  auto sw = Solver::Build(model);
  auto sz = Solver::Build(model);
  auto cmw = sw.MakeCompiledModel();
  auto cmz = sz.MakeCompiledModel();

  auto rw = ThetaContinuation{1e-8, 50, 1, true}.Run(cmw);
  auto rz = GeodesicBarrierThetaContinuation{1e-8, 50, 1, true}.Run(cmz);

  printf("  W-space: %d iters, gap=%.2e, mu=%.2e\n",
         rw.iterations, rw.complementarity, rw.mu);
  printf("  z-space: %d iters, gap=%.2e, mu=%.2e\n",
         rz.iterations, rz.complementarity, rz.mu);

  // Both should converge.
  EXPECT_LT(rw.mu, 1e-6) << name << ": W-space should converge";
  EXPECT_LT(rz.mu, 1e-6) << name << ": z-space should converge";

  // Solutions should match.
  if (rw.x.size() > 0 && rz.x.size() > 0) {
    printf("  x_diff = %.2e\n", (rw.x - rz.x).norm());
    EXPECT_LT((rw.x - rz.x).norm(), 1e-4) << name << ": solution mismatch";
  }
}

TEST(GeodesicBarrierQP, BarrierThetaCont_LP) {
  srand(99);
  const int n = 6, m1 = 10, m2 = 8;
  MatrixXd A1 = MatrixXd::Random(m1, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m1, n);
  MatrixXd A2 = MatrixXd::Random(m2, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m2, n);
  VectorXd b1 = VectorXd::Ones(m1), b2 = VectorXd::Ones(m2);
  VectorXd c = A1.transpose() * VectorXd::Ones(m1) +
               A2.transpose() * VectorXd::Ones(m2);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A1), b1, vars);
  model.AddLinearConstraint(toSparse(A2), b2, vars);
  model.SetLinearCost(c);
  CompareThetaContBarrierVsClassic("LP", model);
}

TEST(GeodesicBarrierQP, BarrierThetaCont_QP) {
  srand(42);
  const int n = 6, m = 12;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  MatrixXd R = 0.3 * MatrixXd::Random(n, n);
  Eigen::SparseMatrix<double> Q = toSparse(R.transpose() * R +
                                            MatrixXd::Identity(n, n));
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.AddQuadraticCost(Q, vars);
  model.SetLinearCost(c);
  CompareThetaContBarrierVsClassic("QP", model);
}

TEST(GeodesicBarrierQP, BarrierThetaCont_EqualityConstraints) {
  srand(77);
  const int n = 8, m = 15, p = 2;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  MatrixXd C_dense = MatrixXd::Random(p, n);
  VectorXd d = VectorXd::Zero(p);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.AddEqualityConstraint(toSparse(C_dense), d, vars);
  model.SetLinearCost(c);
  CompareThetaContBarrierVsClassic("LP+Equality", model);
}

TEST(GeodesicBarrierQP, BarrierThetaCont_QPWithEquality) {
  srand(55);
  const int n = 8, m = 15, p = 2;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  MatrixXd R = 0.3 * MatrixXd::Random(n, n);
  Eigen::SparseMatrix<double> Q = toSparse(R.transpose() * R +
                                            MatrixXd::Identity(n, n));
  MatrixXd C_dense = MatrixXd::Random(p, n);
  VectorXd d = VectorXd::Zero(p);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.AddQuadraticCost(Q, vars);
  model.AddEqualityConstraint(toSparse(C_dense), d, vars);
  model.SetLinearCost(c);
  CompareThetaContBarrierVsClassic("QP+Equality", model);
}

// PSD: verify GeodesicBarrierLP matches GeodesicLP.
TEST(GeodesicBarrierQP, BarrierLP_SDP) {
  srand(42);
  const int n_psd = 3, p = 4;
  // Build a small SDP: min c^T x s.t. Σ A_i x_i + B ≽ 0.
  std::vector<Eigen::SparseMatrix<double>> A_list;
  auto make_sym = [](int n) {
    MatrixXd M = MatrixXd::Random(n, n);
    return (M + M.transpose()) / 2.0;
  };
  for (int i = 0; i < p; ++i)
    A_list.push_back(toSparse(make_sym(n_psd)));
  Eigen::SparseMatrix<double> B = toSparse(
      3.0 * MatrixXd::Identity(n_psd, n_psd));
  // Cost: c_i = trace(A_i) so x=0 is centered (slack = B = 3I).
  VectorXd c(p);
  for (int i = 0; i < p; ++i)
    c(i) = Eigen::MatrixXd(A_list[i]).trace();
  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddPSDConstraint(A_list, B, vars, false);
  model.SetLinearCost(c);
  CompareBarrierVsClassic("SDP", model);
}

// SOC: verify GeodesicBarrierLP matches GeodesicLP.
TEST(GeodesicBarrierQP, BarrierLP_SOC) {
  srand(77);
  const int n = 5, m = 4;  // SOC dim = 1+m = 5
  // SOC constraint: ||A₁x + b₁|| ≤ A₀x + b₀.
  // A is (1+m)×n, b is (1+m).
  MatrixXd A_dense = MatrixXd::Random(1 + m, n) * 0.3;
  VectorXd b_soc(1 + m);
  b_soc(0) = 2.0;  // b₀ = 2 (scalar bound)
  b_soc.tail(m).setZero();  // b₁ = 0
  // Cost: c = A₀ (so x=0 is on the central path with slack = (2, 0,...,0)).
  VectorXd c = A_dense.row(0).transpose();

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddSOCConstraint(toSparse(A_dense), b_soc, vars);
  model.SetLinearCost(c);
  CompareBarrierVsClassic("SOC", model);
}

// Compare GeodesicLP with and without Mehrotra correction.
TEST(GeodesicBarrierQP, MehrotraCorrection) {
  srand(99);
  const int n = 6, m = 14;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.SetLinearCost(c);

  // Without correction.
  auto s1 = Solver::Build(model);
  auto cm1 = s1.MakeCompiledModel();
  auto r1 = GeodesicLP{1e-10, 30, 0, true}.Run(cm1);

  // With Jacobian reuse (3 inner centering steps).
  auto s2 = Solver::Build(model);
  auto cm2 = s2.MakeCompiledModel();
  auto r2 = GeodesicLP{1e-10, 30, 1, true}.Run(cm2);

  printf("\n=== Mehrotra correction comparison ===\n");
  printf("  Without: %d iters, %d solves, gap=%.2e\n",
         r1.iterations, r1.total_solves, r1.complementarity);
  printf("  With:    %d iters, %d solves, gap=%.2e\n",
         r2.iterations, r2.total_solves, r2.complementarity);

  // Both should converge.
  EXPECT_LT(r1.complementarity, 1e-8);
  EXPECT_LT(r2.complementarity, 1e-8);

  // Compare iteration stats.
  int n_iters = std::min(r1.iterations, r2.iterations);
  printf("  iter  gap_without     gap_with       ratio\n");
  for (int i = 0; i < n_iters; ++i) {
    double g1 = r1.iter_stats[i].complementarity;
    double g2 = r2.iter_stats[i].complementarity;
    printf("  %3d   %12.4e    %12.4e    %.2f\n", i, g1, g2,
           (g1 > 0 && g2 > 0) ? g1 / g2 : 0.0);
  }
}

// Frozen-Jacobian speedup on SDP.
TEST(GeodesicBarrierQP, FrozenJacobian_SDP) {
  srand(42);
  const int n_psd = 4, p = 6;
  std::vector<Eigen::SparseMatrix<double>> A_list;
  auto make_sym = [](int n) {
    MatrixXd M = MatrixXd::Random(n, n);
    return (M + M.transpose()) / 2.0;
  };
  for (int i = 0; i < p; ++i)
    A_list.push_back(toSparse(make_sym(n_psd)));
  Eigen::SparseMatrix<double> B = toSparse(
      3.0 * MatrixXd::Identity(n_psd, n_psd));
  VectorXd c(p);
  for (int i = 0; i < p; ++i)
    c(i) = Eigen::MatrixXd(A_list[i]).trace();
  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddPSDConstraint(A_list, B, vars, false);
  model.SetLinearCost(c);

  // Baseline: GeodesicLP (no frozen-Jacobian).
  auto s1 = Solver::Build(model);
  auto cm1 = s1.MakeCompiledModel();
  auto r1 = GeodesicLP{1e-10, 30, 0, true}.Run(cm1);

  // Frozen-Jacobian (1 inner step).
  auto s2 = Solver::Build(model);
  auto cm2 = s2.MakeCompiledModel();
  auto r2 = GeodesicLP{1e-10, 30, 1, true}.Run(cm2);

  printf("\n=== SDP frozen-Jacobian comparison ===\n");
  printf("  Baseline:       %2d fac, %3d solves, gap=%.2e\n",
         r1.total_factorizations, r1.total_solves, r1.complementarity);
  printf("  Frozen-J (1):   %2d fac, %3d solves, gap=%.2e\n",
         r2.total_factorizations, r2.total_solves, r2.complementarity);
  if (r1.total_factorizations > 0) {
    printf("  Factorization reduction: %.0f%%\n",
           100.0 * (1.0 - static_cast<double>(r2.total_factorizations) /
                          r1.total_factorizations));
  }

  EXPECT_LT(r1.complementarity, 1e-8);
  EXPECT_LT(r2.complementarity, 1e-8);
  EXPECT_LT(r2.total_factorizations, r1.total_factorizations);
}

// Frozen-Jacobian speedup on LP (nonneg).
TEST(GeodesicBarrierQP, FrozenJacobian_LP) {
  srand(99);
  const int n = 6, m = 14;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.SetLinearCost(c);

  auto s1 = Solver::Build(model);
  auto cm1 = s1.MakeCompiledModel();
  auto r1 = GeodesicLP{1e-10, 30, 0, false}.Run(cm1);

  auto s2 = Solver::Build(model);
  auto cm2 = s2.MakeCompiledModel();
  auto r2 = GeodesicLP{1e-10, 30, 1, false}.Run(cm2);

  printf("\n=== LP frozen-Jacobian comparison ===\n");
  printf("  Baseline:       %2d fac, %3d solves, gap=%.2e\n",
         r1.total_factorizations, r1.total_solves, r1.complementarity);
  printf("  Frozen-J (1):   %2d fac, %3d solves, gap=%.2e\n",
         r2.total_factorizations, r2.total_solves, r2.complementarity);
  if (r1.total_factorizations > 0) {
    printf("  Factorization reduction: %.0f%%\n",
           100.0 * (1.0 - static_cast<double>(r2.total_factorizations) /
                          r1.total_factorizations));
  }

  EXPECT_LT(r1.complementarity, 1e-8);
  EXPECT_LT(r2.complementarity, 1e-8);
  EXPECT_LT(r2.total_factorizations, r1.total_factorizations);
}

// Frozen-Jacobian speedup on SOC (multiple cones for nontrivial problem).
TEST(GeodesicBarrierQP, FrozenJacobian_SOC) {
  srand(77);
  const int n = 8, soc_dim = 4, num_cones = 3;
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  // Also add nonneg constraints for boundedness.
  MatrixXd A_nn = MatrixXd::Identity(n, n);
  VectorXd b_nn = 5.0 * VectorXd::Ones(n);
  model.AddLinearConstraint(toSparse(A_nn), b_nn, vars);
  for (int k = 0; k < num_cones; ++k) {
    MatrixXd A_dense = MatrixXd::Random(soc_dim, n) * 0.3;
    VectorXd b_soc(soc_dim);
    b_soc(0) = 2.0 + k;
    b_soc.tail(soc_dim - 1).setZero();
    model.AddSOCConstraint(toSparse(A_dense), b_soc, vars);
  }
  // Cost: sum of A₀ rows so x=0 is near-centered.
  VectorXd c = VectorXd::Zero(n);
  for (int k = 0; k < num_cones; ++k)
    c += MatrixXd::Random(soc_dim, n).row(0).transpose() * 0.1;
  model.SetLinearCost(c);

  auto s1 = Solver::Build(model);
  auto cm1 = s1.MakeCompiledModel();
  auto r1 = GeodesicLP{1e-10, 30, 0, false}.Run(cm1);

  auto s2 = Solver::Build(model);
  auto cm2 = s2.MakeCompiledModel();
  auto r2 = GeodesicLP{1e-10, 30, 1, false}.Run(cm2);

  printf("\n=== SOC frozen-Jacobian comparison ===\n");
  printf("  Baseline:       %2d fac, %3d solves, gap=%.2e\n",
         r1.total_factorizations, r1.total_solves, r1.complementarity);
  printf("  Frozen-J (1):   %2d fac, %3d solves, gap=%.2e\n",
         r2.total_factorizations, r2.total_solves, r2.complementarity);
  if (r1.total_factorizations > 0) {
    printf("  Factorization reduction: %.0f%%\n",
           100.0 * (1.0 - static_cast<double>(r2.total_factorizations) /
                          r1.total_factorizations));
  }

  EXPECT_LT(r1.complementarity, 1e-8);
  EXPECT_LT(r2.complementarity, 1e-8);
  EXPECT_LE(r2.total_factorizations, r1.total_factorizations);
}

// Frozen-Jacobian speedup on ThetaContinuation (LP).
TEST(GeodesicBarrierQP, FrozenJacobian_ThetaCont_LP) {
  srand(99);
  const int n = 6, m = 14;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.SetLinearCost(c);

  // Baseline: ThetaContinuation (no frozen-Jacobian).
  auto s1 = Solver::Build(model);
  auto cm1 = s1.MakeCompiledModel();
  auto r1 = ThetaContinuation{1e-10, 50, 0, true}.Run(cm1);

  // Frozen-Jacobian (1 inner step).
  auto s2 = Solver::Build(model);
  auto cm2 = s2.MakeCompiledModel();
  auto r2 = ThetaContinuation{1e-10, 50, 1, true}.Run(cm2);

  printf("\n=== ThetaCont LP frozen-Jacobian comparison ===\n");
  printf("  Baseline:       %2d fac, %3d solves, gap=%.2e\n",
         r1.total_factorizations, r1.total_solves, r1.complementarity);
  printf("  Frozen-J (1):   %2d fac, %3d solves, gap=%.2e\n",
         r2.total_factorizations, r2.total_solves, r2.complementarity);
  if (r1.total_factorizations > 0) {
    printf("  Factorization reduction: %.0f%%\n",
           100.0 * (1.0 - static_cast<double>(r2.total_factorizations) /
                          r1.total_factorizations));
  }

  EXPECT_LT(r1.complementarity, 1e-8);
  EXPECT_LT(r2.complementarity, 1e-8);
  EXPECT_LE(r2.total_factorizations, r1.total_factorizations);
}

// Frozen-Jacobian speedup on ThetaContinuation (SDP).
TEST(GeodesicBarrierQP, FrozenJacobian_ThetaCont_SDP) {
  srand(42);
  const int n_psd = 4, p = 6;
  std::vector<Eigen::SparseMatrix<double>> A_list;
  auto make_sym = [](int n) {
    MatrixXd M = MatrixXd::Random(n, n);
    return (M + M.transpose()) / 2.0;
  };
  for (int i = 0; i < p; ++i)
    A_list.push_back(toSparse(make_sym(n_psd)));
  Eigen::SparseMatrix<double> B = toSparse(
      3.0 * MatrixXd::Identity(n_psd, n_psd));
  VectorXd c(p);
  for (int i = 0; i < p; ++i)
    c(i) = Eigen::MatrixXd(A_list[i]).trace();
  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddPSDConstraint(A_list, B, vars, false);
  model.SetLinearCost(c);

  auto s1 = Solver::Build(model);
  auto cm1 = s1.MakeCompiledModel();
  auto r1 = ThetaContinuation{1e-10, 50, 0, true}.Run(cm1);

  auto s2 = Solver::Build(model);
  auto cm2 = s2.MakeCompiledModel();
  auto r2 = ThetaContinuation{1e-10, 50, 1, true}.Run(cm2);

  printf("\n=== ThetaCont SDP frozen-Jacobian comparison ===\n");
  printf("  Baseline:       %2d fac, %3d solves, gap=%.2e\n",
         r1.total_factorizations, r1.total_solves, r1.complementarity);
  printf("  Frozen-J (1):   %2d fac, %3d solves, gap=%.2e\n",
         r2.total_factorizations, r2.total_solves, r2.complementarity);
  if (r1.total_factorizations > 0) {
    printf("  Factorization reduction: %.0f%%\n",
           100.0 * (1.0 - static_cast<double>(r2.total_factorizations) /
                          r1.total_factorizations));
  }

  EXPECT_LT(r1.complementarity, 1e-8);
  EXPECT_LT(r2.complementarity, 1e-8);
  EXPECT_LE(r2.total_factorizations, r1.total_factorizations);
}

// Debug: verify refactor_inner mode produces same trajectory as baseline.
TEST(GeodesicBarrierQP, FrozenJacobian_ThetaCont_SpinFactor) {
  auto [soc_s, sdp_s] = BuildSpinFactorPair(2, 2, 77);

  // Baseline: max_centering_steps=0.
  CompiledModel cm_base(*soc_s.kkt(), soc_s.MakeCostRHS());
  RowSpace W_base = cm_base.MakeRowSpace(); setOnes(W_base);
  auto r_base = SolveGeodesicThetaContinuation(cm_base, W_base, 20, 0, 1e-10);

  // refactor_inner=true, max_centering_steps=1: each "outer" does 2 steps.
  // Should visit the same W points as baseline, just in pairs.
  CompiledModel cm_ref(*soc_s.kkt(), soc_s.MakeCostRHS());
  RowSpace W_ref = cm_ref.MakeRowSpace(); setOnes(W_ref);
  auto r_ref = SolveGeodesicThetaContinuation(cm_ref, W_ref, 20, 1, 1e-10);

  printf("\n=== Refactor-inner identity check ===\n");
  printf("  Baseline:       %2d iters, %2d fac, gap=%.2e\n",
         r_base.iterations, r_base.total_factorizations,
         r_base.complementarity);
  printf("  Refactor-inner: %2d iters, %2d fac, gap=%.2e\n",
         r_ref.iterations, r_ref.total_factorizations,
         r_ref.complementarity);

  // The refactor-inner version does 2 steps per outer iteration.
  // Its iter_stats should match baseline's iter_stats at every OTHER index.
  // refactor_inner iter 0 = baseline iter 0 (outer step)
  // refactor_inner iter 0 inner = baseline iter 1 (inner step after refactor)
  // But iter_stats only records outer steps...
  // So let's just compare mu sequences: they should interleave.
  // refactor_inner iter i corresponds to baseline iter 2*i (every other step).
  printf("  ref_i  base_i  ref_mu         base_mu        err\n");
  for (int i = 0; i < r_ref.iterations; ++i) {
    int base_i = 2 * i;
    if (base_i >= r_base.iterations) break;
    double r_mu = r_ref.iter_stats[i].mu;
    double b_mu = r_base.iter_stats[base_i].mu;
    double err = std::abs(b_mu - r_mu) / (std::abs(b_mu) + 1e-30);
    printf("  %3d    %3d    %.6e  %.6e  %.2e %s\n",
           i, base_i, r_mu, b_mu, err, err < 1e-10 ? "OK" : "MISMATCH");
    EXPECT_LT(err, 1e-10) << "mu mismatch at ref iter " << i
                           << " (baseline iter " << base_i << ")";
  }
}

static void PrintSolveResult(const char* name, const SolveResult& r) {
  printf("  %-25s  %3d  %5d  %12.6e  %10.2e  %10.2e  %10.2e  %10.2e  %10.2e\n",
         name, r.factorizations, r.iterations,
         r.objective,
         r.optimality.dual_residual,
         r.optimality.complementarity,
         r.optimality.min_slack,
         r.optimality.min_dual,
         r.duals.stationarity_gradient.norm());
}

// Compare ThetaCont (baseline), ThetaCont (frozen-J), ThetaContR, and GeodesicLP.
TEST(GeodesicBarrierQP, CompareAlgorithms_LP) {
  srand(99);
  const int n = 6, m = 14;
  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(toSparse(A), b, vars);
  model.SetLinearCost(c);

  auto r1 = Solver::Build(model).Solve(ThetaContinuation{1e-10, 50, 0});
  auto r2 = Solver::Build(model).Solve(ThetaContinuation{1e-10, 50, 1});
  auto r3 = Solver::Build(model).Solve(ThetaContinuationR{1e-10, 500});
  auto r4 = Solver::Build(model).Solve(GeodesicLP{1e-10, 30});
  auto r5 = Solver::Build(model).Solve(GeodesicLP{1e-10, 30, 1});
  auto r6 = Solver::Build(model).Solve(HybridR{1e-10, 500});
  auto r7 = Solver::Build(model).Solve(PhaseOneHybrid{1e-10, 500});
  auto r8 = Solver::Build(model).Solve(HybridOnly{1e-10, 500});

  printf("\n=== LP algorithm comparison ===\n");
  printf("  %-25s  %3s  %5s  %12s  %10s  %10s  %10s  %10s  %10s\n",
         "Algorithm", "fac", "iter", "objective",
         "dual_res", "compl", "min_s", "min_lam", "stat_grad");
  PrintSolveResult("ThetaCont", r1);
  PrintSolveResult("ThetaCont + frozen-J", r2);
  PrintSolveResult("ThetaContR", r3);
  PrintSolveResult("GeodesicLP", r4);
  PrintSolveResult("GeodesicLP + frozen-J", r5);
  PrintSolveResult("HybridR", r6);
  PrintSolveResult("PhaseOneHybrid", r7);
  PrintSolveResult("HybridOnly", r8);
}

TEST(GeodesicBarrierQP, CompareAlgorithms_SDP) {
  srand(42);
  const int n_psd = 4, p = 6;
  std::vector<Eigen::SparseMatrix<double>> A_list;
  auto make_sym = [](int n) {
    MatrixXd M = MatrixXd::Random(n, n);
    return (M + M.transpose()) / 2.0;
  };
  for (int i = 0; i < p; ++i)
    A_list.push_back(toSparse(make_sym(n_psd)));
  Eigen::SparseMatrix<double> B = toSparse(
      3.0 * MatrixXd::Identity(n_psd, n_psd));
  VectorXd c(p);
  for (int i = 0; i < p; ++i)
    c(i) = Eigen::MatrixXd(A_list[i]).trace();
  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddPSDConstraint(A_list, B, vars, false);
  model.SetLinearCost(c);

  auto r1 = Solver::Build(model).Solve(ThetaContinuation{1e-10, 50, 0});
  auto r2 = Solver::Build(model).Solve(ThetaContinuation{1e-10, 50, 1});
  auto r3 = Solver::Build(model).Solve(ThetaContinuationR{1e-10, 500});
  auto r4 = Solver::Build(model).Solve(GeodesicLP{1e-10, 30});
  auto r5 = Solver::Build(model).Solve(GeodesicLP{1e-10, 30, 1});
  auto r6 = Solver::Build(model).Solve(HybridR{1e-10, 500});
  auto r7 = Solver::Build(model).Solve(PhaseOneHybrid{1e-10, 500});
  auto r8 = Solver::Build(model).Solve(HybridOnly{1e-10, 500});

  printf("\n=== SDP algorithm comparison ===\n");
  printf("  %-25s  %3s  %5s  %12s  %10s  %10s  %10s  %10s  %10s\n",
         "Algorithm", "fac", "iter", "objective",
         "dual_res", "compl", "min_s", "min_lam", "stat_grad");
  PrintSolveResult("ThetaCont", r1);
  PrintSolveResult("ThetaCont + frozen-J", r2);
  PrintSolveResult("ThetaContR", r3);
  PrintSolveResult("GeodesicLP", r4);
  PrintSolveResult("GeodesicLP + frozen-J", r5);
  PrintSolveResult("HybridR", r6);
  PrintSolveResult("PhaseOneHybrid", r7);
  PrintSolveResult("HybridOnly", r8);
}

// Sweep over problem sizes and compare all algorithms.
TEST(GeodesicBarrierQP, AlgorithmSweep_LP) {
  struct Config { int n; int m; };
  Config configs[] = {{4, 8}, {6, 14}, {10, 25}, {15, 40}, {20, 60}, {30, 80}};

  printf("\n=== LP sweep: factorizations (fac) and stationarity (stat) ===\n");
  printf("  %4s %4s  %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s\n",
         "n", "m", "ThetaCont", "TC+frozenJ", "ThetaContR",
         "GeodesicLP", "LP+frozenJ", "HybridR", "P1Hybrid", "HybridOnly");
  printf("  %4s %4s  %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s\n",
         "", "", "fac/stat", "fac/stat", "fac/stat",
         "fac/stat", "fac/stat", "fac/stat", "fac/stat", "fac/stat");
  printf("  %s\n", std::string(112, '-').c_str());

  for (auto& cfg : configs) {
    srand(42 + cfg.n);
    MatrixXd A = MatrixXd::Random(cfg.m, cfg.n).cwiseAbs()
                 + 0.1 * MatrixXd::Ones(cfg.m, cfg.n);
    VectorXd b = VectorXd::Ones(cfg.m);
    VectorXd c = A.transpose() * VectorXd::Ones(cfg.m);
    std::vector<int> vars(cfg.n);
    std::iota(vars.begin(), vars.end(), 0);

    Model model;
    model.AddLinearConstraint(toSparse(A), b, vars);
    model.SetLinearCost(c);

    auto r1 = Solver::Build(model).Solve(ThetaContinuation{1e-10, 50, 0});
    auto r2 = Solver::Build(model).Solve(ThetaContinuation{1e-10, 50, 1});
    auto r3 = Solver::Build(model).Solve(ThetaContinuationR{1e-10, 500});
    auto r4 = Solver::Build(model).Solve(GeodesicLP{1e-10, 30});
    auto r5 = Solver::Build(model).Solve(GeodesicLP{1e-10, 30, 1});
    auto r6 = Solver::Build(model).Solve(HybridR{1e-10, 500});
    auto r7 = Solver::Build(model).Solve(PhaseOneHybrid{1e-10, 500});
    auto r8 = Solver::Build(model).Solve(HybridOnly{1e-10, 500});

    auto fmt = [](const SolveResult& r) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%3d/%.0e", r.factorizations,
               r.duals.stationarity_gradient.norm());
      return std::string(buf);
    };

    printf("  %4d %4d  %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s\n",
           cfg.n, cfg.m,
           fmt(r1).c_str(), fmt(r2).c_str(), fmt(r3).c_str(),
           fmt(r4).c_str(), fmt(r5).c_str(), fmt(r6).c_str(),
           fmt(r7).c_str(), fmt(r8).c_str());
  }
}

TEST(GeodesicBarrierQP, AlgorithmSweep_SDP) {
  struct Config { int n_psd; int p; };
  Config configs[] = {{3, 4}, {4, 6}, {5, 8}, {6, 10}, {8, 12}, {10, 15}};

  printf("\n=== SDP sweep: factorizations (fac) and stationarity (stat) ===\n");
  printf("  %5s %4s  %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s\n",
         "n_psd", "p", "ThetaCont", "TC+frozenJ", "ThetaContR",
         "GeodesicLP", "LP+frozenJ", "HybridR", "P1Hybrid", "HybridOnly");
  printf("  %5s %4s  %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s\n",
         "", "", "fac/stat", "fac/stat", "fac/stat",
         "fac/stat", "fac/stat", "fac/stat", "fac/stat", "fac/stat");
  printf("  %s\n", std::string(114, '-').c_str());

  auto make_sym = [](int n) {
    MatrixXd M = MatrixXd::Random(n, n);
    return (M + M.transpose()) / 2.0;
  };

  for (auto& cfg : configs) {
    srand(42 + cfg.n_psd);
    std::vector<Eigen::SparseMatrix<double>> A_list;
    for (int i = 0; i < cfg.p; ++i)
      A_list.push_back(toSparse(make_sym(cfg.n_psd)));
    Eigen::SparseMatrix<double> B = toSparse(
        3.0 * MatrixXd::Identity(cfg.n_psd, cfg.n_psd));
    VectorXd c(cfg.p);
    for (int i = 0; i < cfg.p; ++i)
      c(i) = Eigen::MatrixXd(A_list[i]).trace();
    std::vector<int> vars(cfg.p);
    std::iota(vars.begin(), vars.end(), 0);

    Model model;
    model.AddPSDConstraint(A_list, B, vars, false);
    model.SetLinearCost(c);

    auto r1 = Solver::Build(model).Solve(ThetaContinuation{1e-10, 50, 0});
    auto r2 = Solver::Build(model).Solve(ThetaContinuation{1e-10, 50, 1});
    auto r3 = Solver::Build(model).Solve(ThetaContinuationR{1e-10, 500});
    auto r4 = Solver::Build(model).Solve(GeodesicLP{1e-10, 30});
    auto r5 = Solver::Build(model).Solve(GeodesicLP{1e-10, 30, 1});
    auto r6 = Solver::Build(model).Solve(HybridR{1e-10, 500});
    auto r7 = Solver::Build(model).Solve(PhaseOneHybrid{1e-10, 500});
    auto r8 = Solver::Build(model).Solve(HybridOnly{1e-10, 500});

    auto fmt = [](const SolveResult& r) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%3d/%.0e", r.factorizations,
               r.duals.stationarity_gradient.norm());
      return std::string(buf);
    };

    printf("  %5d %4d  %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s\n",
           cfg.n_psd, cfg.p,
           fmt(r1).c_str(), fmt(r2).c_str(), fmt(r3).c_str(),
           fmt(r4).c_str(), fmt(r5).c_str(), fmt(r6).c_str(),
           fmt(r7).c_str(), fmt(r8).c_str());
  }
}

}  // namespace
}  // namespace conex
