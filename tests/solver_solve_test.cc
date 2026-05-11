#include <gtest/gtest.h>

#include <cmath>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/conex.h"
#include "conex/common/extended_embedding.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/tree_spec.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

TEST(SolverSolve, LPDualFeasibility) {
  // min c'x  s.t. Ax + b >= 0
  // KKT: c = A' lambda, lambda >= 0, s >= 0, lambda . s = 0.
  srand(42);
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

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(conex::PhaseOneHybrid());

  printf("  mu = %.2e, converged = %d\n", result.mu, result.converged);
  ASSERT_EQ(result.duals.lambda.size(), 1u);
  ASSERT_EQ(result.duals.slack.size(), 1u);

  const auto& lam = result.duals.lambda[0];
  const auto& s = result.duals.slack[0];
  ASSERT_EQ(lam.size(), m);
  ASSERT_EQ(s.size(), m);

  // Primal feasibility: s >= 0.
  printf("  min_slack = %.2e\n", s.minCoeff());
  EXPECT_GE(s.minCoeff(), -1e-5);

  // Dual non-negativity: lambda >= 0.
  printf("  min_lambda = %.2e\n", lam.minCoeff());
  EXPECT_GE(lam.minCoeff(), -1e-5);

  // Complementarity: lambda . s ≈ 0.
  double cs = lam.dot(s);
  printf("  complementarity = %.2e\n", cs);
  EXPECT_LT(cs, 1e-5);

  // Dual feasibility: A' lambda ≈ c.
  VectorXd dual_res = A_dense.transpose() * lam - c;
  printf("  dual_residual = %.2e\n", dual_res.norm());
  EXPECT_LT(dual_res.norm(), 1e-5);
}

TEST(SolverSolve, QPDualFeasibility) {
  // min (1/2)x'Qx + c'x  s.t. Ax + b >= 0
  // KKT: Qx + c = A' lambda.
  srand(42);
  const int n = 5, m = 8;
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());
  VectorXd b = VectorXd::Ones(m);

  // Diagonal Q for positive definiteness.
  Eigen::SparseMatrix<double> Q(n, n);
  std::vector<Eigen::Triplet<double>> qt;
  for (int i = 0; i < n; ++i) qt.emplace_back(i, i, 1.0);
  Q.setFromTriplets(qt.begin(), qt.end());

  VectorXd c_cost = VectorXd::Random(n);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.AddQuadraticCost(Q, vars);
  model.SetLinearCost(c_cost);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(conex::ThetaContinuation());

  printf("  mu = %.2e\n", result.mu);
  ASSERT_EQ(result.duals.lambda.size(), 1u);
  ASSERT_EQ(result.duals.slack.size(), 1u);

  const auto& lam = result.duals.lambda[0];
  const auto& s = result.duals.slack[0];

  // Primal feasibility.
  printf("  min_slack = %.2e\n", s.minCoeff());
  EXPECT_GE(s.minCoeff(), -1e-5);

  // Dual non-negativity.
  printf("  min_lambda = %.2e\n", lam.minCoeff());
  EXPECT_GE(lam.minCoeff(), -1e-5);

  // Complementarity.
  double cs = lam.dot(s);
  printf("  complementarity = %.2e\n", cs);
  EXPECT_LT(cs, 1e-5);

  // Stationarity: Qx + c = A' lambda.
  VectorXd grad = Q * result.x + c_cost;
  VectorXd dual_res = grad - A_dense.transpose() * lam;
  printf("  stationarity_residual = %.2e\n", dual_res.norm());
  EXPECT_LT(dual_res.norm(), 1e-5);
}

TEST(SolverSolve, MultipleConstraints) {
  // min c'x  s.t.  A1*x + b1 >= 0,  A2*x + b2 >= 0
  //
  // Verify per-constraint duals are correctly indexed and sized
  // when the Model has multiple inequality constraints.
  srand(77);
  const int n = 6;

  // Two inequality constraints with different row counts.
  const int m1 = 4, m2 = 3;
  MatrixXd A1d = MatrixXd::Random(m1, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m1, n);
  MatrixXd A2d = MatrixXd::Random(m2, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m2, n);
  Eigen::SparseMatrix<double> A1 = A1d.sparseView();
  Eigen::SparseMatrix<double> A2 = A2d.sparseView();
  VectorXd b1 = VectorXd::Ones(m1);
  VectorXd b2 = VectorXd::Ones(m2);

  // Cost: A1^T 1 + A2^T 1 (central path at W=1 is near optimal).
  VectorXd c = A1d.transpose() * VectorXd::Ones(m1) +
               A2d.transpose() * VectorXd::Ones(m2);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(A1, b1, vars);
  model.AddLinearConstraint(A2, b2, vars);
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(conex::PhaseOneHybrid());

  printf("  mu = %.2e\n", result.mu);

  // --- Structural checks ---
  ASSERT_EQ(result.duals.lambda.size(), 2u);
  ASSERT_EQ(result.duals.slack.size(), 2u);

  // Sizes match constraint row counts.
  EXPECT_EQ(result.duals.lambda[0].size(), m1);
  EXPECT_EQ(result.duals.lambda[1].size(), m2);
  EXPECT_EQ(result.duals.slack[0].size(), m1);
  EXPECT_EQ(result.duals.slack[1].size(), m2);

  // --- Primal feasibility: per-constraint slacks >= 0 ---
  printf("  slack[0] min = %.2e, slack[1] min = %.2e\n",
         result.duals.slack[0].minCoeff(), result.duals.slack[1].minCoeff());
  EXPECT_GE(result.duals.slack[0].minCoeff(), -1e-5);
  EXPECT_GE(result.duals.slack[1].minCoeff(), -1e-5);

  // Verify slacks match A*x + b directly.
  VectorXd s1_check = A1d * result.x + b1;
  VectorXd s2_check = A2d * result.x + b2;
  EXPECT_LT((result.duals.slack[0] - s1_check).norm(), 1e-6);
  EXPECT_LT((result.duals.slack[1] - s2_check).norm(), 1e-6);

  // --- Dual non-negativity ---
  EXPECT_GE(result.duals.lambda[0].minCoeff(), -1e-5);
  EXPECT_GE(result.duals.lambda[1].minCoeff(), -1e-5);

  // --- Per-constraint complementarity ---
  double cs0 = result.duals.lambda[0].dot(result.duals.slack[0]);
  double cs1 = result.duals.lambda[1].dot(result.duals.slack[1]);
  printf("  complementarity: %.2e, %.2e\n", cs0, cs1);
  EXPECT_LT(cs0, 1e-5);
  EXPECT_LT(cs1, 1e-5);

  // --- Stationarity: c = A1' lambda[0] + A2' lambda[1] ---
  VectorXd dual_res = c - A1d.transpose() * result.duals.lambda[0]
                         - A2d.transpose() * result.duals.lambda[1];
  printf("  stationarity_residual = %.2e\n", dual_res.norm());
  EXPECT_LT(dual_res.norm(), 1e-5);
}

TEST(SolverSolve, EqualityConstraints) {
  // min c'x  s.t.  Ax + b >= 0,  C1*x = d1,  C2*x = d2
  //
  // Uses ThetaContinuation which handles equality RHS via
  // ComputeFullDecomposition + EqualityAffineTermRHS.
  srand(99);
  const int n = 6, m = 10;

  MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  Eigen::SparseMatrix<double> A = Ad.sparseView();
  VectorXd b = VectorXd::Ones(m);

  // Two equality constraints: x0 + x1 = 0.5,  x2 + x3 = 0.5.
  Eigen::SparseMatrix<double> C1(1, n), C2(1, n);
  {
    std::vector<Eigen::Triplet<double>> t;
    t.emplace_back(0, 0, 1.0); t.emplace_back(0, 1, 1.0);
    C1.setFromTriplets(t.begin(), t.end());
  }
  {
    std::vector<Eigen::Triplet<double>> t;
    t.emplace_back(0, 2, 1.0); t.emplace_back(0, 3, 1.0);
    C2.setFromTriplets(t.begin(), t.end());
  }
  VectorXd d1(1), d2(1);
  d1 << 0.5;
  d2 << 0.5;

  VectorXd c = Ad.transpose() * VectorXd::Ones(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.AddEqualityConstraint(C1, d1, vars);
  model.AddEqualityConstraint(C2, d2, vars);
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(conex::ThetaContinuation());

  printf("  mu = %.2e\n", result.mu);

  // --- Structural checks ---
  ASSERT_EQ(result.duals.lambda.size(), 1u);  // one inequality constraint
  ASSERT_EQ(result.duals.slack.size(), 1u);
  ASSERT_EQ(result.duals.nu.size(), 2u);       // two equality constraints
  EXPECT_EQ(result.duals.nu[0].size(), 1);
  EXPECT_EQ(result.duals.nu[1].size(), 1);

  // --- Primal feasibility ---
  printf("  min_slack = %.2e\n", result.duals.slack[0].minCoeff());
  EXPECT_GE(result.duals.slack[0].minCoeff(), -1e-5);

  // Equality: x0+x1 ≈ 0.5, x2+x3 ≈ 0.5.
  double eq1_err = std::abs(result.x(0) + result.x(1) - 0.5);
  double eq2_err = std::abs(result.x(2) + result.x(3) - 0.5);
  printf("  equality errors: %.2e, %.2e\n", eq1_err, eq2_err);
  EXPECT_LT(eq1_err, 1e-5);
  EXPECT_LT(eq2_err, 1e-5);

  // --- Complementarity ---
  double cs = result.duals.lambda[0].dot(result.duals.slack[0]);
  printf("  complementarity = %.2e\n", cs);
  EXPECT_LT(cs, 1e-5);

  // --- Stationarity: c = A' lambda + C1' nu1 + C2' nu2 ---
  VectorXd At_lam = Ad.transpose() * result.duals.lambda[0];
  VectorXd Ct_nu = Eigen::MatrixXd(C1).transpose() * result.duals.nu[0] +
                   Eigen::MatrixXd(C2).transpose() * result.duals.nu[1];
  VectorXd stat_res = c - At_lam - Ct_nu;
  printf("  stationarity_residual = %.2e\n", stat_res.norm());
  EXPECT_LT(stat_res.norm(), 1e-5);
}

TEST(SolverSolve, PSDConstraint) {
  // min c'x  s.t.  Σ A_k x_k + B ≽ 0
  // KKT: c_k = tr(A_k · Λ),  S ≽ 0,  Λ ≽ 0,  tr(S·Λ) = 0.
  srand(42);
  const int n_mat = 3;  // 3×3 PSD matrices
  const int p = 4;      // 4 optimization variables

  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars(p);
  VectorXd c(p);
  for (int k = 0; k < p; ++k) {
    MatrixXd Ak = MatrixXd::Random(n_mat, n_mat);
    Ak = 0.5 * (Ak + Ak.transpose());
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < n_mat; ++i)
      for (int j = 0; j < n_mat; ++j)
        if (std::abs(Ak(i, j)) > 1e-14)
          trips.emplace_back(i, j, Ak(i, j));
    Eigen::SparseMatrix<double> As(n_mat, n_mat);
    As.setFromTriplets(trips.begin(), trips.end());
    A_list.push_back(As);
    vars[k] = k;
    // c_k = tr(A_k) so W=I at k=1 is centered.
    c(k) = Ak.trace();
  }
  Eigen::SparseMatrix<double> B =
      Eigen::MatrixXd::Identity(n_mat, n_mat).sparseView();

  Model model;
  model.AddPSDConstraint(A_list, B, vars, /*use_chordal=*/false);
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(conex::ThetaContinuation());

  printf("  mu = %.2e\n", result.mu);

  // --- Structural checks ---
  ASSERT_EQ(result.duals.psd_lambda.size(), 1u);
  ASSERT_EQ(result.duals.psd_slack.size(), 1u);
  EXPECT_EQ(result.duals.psd_lambda[0].rows(), n_mat);
  EXPECT_EQ(result.duals.psd_lambda[0].cols(), n_mat);
  EXPECT_EQ(result.duals.psd_slack[0].rows(), n_mat);
  EXPECT_EQ(result.duals.psd_slack[0].cols(), n_mat);

  const auto& Lambda = result.duals.psd_lambda[0];
  const auto& S = result.duals.psd_slack[0];

  // --- Primal feasibility: S ≽ 0 (check min eigenvalue) ---
  Eigen::SelfAdjointEigenSolver<MatrixXd> eig_s(S);
  double min_eig_s = eig_s.eigenvalues().minCoeff();
  printf("  min_eig(S) = %.2e\n", min_eig_s);
  EXPECT_GE(min_eig_s, -1e-5);

  // --- Dual feasibility: Λ ≽ 0 ---
  Eigen::SelfAdjointEigenSolver<MatrixXd> eig_l(Lambda);
  double min_eig_l = eig_l.eigenvalues().minCoeff();
  printf("  min_eig(Lambda) = %.2e\n", min_eig_l);
  EXPECT_GE(min_eig_l, -1e-5);

  // --- Complementarity: tr(S · Λ) ≈ 0 ---
  double cs = (S * Lambda).trace();
  printf("  tr(S*Lambda) = %.2e\n", cs);
  EXPECT_LT(std::abs(cs), 1e-5);

  // --- Stationarity: c_k = tr(A_k · Λ) ---
  VectorXd stat_res(p);
  for (int k = 0; k < p; ++k) {
    // tr(A_k · Λ) using sparse A_k, no dense promotion.
    double trAL = 0;
    const auto& Ak = A_list[k];
    for (int col = 0; col < Ak.outerSize(); ++col)
      for (Eigen::SparseMatrix<double>::InnerIterator it(Ak, col); it; ++it)
        trAL += it.value() * Lambda(it.row(), it.col());
    stat_res(k) = c(k) - trAL;
  }
  printf("  stationarity_residual = %.2e\n", stat_res.norm());
  EXPECT_LT(stat_res.norm(), 1e-5);
}

TEST(SolverSolve, SOCConstraint) {
  // min c'x  s.t.  ||A₁x + b₁|| ≤ A₀x + b₀
  // SOC dual feasibility: λ₀ >= ||λ₁||.
  srand(42);
  const int vec_dim = 3;
  const int n_soc = 1 + vec_dim;
  const int p = 3;

  MatrixXd A_dense = MatrixXd::Random(n_soc, p);
  // Ensure A₀ row is positive so the SOC has interior.
  A_dense.row(0) = A_dense.row(0).cwiseAbs() + 0.5 * VectorXd::Ones(p).transpose();

  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < n_soc; ++i)
    for (int j = 0; j < p; ++j)
      if (std::abs(A_dense(i, j)) > 1e-14)
        trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(n_soc, p);
  A.setFromTriplets(trips.begin(), trips.end());

  VectorXd b = VectorXd::Zero(n_soc);
  b(0) = 1.0;

  // Cost: c = A₀ so W=I at k=1 is centered.
  VectorXd c = A_dense.row(0).transpose();

  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddSOCConstraint(A, b, vars);
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(conex::ThetaContinuation());

  printf("  mu = %.2e\n", result.mu);

  ASSERT_EQ(result.duals.lambda.size(), 1u);
  ASSERT_EQ(result.duals.slack.size(), 1u);

  const auto& lam = result.duals.lambda[0];
  const auto& s = result.duals.slack[0];
  ASSERT_EQ(lam.size(), n_soc);
  ASSERT_EQ(s.size(), n_soc);

  // SOC membership: s₀ >= ||s₁||.
  double s0 = s(0);
  double s1_norm = s.tail(vec_dim).norm();
  printf("  s0 = %.4e, ||s1|| = %.4e\n", s0, s1_norm);
  EXPECT_GE(s0 + 1e-5, s1_norm);

  // Dual SOC membership: λ₀ >= ||λ₁||.
  double l0 = lam(0);
  double l1_norm = lam.tail(vec_dim).norm();
  printf("  lam0 = %.4e, ||lam1|| = %.4e\n", l0, l1_norm);
  EXPECT_GE(l0 + 1e-5, l1_norm);

  // Complementarity: λ · s ≈ 0.
  double cs = lam.dot(s);
  printf("  complementarity = %.2e\n", cs);
  EXPECT_LT(std::abs(cs), 1e-5);

  // Stationarity: c = A' λ.
  VectorXd stat_res = c - A_dense.transpose() * lam;
  printf("  stationarity_residual = %.2e\n", stat_res.norm());
  EXPECT_LT(stat_res.norm(), 1e-5);
}

TEST(SolverSolve, AllConstraintTypes) {
  // Two of each constraint type with different sizes to catch indexing bugs.
  // Constraints are intentionally interleaved in Model registration order
  // (LP1, SOC1, PSD1, LP2, SOC2, PSD2, Q, EQ1, EQ2) to stress the
  // RowSpace offset computation which reorders by type.
  srand(88);
  const int n = 10;

  // --- LP1: 3 rows on vars 0..3 ---
  const int m_lp1 = 3;
  MatrixXd A_lp1_d = MatrixXd::Random(m_lp1, 4).cwiseAbs() + 0.1 * MatrixXd::Ones(m_lp1, 4);
  Eigen::SparseMatrix<double> A_lp1 = A_lp1_d.sparseView();
  VectorXd b_lp1 = VectorXd::Ones(m_lp1);
  std::vector<int> lp1_vars = {0, 1, 2, 3};

  // --- SOC1: dim 3 on vars 0..2 ---
  const int soc1_dim = 3;
  MatrixXd A_soc1_d = MatrixXd::Random(soc1_dim, 3);
  A_soc1_d.row(0) = A_soc1_d.row(0).cwiseAbs() + 0.5 * VectorXd::Ones(3).transpose();
  Eigen::SparseMatrix<double> A_soc1 = A_soc1_d.sparseView();
  VectorXd b_soc1 = VectorXd::Zero(soc1_dim); b_soc1(0) = 1.0;
  std::vector<int> soc1_vars = {0, 1, 2};

  // --- PSD1: 2×2, 2 variables on vars 4..5 ---
  const int n_psd1 = 2, p_psd1 = 2;
  std::vector<Eigen::SparseMatrix<double>> psd1_A;
  {
    MatrixXd A0 = MatrixXd::Zero(n_psd1, n_psd1); A0(0, 0) = 1.0;
    MatrixXd A1 = MatrixXd::Zero(n_psd1, n_psd1); A1(1, 1) = 1.0;
    psd1_A.push_back(A0.sparseView());
    psd1_A.push_back(A1.sparseView());
  }
  auto psd1_B = Eigen::MatrixXd::Identity(n_psd1, n_psd1).sparseView();
  std::vector<int> psd1_vars = {4, 5};

  // --- LP2: 2 rows on vars 2..4 (overlaps with LP1 and PSD1) ---
  const int m_lp2 = 2;
  MatrixXd A_lp2_d = MatrixXd::Random(m_lp2, 3).cwiseAbs() + 0.1 * MatrixXd::Ones(m_lp2, 3);
  Eigen::SparseMatrix<double> A_lp2 = A_lp2_d.sparseView();
  VectorXd b_lp2 = VectorXd::Ones(m_lp2);
  std::vector<int> lp2_vars = {2, 3, 4};

  // --- SOC2: dim 4 on vars 5..7 (different size from SOC1) ---
  const int soc2_dim = 4;
  MatrixXd A_soc2_d = MatrixXd::Random(soc2_dim, 3);
  A_soc2_d.row(0) = A_soc2_d.row(0).cwiseAbs() + 0.5 * VectorXd::Ones(3).transpose();
  Eigen::SparseMatrix<double> A_soc2 = A_soc2_d.sparseView();
  VectorXd b_soc2 = VectorXd::Zero(soc2_dim); b_soc2(0) = 1.0;
  std::vector<int> soc2_vars = {5, 6, 7};

  // --- PSD2: 3×3, 3 variables on vars 7..9 (different size from PSD1) ---
  const int n_psd2 = 3, p_psd2 = 3;
  std::vector<Eigen::SparseMatrix<double>> psd2_A;
  {
    MatrixXd A0 = MatrixXd::Zero(n_psd2, n_psd2); A0(0, 0) = 1.0;
    MatrixXd A1 = MatrixXd::Zero(n_psd2, n_psd2); A1(1, 1) = 1.0;
    MatrixXd A2 = MatrixXd::Zero(n_psd2, n_psd2); A2(2, 2) = 1.0;
    psd2_A.push_back(A0.sparseView());
    psd2_A.push_back(A1.sparseView());
    psd2_A.push_back(A2.sparseView());
  }
  auto psd2_B = Eigen::MatrixXd::Identity(n_psd2, n_psd2).sparseView();
  std::vector<int> psd2_vars = {7, 8, 9};

  // --- EQ1: x0 + x1 = 0.5 ---
  Eigen::SparseMatrix<double> C1(1, n);
  { std::vector<Eigen::Triplet<double>> t;
    t.emplace_back(0, 0, 1.0); t.emplace_back(0, 1, 1.0);
    C1.setFromTriplets(t.begin(), t.end()); }
  VectorXd d1(1); d1 << 0.5;

  // --- EQ2: x8 + x9 = 0.3 (touches PSD2 vars) ---
  Eigen::SparseMatrix<double> C2(1, n);
  { std::vector<Eigen::Triplet<double>> t;
    t.emplace_back(0, 8, 1.0); t.emplace_back(0, 9, 1.0);
    C2.setFromTriplets(t.begin(), t.end()); }
  VectorXd d2(1); d2 << 0.3;

  std::vector<int> eq_vars(n);
  std::iota(eq_vars.begin(), eq_vars.end(), 0);

  // --- Quadratic cost ---
  Eigen::SparseMatrix<double> Q(n, n);
  { std::vector<Eigen::Triplet<double>> qt;
    for (int i = 0; i < n; ++i) qt.emplace_back(i, i, 0.1);
    Q.setFromTriplets(qt.begin(), qt.end()); }
  std::vector<int> all_vars(n);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // Build cost: sum of central-path costs for each constraint.
  VectorXd c = VectorXd::Zero(n);
  for (int j = 0; j < (int)lp1_vars.size(); ++j)
    for (int i = 0; i < m_lp1; ++i) c(lp1_vars[j]) += A_lp1_d(i, j);
  for (int j = 0; j < (int)lp2_vars.size(); ++j)
    for (int i = 0; i < m_lp2; ++i) c(lp2_vars[j]) += A_lp2_d(i, j);
  for (int j = 0; j < (int)soc1_vars.size(); ++j)
    c(soc1_vars[j]) += A_soc1_d(0, j);
  for (int j = 0; j < (int)soc2_vars.size(); ++j)
    c(soc2_vars[j]) += A_soc2_d(0, j);
  for (int k = 0; k < p_psd1; ++k)
    c(psd1_vars[k]) += Eigen::MatrixXd(psd1_A[k]).trace();
  for (int k = 0; k < p_psd2; ++k)
    c(psd2_vars[k]) += Eigen::MatrixXd(psd2_A[k]).trace();

  // Register in interleaved order: LP1, SOC1, PSD1, LP2, SOC2, PSD2, Q, EQ1, EQ2.
  Model model;
  model.AddLinearConstraint(A_lp1, b_lp1, lp1_vars);
  model.AddSOCConstraint(A_soc1, b_soc1, soc1_vars);
  model.AddPSDConstraint(psd1_A, psd1_B, psd1_vars, false);
  model.AddLinearConstraint(A_lp2, b_lp2, lp2_vars);
  model.AddSOCConstraint(A_soc2, b_soc2, soc2_vars);
  model.AddPSDConstraint(psd2_A, psd2_B, psd2_vars, false);
  model.AddQuadraticCost(Q, all_vars);
  model.AddEqualityConstraint(C1, d1, eq_vars);
  model.AddEqualityConstraint(C2, d2, eq_vars);
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(conex::ThetaContinuation());

  printf("  mu = %.2e\n", result.mu);

  // --- Structural checks: 2 of each ---
  // lambda/slack: LP1, SOC1, LP2, SOC2 (Model order).
  ASSERT_EQ(result.duals.lambda.size(), 4u);
  ASSERT_EQ(result.duals.slack.size(), 4u);
  EXPECT_EQ(result.duals.lambda[0].size(), m_lp1);   // LP1
  EXPECT_EQ(result.duals.lambda[1].size(), soc1_dim); // SOC1
  EXPECT_EQ(result.duals.lambda[2].size(), m_lp2);    // LP2
  EXPECT_EQ(result.duals.lambda[3].size(), soc2_dim); // SOC2

  ASSERT_EQ(result.duals.psd_lambda.size(), 2u);
  ASSERT_EQ(result.duals.psd_slack.size(), 2u);
  EXPECT_EQ(result.duals.psd_lambda[0].rows(), n_psd1);  // PSD1: 2×2
  EXPECT_EQ(result.duals.psd_lambda[1].rows(), n_psd2);  // PSD2: 3×3

  ASSERT_EQ(result.duals.nu.size(), 2u);
  EXPECT_EQ(result.duals.nu[0].size(), 1);
  EXPECT_EQ(result.duals.nu[1].size(), 1);

  // --- LP primal/dual feasibility ---
  for (int idx = 0; idx < 4; ++idx) {
    bool is_soc = (idx == 1 || idx == 3);
    printf("  cone[%d] (%s): slack_min=%.2e, lambda_min=%.2e\n",
           idx, is_soc ? "SOC" : "LP",
           result.duals.slack[idx].minCoeff(),
           result.duals.lambda[idx].minCoeff());
    if (!is_soc) {
      EXPECT_GE(result.duals.slack[idx].minCoeff(), -1e-5);
      EXPECT_GE(result.duals.lambda[idx].minCoeff(), -1e-5);
    }
  }

  // --- SOC membership ---
  for (int idx : {1, 3}) {
    const auto& s = result.duals.slack[idx];
    const auto& l = result.duals.lambda[idx];
    EXPECT_GE(s(0) + 1e-5, s.tail(s.size() - 1).norm());
    EXPECT_GE(l(0) + 1e-5, l.tail(l.size() - 1).norm());
  }

  // --- PSD feasibility ---
  for (int p = 0; p < 2; ++p) {
    Eigen::SelfAdjointEigenSolver<MatrixXd> eig_s(result.duals.psd_slack[p]);
    Eigen::SelfAdjointEigenSolver<MatrixXd> eig_l(result.duals.psd_lambda[p]);
    printf("  PSD[%d]: min_eig(S)=%.2e, min_eig(L)=%.2e\n", p,
           eig_s.eigenvalues().minCoeff(), eig_l.eigenvalues().minCoeff());
    EXPECT_GE(eig_s.eigenvalues().minCoeff(), -1e-5);
    EXPECT_GE(eig_l.eigenvalues().minCoeff(), -1e-5);
  }

  // --- Equality ---
  double eq1_err = std::abs(result.x(0) + result.x(1) - 0.5);
  double eq2_err = std::abs(result.x(8) + result.x(9) - 0.3);
  printf("  equality errors: %.2e, %.2e\n", eq1_err, eq2_err);
  EXPECT_LT(eq1_err, 1e-5);
  EXPECT_LT(eq2_err, 1e-5);

  // --- Per-constraint complementarity ---
  double total_cs = 0;
  for (int idx = 0; idx < 4; ++idx)
    total_cs += std::abs(result.duals.lambda[idx].dot(result.duals.slack[idx]));
  for (int p = 0; p < 2; ++p)
    total_cs += std::abs((result.duals.psd_slack[p] * result.duals.psd_lambda[p]).trace());
  printf("  total complementarity = %.2e\n", total_cs);
  EXPECT_LT(total_cs, 1e-4);

  // --- Stationarity: c + Qx = Σ A_i' λ_i + Σ tr(A_k · Λ_j) + Σ C_k' ν_k ---
  VectorXd grad = c + Q * result.x;
  VectorXd rhs = VectorXd::Zero(n);

  // LP contributions (indices 0, 2 in lambda).
  struct LPInfo { MatrixXd A; std::vector<int> vars; int lam_idx; };
  LPInfo lps[] = {{A_lp1_d, lp1_vars, 0}, {A_lp2_d, lp2_vars, 2}};
  for (auto& lp : lps) {
    VectorXd At_l = lp.A.transpose() * result.duals.lambda[lp.lam_idx];
    for (int j = 0; j < (int)lp.vars.size(); ++j) rhs(lp.vars[j]) += At_l(j);
  }

  // SOC contributions (indices 1, 3 in lambda).
  struct SOCInfo { MatrixXd A; std::vector<int> vars; int lam_idx; };
  SOCInfo socs[] = {{A_soc1_d, soc1_vars, 1}, {A_soc2_d, soc2_vars, 3}};
  for (auto& soc : socs) {
    VectorXd At_l = soc.A.transpose() * result.duals.lambda[soc.lam_idx];
    for (int j = 0; j < (int)soc.vars.size(); ++j) rhs(soc.vars[j]) += At_l(j);
  }

  // PSD contributions.
  struct PSDInfo { std::vector<Eigen::SparseMatrix<double>>* A; std::vector<int> vars; int psd_idx; };
  PSDInfo psds[] = {{&psd1_A, psd1_vars, 0}, {&psd2_A, psd2_vars, 1}};
  for (auto& psd : psds) {
    const auto& L = result.duals.psd_lambda[psd.psd_idx];
    for (int k = 0; k < (int)psd.A->size(); ++k) {
      double trAL = 0;
      const auto& Ak = (*psd.A)[k];
      for (int col = 0; col < Ak.outerSize(); ++col)
        for (Eigen::SparseMatrix<double>::InnerIterator it(Ak, col); it; ++it)
          trAL += it.value() * L(it.row(), it.col());
      rhs(psd.vars[k]) += trAL;
    }
  }

  // Equality contributions.
  rhs += Eigen::MatrixXd(C1).transpose() * result.duals.nu[0];
  rhs += Eigen::MatrixXd(C2).transpose() * result.duals.nu[1];

  VectorXd stat_res = grad - rhs;
  printf("  stationarity_residual = %.2e\n", stat_res.norm());
  EXPECT_LT(stat_res.norm(), 1e-5);
}

// =====================================================================
// GeodesicLP with equality constraints.
// =====================================================================

TEST(SolverSolve, GeodesicLP_LP) {
  // min c'x  s.t. Ax + b >= 0  (no equalities — baseline).
  srand(42);
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

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(conex::GeodesicLP());

  printf("  GeodesicLP (no eq): mu=%.2e obj=%.6e\n",
         result.mu, result.objective);
  EXPECT_LT(result.mu, 1e-6);
}

TEST(SolverSolve, GeodesicLP_WithEquality) {
  // min c'x  s.t.  Ax + b >= 0,  C1*x = d1,  C2*x = d2.
  // GeodesicLP must inject d_eq to satisfy equalities.
  srand(99);
  const int n = 6, m = 10;

  MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  Eigen::SparseMatrix<double> A = Ad.sparseView();
  VectorXd b = VectorXd::Ones(m);

  // Two equality constraints.
  Eigen::SparseMatrix<double> C1(1, n), C2(1, n);
  {
    std::vector<Eigen::Triplet<double>> t;
    t.emplace_back(0, 0, 1.0); t.emplace_back(0, 1, 1.0);
    C1.setFromTriplets(t.begin(), t.end());
  }
  {
    std::vector<Eigen::Triplet<double>> t;
    t.emplace_back(0, 2, 1.0); t.emplace_back(0, 3, 1.0);
    C2.setFromTriplets(t.begin(), t.end());
  }
  VectorXd d1(1), d2(1);
  d1 << 0.5;
  d2 << 0.5;

  VectorXd c = Ad.transpose() * VectorXd::Ones(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.AddEqualityConstraint(C1, d1, vars);
  model.AddEqualityConstraint(C2, d2, vars);
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(conex::GeodesicLP());

  printf("  GeodesicLP (with eq): mu=%.2e obj=%.6e\n",
         result.mu, result.objective);

  // Equality: x0+x1 ≈ 0.5, x2+x3 ≈ 0.5.
  double eq1_err = std::abs(result.x(0) + result.x(1) - 0.5);
  double eq2_err = std::abs(result.x(2) + result.x(3) - 0.5);
  printf("  equality errors: %.2e, %.2e\n", eq1_err, eq2_err);
  EXPECT_LT(eq1_err, 1e-5);
  EXPECT_LT(eq2_err, 1e-5);
}

TEST(SolverSolve, GeodesicLP_Embedding) {
  // Solve the extended embedding with GeodesicLP.
  // min alpha*theta s.t. embedding equalities, x,s,tau,kappa >= 0.
  const int n = 2, m = 1;
  Eigen::SparseMatrix<double> A(m, n);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 1.0;
  VectorXd b(m); b << 2.0;
  VectorXd c(n); c << 1.0, 2.0;

  auto [emb_model, info, emb_tree] = conex::BuildExtendedEmbedding(A, b, c);

  auto solver = Solver::Build(emb_model);
  auto result = solver.Solve(conex::GeodesicLP{1e-8, 30, 0, false});

  double theta = result.x(info.theta_idx());
  double tau = result.x(info.tau_idx());
  printf("  GeodesicLP (embedding): theta=%.2e tau=%.4f obj=%.6e\n",
         theta, tau, result.objective);

  EXPECT_LT(std::abs(theta), 1e-3);
  if (tau > 1e-6) {
    VectorXd x_opt = result.x.segment(info.x_start(), n) / tau;
    double obj = c.dot(x_opt);
    printf("  x/tau = (%.4f, %.4f)  obj=%.4f (expected 2.0)\n",
           x_opt(0), x_opt(1), obj);
    EXPECT_NEAR(obj, 2.0, 0.1);
  }
}

TEST(SolverSolve, DenseEmbeddingLU) {
  // Test the dense KKT solver with LU on the extended embedding.
  // Compare RLDLT (default) vs LU on m=n and m>n cases.
  srand(42);

  auto test_case = [](int n, int m) {
    MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
    VectorXd b = Ad * VectorXd::Ones(n);
    VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < m; i++)
      for (int j = 0; j < n; j++)
        trips.emplace_back(i, j, Ad(i, j));
    Eigen::SparseMatrix<double> A(m, n);
    A.setFromTriplets(trips.begin(), trips.end());

    auto [emb_model, info, emb_tree] = BuildExtendedEmbedding(A, b, c);
    int N = info.total_vars();

    // Build dense (single-clique) tree for both solvers.
    TreeSpec tree;
    int clique = tree.AddClique();
    for (int i = 0; i < emb_model.num_constraints(); ++i)
      tree.Assign(i, clique);

    // Solve with RLDLT (default).
    auto solver_default = Solver::Build(emb_model, tree);
    auto result_default = solver_default.Solve(HybridOnly{1e-10, 500, false});

    // Solve with LU on the same dense tree.
    SolverConfiguration cfg;
    cfg.tree.use_lu_for_indefinite = true;
    auto solver_lu = Solver::Build(emb_model, tree, cfg);
    auto result_lu = solver_lu.Solve(HybridOnly{1e-10, 500, false});

    // Compare solutions in Model space.
    bool lu_ok = std::isfinite(result_lu.x.norm());
    double diff = lu_ok ? (result_default.x - result_lu.x).norm() : NAN;
    double rel = lu_ok ? diff / std::max(result_default.x.norm(), 1e-15) : NAN;
    printf("  n=%d m=%d N=%d: ||x_default - x_lu|| = %.2e  rel=%.2e  lu_ok=%d\n",
           n, m, N, diff, rel, lu_ok);
  };

  // m <= n: LU works. RLDLT and LU agree to ~1e-6.
  test_case(2, 2);
  test_case(3, 3);
  test_case(5, 5);
  test_case(5, 3);
  test_case(10, 3);
}

// Test that Solver::Solve correctly reports stationarity for problems
// with equality constraints.  The stationarity gradient should be
//   c + Qx - A'λ - C'ν ≈ 0
// where ν are the equality duals.
TEST(SolverSolve, StationarityWithEquality) {
  // min c'x  s.t.  Ax + b >= 0,  Cx = d.
  // Choose (A, b, c) so that W=I, k=1 is on the central path.
  srand(77);
  const int n = 6, m = 10;

  MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  Eigen::SparseMatrix<double> A = Ad.sparseView();
  VectorXd b = VectorXd::Ones(m);
  // c = A'e so that (x=0, lambda=e) is roughly on the central path.
  VectorXd c = Ad.transpose() * VectorXd::Ones(m);

  // Equality: x0 + x1 = 0.5.
  Eigen::SparseMatrix<double> C(1, n);
  {
    std::vector<Eigen::Triplet<double>> t;
    t.emplace_back(0, 0, 1.0);
    t.emplace_back(0, 1, 1.0);
    C.setFromTriplets(t.begin(), t.end());
  }
  VectorXd d(1);
  d << 0.5;

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.AddEqualityConstraint(C, d, vars);
  model.SetLinearCost(c);

  // Test with each algorithm.
  auto test_algo = [&](const char* name, auto algo) {
    auto solver = Solver::Build(model);
    auto result = solver.Solve(algo);

    printf("  %s: mu=%.2e obj=%.6e\n", name, result.mu, result.objective);

    // Equality feasibility: Cx = d.
    double eq_err = std::abs(result.x(0) + result.x(1) - 0.5);
    printf("    eq_err=%.2e\n", eq_err);
    EXPECT_LT(eq_err, 1e-3) << name << ": equality violated";

    // Stationarity: c + Qx - A'λ - C'ν ≈ 0.
    // Only check over Model variables (not internal duals).
    double stat_norm = result.duals.stationarity_gradient.norm();
    printf("    stationarity=%.2e\n", stat_norm);
    EXPECT_LT(stat_norm, 1e-1) << name << ": stationarity residual too large";

    // Check equality duals were extracted.
    ASSERT_EQ(result.duals.nu.size(), 1u) << name;
    printf("    nu[0]=%.4e\n", result.duals.nu[0](0));

    // Check equality residual from ExtractDuals.
    ASSERT_EQ(result.duals.eq_residual.size(), 1u) << name;
    double eq_res = result.duals.eq_residual[0].norm();
    printf("    eq_residual=%.2e\n", eq_res);
    EXPECT_LT(eq_res, 1e-6) << name << ": equality residual too large";
  };

  test_algo("ThetaContinuation", ThetaContinuation{1e-8, 500, 1, false});
  test_algo("HybridOnly", HybridOnly{1e-8, 500, false});
  test_algo("GeodesicLP", GeodesicLP{1e-8, 30, 0, false});
}

}  // namespace
}  // namespace conex
