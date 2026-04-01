#include <gtest/gtest.h>

#include <chrono>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/barrier_qp.h"
#include "conex/algorithms/irls.h"
#include "conex/algorithms/lqr_tree_solver.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

// =====================================================================
// IRLS tests
// =====================================================================

TEST(IRLS, BasicL1) {
  srand(42);
  const int m = 50, n = 5;
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; r++)
    for (int c = 0; c < n; c++)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX - 0.5);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  VectorXd x_true = VectorXd::Random(n);
  VectorXd b = MatrixXd(A) * x_true;
  for (int i = 0; i < m / 10; i++)
    b(rand() % m) += 10.0 * ((double)rand() / RAND_MAX - 0.5);

  auto result = SolveIRLS(A, b, 50, 1e-6, 1e-8);
  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 0.5);
  EXPECT_GT(result.iterations, 1);
  printf("IRLS: %d iters, L1=%.4f, err=%.4f, %.0fus\n",
         result.iterations, result.l1_objective, err, result.solve_time_us);
}

TEST(IRLS, ConvergesToL2WithoutOutliers) {
  srand(42);
  const int m = 30, n = 5;
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; r++)
    for (int c = 0; c < n; c++)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX - 0.5);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  VectorXd x_true = VectorXd::Random(n);
  VectorXd b = MatrixXd(A) * x_true;
  auto result = SolveIRLS(A, b, 50, 1e-8, 1e-10);
  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-6);
}

// =====================================================================
// Barrier QP tests
// =====================================================================

TEST(BarrierQP, UnconstrainedInsideFeasible) {
  const int n = 2, m = 3;
  std::vector<Eigen::Triplet<double>> qt;
  qt.emplace_back(0, 0, 1.0); qt.emplace_back(1, 1, 1.0);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());
  VectorXd c = VectorXd::Zero(n);

  std::vector<Eigen::Triplet<double>> at;
  at.emplace_back(0, 0, 1.0); at.emplace_back(0, 1, 1.0);
  at.emplace_back(1, 0, -1.0);
  at.emplace_back(2, 1, -1.0);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(at.begin(), at.end());
  VectorXd b(m); b << 1.0, 0.0, 0.0;
  VectorXd x0(n); x0 << 0.3, 0.3;

  auto result = SolveBarrierQP(Q, c, A, b, x0);
  EXPECT_NEAR(result.x(0), 0.0, 0.01);
  EXPECT_NEAR(result.x(1), 0.0, 0.01);
  EXPECT_NEAR(result.objective, 0.0, 0.01);
  printf("QP unconstrained: obj=%.6f, x=[%.4f, %.4f], gap=%.2e, "
         "%d outer, %d newton, %.0fus\n",
         result.objective, result.x(0), result.x(1), result.duality_gap,
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

TEST(BarrierQP, ActiveConstraint) {
  const int n = 2, m = 3;
  std::vector<Eigen::Triplet<double>> qt;
  qt.emplace_back(0, 0, 0.001); qt.emplace_back(1, 1, 0.001);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());
  VectorXd c(n); c << 1.0, 0.0;

  std::vector<Eigen::Triplet<double>> at;
  at.emplace_back(0, 0, 1.0); at.emplace_back(0, 1, 1.0);
  at.emplace_back(1, 0, -1.0);
  at.emplace_back(2, 1, -1.0);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(at.begin(), at.end());
  VectorXd b(m); b << 1.0, 0.0, 0.0;
  VectorXd x0(n); x0 << 0.3, 0.3;

  auto result = SolveBarrierQP(Q, c, A, b, x0, 30, 50, 10.0, 1e-8);
  EXPECT_LT(result.x(0), 0.05);
  EXPECT_GE(result.x(0), -0.01);
  printf("QP active: obj=%.6f, x=[%.4f, %.4f], gap=%.2e, "
         "%d outer, %d newton, %.0fus\n",
         result.objective, result.x(0), result.x(1), result.duality_gap,
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

TEST(BarrierQP, SparseQP) {
  srand(42);
  const int n = 20, m = 15;
  std::vector<Eigen::Triplet<double>> qt;
  for (int i = 0; i < n; i++) {
    qt.emplace_back(i, i, 4.0);
    if (i + 1 < n) {
      qt.emplace_back(i, i + 1, -1.0);
      qt.emplace_back(i + 1, i, -1.0);
    }
  }
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());
  VectorXd c = VectorXd::Random(n) * 0.5;

  std::vector<Eigen::Triplet<double>> at;
  for (int r = 0; r < m; r++)
    for (int j = 0; j < 3; j++)
      at.emplace_back(r, rand() % n, (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(at.begin(), at.end());
  VectorXd b = VectorXd::Ones(m);
  VectorXd x0 = VectorXd::Zero(n);

  auto result = SolveBarrierQP(Q, c, A, b, x0, 30, 50, 10.0, 1e-6);
  VectorXd slack = b - A * result.x;
  EXPECT_GE(slack.minCoeff(), -1e-6);
  printf("QP sparse: obj=%.6f, gap=%.2e, slack_min=%.2e, "
         "%d outer, %d newton, %.0fus\n",
         result.objective, result.duality_gap, slack.minCoeff(),
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

TEST(BarrierQP, SolverReuse) {
  const int n = 10, m = 5;
  std::vector<Eigen::Triplet<double>> qt;
  for (int i = 0; i < n; i++) qt.emplace_back(i, i, 2.0);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());
  VectorXd c = VectorXd::Random(n);

  std::vector<Eigen::Triplet<double>> at;
  for (int r = 0; r < m; r++) at.emplace_back(r, r, 1.0);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(at.begin(), at.end());
  VectorXd b = VectorXd::Ones(m) * 5.0;
  VectorXd x0 = VectorXd::Zero(n);

  auto result = SolveBarrierQP(Q, c, A, b, x0, 20, 30, 10.0, 1e-8);
  EXPECT_GT(result.total_newton_steps, 1);
  EXPECT_LT(result.duality_gap, 1e-6);
  VectorXd slack = b - A * result.x;
  EXPECT_GE(slack.minCoeff(), -1e-6);
  printf("QP reuse: obj=%.6f, gap=%.2e, %d outer, %d newton, %.0fus\n",
         result.objective, result.duality_gap,
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

TEST(BarrierQP, ConsolidateMultipleConstraints) {
  // Same QP as UnconstrainedInsideFeasible but with constraints split
  // into two separate AddLinearConstraint calls.
  // min 0.5 x^T I x  s.t.  x1+x2 <= 1, -x1 <= 0, -x2 <= 0
  const int n = 2;

  Eigen::SparseMatrix<double> Q(n, n);
  std::vector<Eigen::Triplet<double>> qt;
  qt.emplace_back(0, 0, 1.0); qt.emplace_back(1, 1, 1.0);
  Q.setFromTriplets(qt.begin(), qt.end());
  VectorXd c = VectorXd::Zero(n);

  // Constraint 1: x1 + x2 <= 1.
  Eigen::SparseMatrix<double> A1(1, n);
  std::vector<Eigen::Triplet<double>> a1t;
  a1t.emplace_back(0, 0, 1.0); a1t.emplace_back(0, 1, 1.0);
  A1.setFromTriplets(a1t.begin(), a1t.end());
  VectorXd b1(1); b1 << 1.0;

  // Constraint 2: -x1 <= 0, -x2 <= 0.
  Eigen::SparseMatrix<double> A2(2, n);
  std::vector<Eigen::Triplet<double>> a2t;
  a2t.emplace_back(0, 0, -1.0);
  a2t.emplace_back(1, 1, -1.0);
  A2.setFromTriplets(a2t.begin(), a2t.end());
  VectorXd b2(2); b2 << 0.0, 0.0;

  std::vector<int> vars = {0, 1};

  Problem problem;
  problem.AddLinearConstraint(A1, b1, vars);
  problem.AddLinearConstraint(A2, b2, vars);
  problem.AddQuadraticCost(Q, vars);

  auto consolidated = problem.Consolidate();
  auto [reduced, expansion] = Preprocess(consolidated);
  VectorXd c_r = expansion.Reduce(c);
  VectorXd x0_r = expansion.Reduce(VectorXd::Constant(n, 0.3));

  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();

  auto c_rhs = kkt->MakeTreeRHS();
  c_rhs = kkt->MakeBlockVariable(c_r);
  auto x = kkt->MakeTreeRHS();
  x = kkt->MakeBlockVariable(x0_r);

  auto result = SolveBarrierQP(*kkt, c_rhs, x);
  result.x = expansion.Expand(result.x);

  EXPECT_NEAR(result.x(0), 0.0, 0.01);
  EXPECT_NEAR(result.x(1), 0.0, 0.01);
  printf("QP consolidated: obj=%.6f, x=[%.4f, %.4f], gap=%.2e, "
         "%d outer, %d newton\n",
         result.objective, result.x(0), result.x(1), result.duality_gap,
         result.outer_iterations, result.total_newton_steps);
}

// =====================================================================
// ProblemSolver tests
// =====================================================================

TEST(ProblemSolver, LeastSquares) {
  srand(42);
  const int m = 30, n = 10;
  Eigen::SparseMatrix<double> A(m, n);
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r)
    for (int c = 0; c < n; ++c)
      if ((r + c) % 3 != 0)
        trips.emplace_back(r, c, (double)rand() / RAND_MAX - 0.5);
  A.setFromTriplets(trips.begin(), trips.end());
  VectorXd b = VectorXd::Random(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  auto c1 = problem.AddLinearConstraint(A, b, vars);

  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.AssembleAndFactor());

  VectorXd rhs_dense = Eigen::MatrixXd(A).transpose() *
                        (Eigen::MatrixXd(A) * VectorXd::Random(n));
  auto rhs = solver.MakeBlockVariable(rhs_dense);
  auto x = solver.MakeBlockVariable();
  solver.SolveInto(rhs, x);

  VectorXd x_dense = solver.Solve(rhs_dense);
  VectorXd x_block = x.Gather();
  double err = (x_block - x_dense).norm() / x_dense.norm();
  EXPECT_LT(err, 1e-10) << "BlockVariable solve doesn't match dense";

  printf("ProblemSolver.LeastSquares: n=%d, solve_err=%.2e\n", n, err);
}

TEST(ProblemSolver, QuadraticCostPlusLinear) {
  srand(42);
  const int m = 20, n = 8;

  MatrixXd A = MatrixXd::Random(m, n);
  VectorXd b = VectorXd::Zero(m);
  MatrixXd Q = MatrixXd::Identity(n, n) * 0.1;

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, b, vars);
  problem.AddQuadraticCost(Q, vars);

  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(n);
  VectorXd x_sol = solver.Solve(rhs);

  MatrixXd M = Q + A.transpose() * A;
  VectorXd x_ref = M.ldlt().solve(rhs);

  double err = (x_sol - x_ref).norm() / x_ref.norm();
  EXPECT_LT(err, 1e-10);

  printf("ProblemSolver.QuadraticCostPlusLinear: n=%d, err=%.2e\n", n, err);
}

TEST(ProblemSolver, SetWeightsAndResolve) {
  srand(42);
  const int m = 20, n = 8;

  MatrixXd A = MatrixXd::Random(m, n);
  VectorXd b = VectorXd::Zero(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  auto c1 = problem.AddLinearConstraint(A, b, vars);

  auto solver = Solver::Build(problem);

  ASSERT_TRUE(solver.AssembleAndFactor());
  VectorXd rhs = VectorXd::Random(n);
  VectorXd x1 = solver.Solve(rhs);

  MatrixXd M1 = A.transpose() * A;
  VectorXd x1_ref = M1.ldlt().solve(rhs);
  EXPECT_LT((x1 - x1_ref).norm() / x1_ref.norm(), 1e-10);

  VectorXd weights(m);
  for (int i = 0; i < m; ++i) weights(i) = i + 1.0;
  solver.SetWeights(c1, weights);
  ASSERT_TRUE(solver.AssembleAndFactor());
  VectorXd x2 = solver.Solve(rhs);

  MatrixXd W = weights.asDiagonal();
  MatrixXd M2 = A.transpose() * W * A;
  VectorXd x2_ref = M2.ldlt().solve(rhs);
  double err = (x2 - x2_ref).norm() / x2_ref.norm();
  EXPECT_LT(err, 1e-10);

  EXPECT_GT((x1 - x2).norm(), 1e-6);

  printf("ProblemSolver.SetWeightsAndResolve: err=%.2e\n", err);
}

TEST(ProblemSolver, Preprocess) {
  srand(42);
  const int m = 10, n = 6;
  Eigen::SparseMatrix<double> A(m, n);
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r)
    for (int c = 0; c < 4; ++c)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX - 0.5);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);

  auto [reduced, expansion] = Preprocess(problem);
  EXPECT_TRUE(expansion.was_reduced());
  EXPECT_EQ(static_cast<int>(expansion.col_map.size()), 4);

  auto solver = Solver::Build(reduced);
  ASSERT_TRUE(solver.AssembleAndFactor());

  VectorXd rhs_full = VectorXd::Random(n);
  VectorXd rhs_reduced = expansion.Reduce(rhs_full);
  VectorXd x_reduced = solver.Solve(rhs_reduced);
  VectorXd x_full = expansion.Expand(x_reduced);

  EXPECT_NEAR(x_full(4), 0.0, 1e-15);
  EXPECT_NEAR(x_full(5), 0.0, 1e-15);

  printf("ProblemSolver.Preprocess: n=%d→%d\n",
         n, static_cast<int>(expansion.col_map.size()));
}

TEST(ProblemSolver, CustomTree) {
  srand(42);
  const int nx = 2;
  MatrixXd Q = MatrixXd::Identity(nx, nx);
  MatrixXd A_couple = MatrixXd::Random(3, 2 * nx);

  std::vector<int> vars0 = {0, 1};
  std::vector<int> vars1 = {2, 3};
  std::vector<int> vars_all = {0, 1, 2, 3};

  Problem problem;
  auto c0 = problem.AddQuadraticCost(Q, vars0);
  auto c1 = problem.AddQuadraticCost(Q, vars1);
  auto c2 = problem.AddLinearConstraint(
      A_couple, VectorXd::Zero(3), vars_all);

  TreeSpec tree;
  int root = tree.AddClique();
  int child = tree.AddClique(root);
  tree.Assign(c0, child);
  tree.Assign(c1, root);
  tree.Assign(c2, child);

  auto solver_custom = Solver::Build(problem, tree);
  ASSERT_TRUE(solver_custom.AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(4);
  VectorXd x_custom = solver_custom.Solve(rhs);

  auto solver_auto = Solver::Build(problem);
  ASSERT_TRUE(solver_auto.AssembleAndFactor());
  VectorXd x_auto = solver_auto.Solve(rhs);

  double err = (x_custom - x_auto).norm() / x_auto.norm();
  EXPECT_LT(err, 1e-10);

  printf("ProblemSolver.CustomTree: err=%.2e\n", err);
}

TEST(ProblemSolver, LQR) {
  srand(42);
  const int nx = 4, nu = 2, T = 20;

  MatrixXd Ad = 0.9 * MatrixXd::Identity(nx, nx) +
                0.1 * MatrixXd::Random(nx, nx);
  MatrixXd Bd = MatrixXd::Random(nx, nu);
  MatrixXd Q = MatrixXd::Identity(nx, nx);
  MatrixXd R = 0.1 * MatrixXd::Identity(nu, nu);
  MatrixXd Qf = 10.0 * Q;
  VectorXd x0 = VectorXd::Ones(nx);

  int step = nx + nu;
  auto x_idx = [&](int t) {
    std::vector<int> v(nx);
    std::iota(v.begin(), v.end(), t * step);
    return v;
  };
  auto u_idx = [&](int t) {
    std::vector<int> v(nu);
    std::iota(v.begin(), v.end(), t * step + nx);
    return v;
  };
  auto xT_idx = [&]() {
    std::vector<int> v(nx);
    std::iota(v.begin(), v.end(), T * step);
    return v;
  };
  auto xu_idx = [&](int t) {
    std::vector<int> v(nx + nu);
    std::iota(v.begin(), v.end(), t * step);
    return v;
  };

  MatrixXd C_dyn(nx, nx + nu + nx);
  C_dyn << -Ad, -Bd, MatrixXd::Identity(nx, nx);
  VectorXd d_zero = VectorXd::Zero(nx);

  MatrixXd QR = MatrixXd::Zero(nx + nu, nx + nu);
  QR.topLeftCorner(nx, nx) = Q;
  QR.bottomRightCorner(nu, nu) = R;

  Problem problem;
  std::vector<ConstraintId> cost_ids, dyn_ids;

  for (int t = 0; t < T; ++t) {
    cost_ids.push_back(problem.AddQuadraticCost(QR, xu_idx(t)));
    std::vector<int> dyn_primal;
    auto xt = x_idx(t), ut = u_idx(t),
         xt1 = (t < T - 1) ? x_idx(t + 1) : xT_idx();
    dyn_primal.insert(dyn_primal.end(), xt.begin(), xt.end());
    dyn_primal.insert(dyn_primal.end(), ut.begin(), ut.end());
    dyn_primal.insert(dyn_primal.end(), xt1.begin(), xt1.end());
    dyn_ids.push_back(problem.AddEqualityConstraint(
        Eigen::SparseMatrix<double>(C_dyn.sparseView()), d_zero, dyn_primal));
  }
  cost_ids.push_back(problem.AddQuadraticCost(Qf, xT_idx()));
  auto c_ic = problem.AddEqualityConstraint(
      Eigen::SparseMatrix<double>(MatrixXd::Identity(nx, nx).sparseView()),
      d_zero, x_idx(0));

  TreeSpec tree;
  std::vector<int> cliques(T + 1);
  cliques[T] = tree.AddClique();
  for (int t = T - 1; t >= 0; --t)
    cliques[t] = tree.AddClique(cliques[t + 1]);

  for (int t = 0; t < T; ++t) {
    tree.Assign(cost_ids[t], cliques[t]);
    tree.Assign(dyn_ids[t], cliques[t]);
  }
  tree.Assign(cost_ids[T], cliques[T]);
  tree.Assign(c_ic, cliques[0]);

  auto solver = Solver::Build(problem, tree);
  ASSERT_TRUE(solver.AssembleAndFactor());

  const auto& ic_duals = solver.dual_variables(c_ic);
  VectorXd rhs = VectorXd::Zero(solver.num_variables());
  for (int i = 0; i < nx; ++i) rhs(ic_duals[i]) = x0(i);

  auto rhs_bv = solver.MakeBlockVariable(rhs);
  auto x_bv = solver.MakeBlockVariable();
  solver.SolveInto(rhs_bv, x_bv);
  VectorXd sol = x_bv.Gather();

  VectorXd x_0_sol(nx);
  auto xi = x_idx(0);
  for (int i = 0; i < nx; ++i) x_0_sol(i) = sol(xi[i]);
  double ic_err = (x_0_sol - x0).norm();
  EXPECT_LT(ic_err, 1e-4) << "Initial condition violated";

  double max_dyn_err = 0;
  for (int t = 0; t < T; ++t) {
    VectorXd xt_sol(nx), ut_sol(nu), xt1_sol(nx);
    auto xti = x_idx(t), uti = u_idx(t);
    auto xt1i = (t < T - 1) ? x_idx(t + 1) : xT_idx();
    for (int i = 0; i < nx; ++i) xt_sol(i) = sol(xti[i]);
    for (int i = 0; i < nu; ++i) ut_sol(i) = sol(uti[i]);
    for (int i = 0; i < nx; ++i) xt1_sol(i) = sol(xt1i[i]);
    double err = (xt1_sol - Ad * xt_sol - Bd * ut_sol).norm();
    max_dyn_err = std::max(max_dyn_err, err);
  }
  EXPECT_LT(max_dyn_err, 1e-4) << "Dynamics violated";

  LQRTreeSolver lqr(Ad, Bd, Q, R, Qf, T);
  lqr.AssembleAndFactor();
  auto sol_ref = lqr.Solve(x0);
  auto x_ref = lqr.ExtractStates(sol_ref);

  double state_err = (x_0_sol - x_ref.col(0)).norm();
  EXPECT_LT(state_err, 1e-8) << "Doesn't match LQRTreeSolver";

  printf("ProblemSolver.LQR: T=%d, ic=%.2e, dyn=%.2e, vs_ref=%.2e\n",
         T, ic_err, max_dyn_err, state_err);
}

TEST(ProblemSolver, EqualityConstrainedLS) {
  srand(42);
  const int m = 15, n = 6, p = 2;

  MatrixXd A = MatrixXd::Random(m, n);
  VectorXd b = VectorXd::Random(m);
  MatrixXd C = MatrixXd::Random(p, n);
  VectorXd d = VectorXd::Random(p);

  std::vector<int> primal_vars(n);
  std::iota(primal_vars.begin(), primal_vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(
      Eigen::SparseMatrix<double>(A.sparseView()),
      VectorXd::Zero(m), primal_vars);
  auto c_eq = problem.AddEqualityConstraint(
      Eigen::SparseMatrix<double>(C.sparseView()),
      d, primal_vars);

  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.AssembleAndFactor());

  const auto& dual_vars = solver.dual_variables(c_eq);
  ASSERT_EQ(static_cast<int>(dual_vars.size()), p);

  int n_total = solver.num_variables();
  VectorXd rhs = VectorXd::Zero(n_total);
  rhs.head(n) = A.transpose() * b;
  for (int i = 0; i < p; ++i) rhs(dual_vars[i]) = d(i);

  auto rhs_bv = solver.MakeBlockVariable(rhs);
  auto x_bv = solver.MakeBlockVariable();
  solver.SolveInto(rhs_bv, x_bv);
  VectorXd sol = x_bv.Gather();

  VectorXd x_sol = sol.head(n);
  VectorXd lam_sol(p);
  for (int i = 0; i < p; ++i) lam_sol(i) = sol(dual_vars[i]);

  double eq_err = (C * x_sol - d).norm();
  EXPECT_LT(eq_err, 1e-10) << "Equality constraint violated";

  double opt_err = (A.transpose() * A * x_sol +
                    C.transpose() * lam_sol -
                    A.transpose() * b).norm();
  EXPECT_LT(opt_err, 1e-10) << "KKT optimality violated";

  MatrixXd K = MatrixXd::Zero(n + p, n + p);
  K.topLeftCorner(n, n) = A.transpose() * A;
  K.topRightCorner(n, p) = C.transpose();
  K.bottomLeftCorner(p, n) = C;
  VectorXd kkt_rhs(n + p);
  kkt_rhs.head(n) = A.transpose() * b;
  kkt_rhs.tail(p) = d;
  VectorXd kkt_sol = K.fullPivLu().solve(kkt_rhs);
  double sol_err = (x_sol - kkt_sol.head(n)).norm() /
                   kkt_sol.head(n).norm();
  EXPECT_LT(sol_err, 1e-10);

  printf("ProblemSolver.EqualityConstrainedLS: eq=%.2e opt=%.2e sol=%.2e\n",
         eq_err, opt_err, sol_err);
}

TEST(ProblemSolver, DenseSolverPD) {
  srand(42);
  const int m = 20, n = 8;
  MatrixXd A = MatrixXd::Random(m, n);
  MatrixXd Q = MatrixXd::Identity(n, n) * 0.1;

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  problem.AddQuadraticCost(Q, vars);

  auto solver = Solver::BuildDense(problem);
  ASSERT_TRUE(solver.AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(n);
  VectorXd x_sol = solver.Solve(rhs);

  MatrixXd M = Q + A.transpose() * A;
  VectorXd x_ref = M.ldlt().solve(rhs);

  double err = (x_sol.head(n) - x_ref).norm() / x_ref.norm();
  EXPECT_LT(err, 1e-10);
  printf("ProblemSolver.DenseSolverPD: err=%.2e\n", err);
}

TEST(ProblemSolver, DenseSolverIndefinite) {
  // min ||Ax-b||^2 s.t. Cx=d via dense solver.
  srand(42);
  const int m = 15, n = 6, p = 2;
  MatrixXd A = MatrixXd::Random(m, n);
  VectorXd b = VectorXd::Random(m);
  MatrixXd C = MatrixXd::Random(p, n);
  VectorXd d = VectorXd::Random(p);

  std::vector<int> primal_vars(n);
  std::iota(primal_vars.begin(), primal_vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(
      Eigen::SparseMatrix<double>(A.sparseView()),
      VectorXd::Zero(m), primal_vars);
  auto c_eq = problem.AddEqualityConstraint(
      Eigen::SparseMatrix<double>(C.sparseView()),
      d, primal_vars);

  auto solver = Solver::BuildDense(problem);
  ASSERT_TRUE(solver.AssembleAndFactor());

  const auto& dual_vars = solver.dual_variables(c_eq);
  int n_total = solver.num_variables();
  VectorXd rhs = VectorXd::Zero(n_total);
  rhs.head(n) = A.transpose() * b;
  for (int i = 0; i < p; ++i) rhs(dual_vars[i]) = d(i);

  // Solve via BlockVariable.
  auto rhs_bv = solver.MakeBlockVariable(rhs);
  auto x_bv = solver.MakeBlockVariable();
  solver.SolveInto(rhs_bv, x_bv);
  VectorXd sol = x_bv.Gather().col(0);

  VectorXd x_sol = sol.head(n);

  // Check equality constraint.
  double eq_err = (C * x_sol - d).norm();
  EXPECT_LT(eq_err, 1e-10) << "Equality constraint violated";

  // Compare with direct dense KKT solve.
  MatrixXd K = MatrixXd::Zero(n + p, n + p);
  K.topLeftCorner(n, n) = A.transpose() * A;
  K.topRightCorner(n, p) = C.transpose();
  K.bottomLeftCorner(p, n) = C;
  VectorXd kkt_rhs(n + p);
  kkt_rhs.head(n) = A.transpose() * b;
  kkt_rhs.tail(p) = d;
  VectorXd kkt_sol = K.fullPivLu().solve(kkt_rhs);
  double sol_err = (x_sol - kkt_sol.head(n)).norm() / kkt_sol.head(n).norm();
  EXPECT_LT(sol_err, 1e-10);

  printf("ProblemSolver.DenseSolverIndefinite: eq=%.2e sol=%.2e\n",
         eq_err, sol_err);
}

TEST(ProblemSolver, RankDeficientEqualities) {
  // Equality constraint C has 4 rows but structural rank 2.
  // Preprocess should drop 2 redundant rows.
  const int m = 20, n = 5;
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r)
    for (int c = 0; c < n; ++c)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX + 0.1);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  // Two independent rows: {0,1} and {2,3}.  Two redundant copies.
  Eigen::SparseMatrix<double> C(4, n);
  std::vector<Eigen::Triplet<double>> ct = {
      {0, 0, 1.0}, {0, 1, 2.0},
      {1, 2, 3.0}, {1, 3, -1.0},
      {2, 0, 0.5}, {2, 1, -1.0},   // same support as row 0
      {3, 2, 2.0}, {3, 3, 1.0}};   // same support as row 1
  C.setFromTriplets(ct.begin(), ct.end());

  VectorXd x_true = VectorXd::Random(n);
  VectorXd d = Eigen::MatrixXd(C) * x_true;
  VectorXd b = Eigen::MatrixXd(A) * x_true;

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  problem.AddEqualityConstraint(C, d, vars);

  auto [reduced, expansion] = Preprocess(problem);
  auto solver = Solver::Build(reduced);
  ASSERT_TRUE(solver.AssembleAndFactor());

  // Build RHS in reduced space.
  VectorXd rhs = VectorXd::Zero(solver.num_variables());
  VectorXd rhs_primal = expansion.Reduce(
      Eigen::MatrixXd(A).transpose() * b);
  rhs.head(rhs_primal.size()) = rhs_primal;

  // Set equality RHS via dual variables.
  // The equality constraint is the second constraint (id=1).
  const auto& duals = solver.dual_variables(1);
  // d was reduced by Preprocess (dropped rows).
  // The reduced equality constraint has fewer rows.
  auto* eq = std::get_if<Problem::EqualityConstraintData>(
      &reduced.constraint(1));
  ASSERT_TRUE(eq != nullptr);
  for (int i = 0; i < static_cast<int>(duals.size()); ++i)
    rhs(duals[i]) = eq->d(i);

  VectorXd sol = solver.Solve(rhs);
  VectorXd x_sol = expansion.Expand(sol.head(expansion.col_map.size()));

  double constraint_err = (Eigen::MatrixXd(C) * x_sol - d).norm();
  EXPECT_LT(constraint_err, 1e-8) << "Equality constraints violated";

  printf("ProblemSolver.RankDeficientEqualities: constraint_err=%.2e\n",
         constraint_err);
}

TEST(ProblemSolver, InconsistentEqualities) {
  // Three rows all touching {x0}: structural rank 1.
  // Row 2 is inconsistent (3*x0 = 10 but should be 9).
  const int n = 3;
  std::vector<int> vars = {0, 1, 2};

  Eigen::SparseMatrix<double> A(10, n);
  std::vector<Eigen::Triplet<double>> at;
  for (int r = 0; r < 10; ++r)
    for (int c = 0; c < n; ++c)
      at.emplace_back(r, c, (double)rand() / RAND_MAX + 0.1);
  A.setFromTriplets(at.begin(), at.end());

  Eigen::SparseMatrix<double> C(3, n);
  std::vector<Eigen::Triplet<double>> ct = {
      {0, 0, 1.0}, {1, 0, 2.0}, {2, 0, 3.0}};
  C.setFromTriplets(ct.begin(), ct.end());
  VectorXd d(3);
  d << 3.0, 6.0, 10.0;  // inconsistent: should be 9.0

  Problem problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(10), vars);
  problem.AddEqualityConstraint(C, d, vars);

  EXPECT_THROW(Preprocess(problem), std::runtime_error);
}

TEST(ProblemSolver, ConsistentEqualities) {
  // Same as above but consistent.
  const int n = 3;
  std::vector<int> vars = {0, 1, 2};

  Eigen::SparseMatrix<double> A(10, n);
  std::vector<Eigen::Triplet<double>> at;
  for (int r = 0; r < 10; ++r)
    for (int c = 0; c < n; ++c)
      at.emplace_back(r, c, (double)rand() / RAND_MAX + 0.1);
  A.setFromTriplets(at.begin(), at.end());

  Eigen::SparseMatrix<double> C(3, n);
  std::vector<Eigen::Triplet<double>> ct = {
      {0, 0, 1.0}, {1, 0, 2.0}, {2, 0, 3.0}};
  C.setFromTriplets(ct.begin(), ct.end());
  VectorXd d(3);
  d << 3.0, 6.0, 9.0;  // consistent

  Problem problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(10), vars);
  problem.AddEqualityConstraint(C, d, vars);

  EXPECT_NO_THROW(Preprocess(problem));
}

TEST(ProblemSolver, BlockDiagonalPattern) {
  // Block-diagonal A: 5 blocks of 8×3.
  srand(99);
  const int num_blocks = 5, rows_per = 8, cols_per = 3;
  const int n = num_blocks * cols_per;
  const int m = num_blocks * rows_per;

  std::vector<Eigen::Triplet<double>> trips;
  for (int b = 0; b < num_blocks; ++b)
    for (int r = 0; r < rows_per; ++r)
      for (int c = 0; c < cols_per; ++c)
        trips.emplace_back(b * rows_per + r, b * cols_per + c,
                           (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = Eigen::MatrixXd(A).transpose() *
                  (Eigen::MatrixXd(A) * x_true);

  Problem problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.AssembleAndFactor());

  VectorXd x_sol = solver.Solve(rhs);
  double err = (x_sol - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-8);
  printf("ProblemSolver.BlockDiagonalPattern: err=%.2e\n", err);
}

TEST(ProblemSolver, BandedPattern) {
  // Banded A: 50 vars, bandwidth 5.
  srand(42);
  const int n = 50, bw = 5, rows_per = 10;
  const int num_groups = n - bw + 1;
  const int m = rows_per * num_groups;

  std::vector<Eigen::Triplet<double>> trips;
  for (int g = 0; g < num_groups; ++g)
    for (int r = 0; r < rows_per; ++r)
      for (int j = 0; j < bw; ++j)
        trips.emplace_back(g * rows_per + r, g + j,
                           0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = Eigen::MatrixXd(A).transpose() *
                  (Eigen::MatrixXd(A) * x_true);

  Problem problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.AssembleAndFactor());

  VectorXd x_sol = solver.Solve(rhs);
  double err = (x_sol - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-8);
  printf("ProblemSolver.BandedPattern: err=%.2e\n", err);
}

TEST(ProblemSolver, QuotientAMD) {
  // PD problem: quotient AMD should match variable-level AMD.
  srand(42);
  const int m = 20, n = 8;
  MatrixXd A = MatrixXd::Random(m, n);
  MatrixXd Q = MatrixXd::Identity(n, n) * 0.1;

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  problem.AddQuadraticCost(Q, vars);

  SolverConfiguration cfg;
  cfg.use_quotient_amd = true;
  auto solver_q = Solver::Build(problem, cfg);
  ASSERT_TRUE(solver_q.AssembleAndFactor());

  cfg.use_quotient_amd = false;
  auto solver_v = Solver::Build(problem, cfg);
  ASSERT_TRUE(solver_v.AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(n);
  VectorXd x_q = solver_q.Solve(rhs);
  VectorXd x_v = solver_v.Solve(rhs);

  // Both should match the reference.
  MatrixXd M = Q + A.transpose() * A;
  VectorXd x_ref = M.ldlt().solve(rhs);
  double err_q = (x_q - x_ref).norm() / x_ref.norm();
  double err_v = (x_v - x_ref).norm() / x_ref.norm();
  EXPECT_LT(err_q, 1e-10);
  EXPECT_LT(err_v, 1e-10);

  printf("ProblemSolver.QuotientAMD: quotient=%.2e variable=%.2e\n",
         err_q, err_v);
}

TEST(ProblemSolver, QuotientAMDChain) {
  // Banded problem: quotient AMD should discover chain structure.
  srand(42);
  const int n = 30, bw = 3, rows_per = 5;
  const int num_groups = n - bw + 1;

  Problem problem;
  for (int g = 0; g < num_groups; ++g) {
    MatrixXd A_block = MatrixXd::Random(rows_per, bw);
    std::vector<int> block_vars(bw);
    std::iota(block_vars.begin(), block_vars.end(), g);
    problem.AddLinearConstraint(A_block, VectorXd::Zero(rows_per), block_vars);
  }

  SolverConfiguration cfg;
  cfg.use_quotient_amd = true;
  auto solver = Solver::Build(problem, cfg);
  ASSERT_TRUE(solver.AssembleAndFactor());

  // Build reference via dense.
  auto solver_d = Solver::BuildDense(problem);
  ASSERT_TRUE(solver_d.AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(n);
  VectorXd x_q = solver.Solve(rhs);
  VectorXd x_d = solver_d.Solve(rhs);

  double err = (x_q - x_d).norm() / x_d.norm();
  EXPECT_LT(err, 1e-10);
  printf("ProblemSolver.QuotientAMDChain: n=%d, err=%.2e\n", n, err);
}

TEST(ProblemSolver, MultiThreaded) {
  // Same banded problem solved with 1, 2, and 4 threads.
  srand(42);
  const int n = 50, bw = 5, rows_per = 10;
  const int num_groups = n - bw + 1;
  const int m = rows_per * num_groups;

  std::vector<Eigen::Triplet<double>> trips;
  for (int g = 0; g < num_groups; ++g)
    for (int r = 0; r < rows_per; ++r)
      for (int j = 0; j < bw; ++j)
        trips.emplace_back(g * rows_per + r, g + j,
                           0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = Eigen::MatrixXd(A).transpose() *
                  (Eigen::MatrixXd(A) * x_true);

  VectorXd x_ref;
  for (int threads : {1, 2, 4}) {
    Problem problem;
    problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);

    SolverConfiguration cfg;
    cfg.num_threads = threads;
    auto solver = Solver::Build(problem, cfg);
    ASSERT_TRUE(solver.AssembleAndFactor());

    VectorXd x_sol = solver.Solve(rhs);
    if (threads == 1) {
      x_ref = x_sol;
    } else {
      double err = (x_sol - x_ref).norm() / x_ref.norm();
      EXPECT_LT(err, 1e-10)
          << "Multi-threaded result differs at threads=" << threads;
    }
  }
  double err = (x_ref - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-8);
  printf("ProblemSolver.MultiThreaded: err=%.2e\n", err);
}

TEST(ProblemSolver, PQTreeReorder) {
  // Use PQ-tree supernode reordering (method 1).
  srand(42);
  const int n = 30, bw = 4, rows_per = 6;
  const int num_groups = n - bw + 1;
  const int m = rows_per * num_groups;

  std::vector<Eigen::Triplet<double>> trips;
  for (int g = 0; g < num_groups; ++g)
    for (int r = 0; r < rows_per; ++r)
      for (int j = 0; j < bw; ++j)
        trips.emplace_back(g * rows_per + r, g + j,
                           0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = Eigen::MatrixXd(A).transpose() *
                  (Eigen::MatrixXd(A) * x_true);

  // Solve with each reorder method and verify all match.
  VectorXd x_ref;
  const char* names[] = {"BFS_GREEDY", "PQ_TREE", "NONE", "BFS_GREEDY_LARGEST"};
  for (int method = 0; method <= 3; ++method) {
    Problem problem;
    problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);

    SolverConfiguration cfg;
    cfg.tree.supernode_reorder_method = method;
    auto solver = Solver::Build(problem, cfg);
    ASSERT_TRUE(solver.AssembleAndFactor());

    VectorXd x_sol = solver.Solve(rhs);
    if (method == 0) {
      x_ref = x_sol;
    }
    double err = (x_sol - x_true).norm() / x_true.norm();
    EXPECT_LT(err, 1e-8) << "Failed with reorder method " << names[method];
  }
  printf("ProblemSolver.PQTreeReorder: all 4 methods match (err<1e-8)\n");
}

// Scenario tree for stochastic optimization.
struct ScenarioNode {
  int parent;
  std::vector<int> children;
  int stage;
};

std::vector<ScenarioNode> MakeScenarioTree(int B, int S) {
  std::vector<ScenarioNode> nodes;
  nodes.push_back({-1, {}, 0});
  for (int s = 1; s < S; ++s) {
    int prev_end = static_cast<int>(nodes.size());
    for (int i = 0; i < prev_end; ++i) {
      if (nodes[i].stage != s - 1) continue;
      for (int b = 0; b < B; ++b) {
        int child_id = static_cast<int>(nodes.size());
        nodes[i].children.push_back(child_id);
        nodes.push_back({i, {}, s});
      }
    }
  }
  return nodes;
}

TEST(ProblemSolver, StochasticCustomVsAutomatic) {
  // Compare custom tree vs automatic for stochastic optimization.
  using clock = std::chrono::high_resolution_clock;
  srand(42);

  const int nx = 4, nu = 2, B = 3;
  MatrixXd Ad = 0.9 * MatrixXd::Identity(nx, nx) +
                0.1 * MatrixXd::Random(nx, nx);
  MatrixXd Bd = MatrixXd::Random(nx, nu);
  MatrixXd Q = MatrixXd::Identity(nx, nx) + 0.5 * MatrixXd::Ones(nx, nx);
  MatrixXd R = 0.1 * MatrixXd::Identity(nu, nu) + 0.05 * MatrixXd::Ones(nu, nu);
  MatrixXd Qf = 10.0 * Q;
  VectorXd x0 = VectorXd::Ones(nx);

  MatrixXd C_dyn(nx, nx + nu + nx);
  C_dyn << -Ad, -Bd, MatrixXd::Identity(nx, nx);
  VectorXd d_zero = VectorXd::Zero(nx);

  MatrixXd QR = MatrixXd::Zero(nx + nu, nx + nu);
  QR.topLeftCorner(nx, nx) = Q;
  QR.bottomRightCorner(nu, nu) = R;

  printf("\n  Stochastic opt (branch=%d): custom tree vs automatic\n", B);
  printf("%-7s %6s %22s %22s %8s\n",
         "stages", "nodes",
         "custom(build+fac+sol)", "auto(build+fac+sol)", "speedup");
  printf("-------  ------ ---------------------- "
         "---------------------- --------\n");

  for (int S : {3, 4, 5, 6}) {
    auto scenario = MakeScenarioTree(B, S);
    int N = static_cast<int>(scenario.size());

    // Variable layout: x_t at [t*step .. t*step+nx-1],
    //                  u_t at [t*step+nx .. t*step+nx+nu-1] (non-leaf).
    int step = nx + nu;
    auto x_idx = [&](int i) {
      std::vector<int> v(nx);
      std::iota(v.begin(), v.end(), i * step);
      return v;
    };
    auto u_idx = [&](int i) {
      std::vector<int> v(nu);
      std::iota(v.begin(), v.end(), i * step + nx);
      return v;
    };
    auto xu_idx = [&](int i) {
      std::vector<int> v(nx + nu);
      std::iota(v.begin(), v.end(), i * step);
      return v;
    };
    int n_primal = N * step;

    // --- Custom tree path ---
    Problem prob_c;
    TreeSpec tree_spec;
    std::vector<int> cliques(N);
    cliques[0] = tree_spec.AddClique();
    for (int i = 1; i < N; ++i)
      cliques[i] = tree_spec.AddClique(cliques[scenario[i].parent]);

    ConstraintId c_ic;
    for (int i = 0; i < N; ++i) {
      bool is_leaf = scenario[i].children.empty();
      bool is_root = (scenario[i].parent == -1);

      if (is_leaf) {
        tree_spec.Assign(prob_c.AddQuadraticCost(Qf, x_idx(i)), cliques[i]);
      } else {
        tree_spec.Assign(prob_c.AddQuadraticCost(QR, xu_idx(i)), cliques[i]);
      }

      if (is_root) {
        c_ic = prob_c.AddEqualityConstraint(
            Eigen::SparseMatrix<double>(
                MatrixXd::Identity(nx, nx).sparseView()),
            d_zero, x_idx(0));
        tree_spec.Assign(c_ic, cliques[0]);
      } else {
        int p = scenario[i].parent;
        std::vector<int> dp;
        auto xp = x_idx(p), up = u_idx(p), xi = x_idx(i);
        dp.insert(dp.end(), xp.begin(), xp.end());
        dp.insert(dp.end(), up.begin(), up.end());
        dp.insert(dp.end(), xi.begin(), xi.end());
        tree_spec.Assign(prob_c.AddEqualityConstraint(
            Eigen::SparseMatrix<double>(C_dyn.sparseView()),
            d_zero, dp), cliques[i]);
      }
    }

    auto tc0 = clock::now();
    auto solver_c = Solver::Build(prob_c, tree_spec);
    solver_c.AssembleAndFactor();
    const auto& ic_duals = solver_c.dual_variables(c_ic);
    VectorXd rhs_c = VectorXd::Zero(solver_c.num_variables());
    for (int j = 0; j < nx; ++j) rhs_c(ic_duals[j]) = x0(j);
    VectorXd sol_c = solver_c.Solve(rhs_c);
    auto tc1 = clock::now();
    double custom_us = std::chrono::duration<double, std::micro>(tc1 - tc0).count();

    // Verify IC.
    VectorXd x0_c(nx);
    auto xi0 = x_idx(0);
    for (int j = 0; j < nx; ++j) x0_c(j) = sol_c(xi0[j]);
    EXPECT_LT((x0_c - x0).norm(), 1e-4) << "Custom IC at S=" << S;

    // --- Automatic path (single sparse Q + C) ---
    // Build sparse Q_cost and C_eq over all nodes.
    std::vector<Eigen::Triplet<double>> qt, ct;
    for (int i = 0; i < N; ++i) {
      bool is_leaf = scenario[i].children.empty();
      auto xi = x_idx(i);
      const auto& Qi = is_leaf ? Qf : Q;
      for (int r = 0; r < nx; ++r)
        for (int c = 0; c < nx; ++c)
          if (Qi(r,c) != 0) qt.emplace_back(xi[r], xi[c], Qi(r,c));
      if (!is_leaf) {
        auto ui = u_idx(i);
        for (int r = 0; r < nu; ++r)
          for (int c = 0; c < nu; ++c)
            if (R(r,c) != 0) qt.emplace_back(ui[r], ui[c], R(r,c));
      }
    }
    Eigen::SparseMatrix<double> Q_cost(n_primal, n_primal);
    Q_cost.setFromTriplets(qt.begin(), qt.end());

    int eq_row = 0;
    for (int i = 1; i < N; ++i) {
      int p = scenario[i].parent;
      auto xp = x_idx(p), up = u_idx(p), xi = x_idx(i);
      for (int r = 0; r < nx; ++r) {
        for (int c = 0; c < nx; ++c)
          if (Ad(r,c) != 0) ct.emplace_back(eq_row+r, xp[c], -Ad(r,c));
        for (int c = 0; c < nu; ++c)
          if (Bd(r,c) != 0) ct.emplace_back(eq_row+r, up[c], -Bd(r,c));
        ct.emplace_back(eq_row+r, xi[r], 1.0);
      }
      eq_row += nx;
    }
    for (int j = 0; j < nx; ++j)
      ct.emplace_back(eq_row + j, x_idx(0)[j], 1.0);
    int n_eq = eq_row + nx;
    Eigen::SparseMatrix<double> C_eq(n_eq, n_primal);
    C_eq.setFromTriplets(ct.begin(), ct.end());
    VectorXd d_eq = VectorXd::Zero(n_eq);
    d_eq.tail(nx) = x0;

    std::vector<int> all_vars(n_primal);
    std::iota(all_vars.begin(), all_vars.end(), 0);

    Problem prob_a;
    prob_a.AddQuadraticCost(Q_cost, all_vars);
    auto ceq_a = prob_a.AddEqualityConstraint(C_eq, d_eq, all_vars);

    auto ta0 = clock::now();
    auto solver_a = Solver::Build(prob_a);
    solver_a.AssembleAndFactor();
    const auto& duals_a = solver_a.dual_variables(ceq_a);
    VectorXd rhs_a = VectorXd::Zero(solver_a.num_variables());
    for (int j = 0; j < n_eq; ++j) rhs_a(duals_a[j]) = d_eq(j);
    VectorXd sol_a = solver_a.Solve(rhs_a);
    auto ta1 = clock::now();
    double auto_us = std::chrono::duration<double, std::micro>(ta1 - ta0).count();

    VectorXd x0_a(nx);
    for (int j = 0; j < nx; ++j) x0_a(j) = sol_a(xi0[j]);
    EXPECT_LT((x0_a - x0).norm(), 1e-4) << "Auto IC at S=" << S;

    printf("%-7d %6d %12.0f          %12.0f          %7.1fx\n",
           S, N, custom_us, auto_us, auto_us / custom_us);
  }
}

TEST(ProblemSolver, LQRCustomVsAutomatic) {
  // Compare custom tree (TreeSpec) vs automatic (AMD) for LQR.
  // Custom tree knows the chain structure; automatic discovers it.
  using clock = std::chrono::high_resolution_clock;
  srand(42);

  const int nx = 4, nu = 2;
  MatrixXd Ad = 0.9 * MatrixXd::Identity(nx, nx) +
                0.1 * MatrixXd::Random(nx, nx);
  MatrixXd Bd = MatrixXd::Random(nx, nu);
  MatrixXd Q = MatrixXd::Identity(nx, nx) + 0.5 * MatrixXd::Ones(nx, nx);
  MatrixXd R = 0.1 * MatrixXd::Identity(nu, nu) + 0.05 * MatrixXd::Ones(nu, nu);
  MatrixXd Qf = 10.0 * Q;
  VectorXd x0 = VectorXd::Ones(nx);

  printf("\n  LQR: custom tree (TreeSpec) vs automatic (AMD)\n");
  printf("%-6s %22s %22s %8s\n",
         "T", "custom(build+fac+sol)", "auto(build+fac+sol)", "speedup");
  printf("------  ---------------------- ---------------------- --------\n");

  for (int T : {10, 50, 100, 200}) {
    int step = nx + nu;
    int n_vars = (T + 1) * nx + T * nu;

    auto x_idx = [&](int t) -> std::vector<int> {
      std::vector<int> v(nx);
      int base = t * (nx + nu);
      std::iota(v.begin(), v.end(), base);
      return v;
    };
    auto u_idx = [&](int t) -> std::vector<int> {
      std::vector<int> v(nu);
      std::iota(v.begin(), v.end(), t * (nx + nu) + nx);
      return v;
    };
    auto xT_idx = [&]() -> std::vector<int> {
      std::vector<int> v(nx);
      std::iota(v.begin(), v.end(), T * (nx + nu));
      return v;
    };

    // Build Problem (shared by both paths).
    auto build_problem = [&]() {
      Problem problem;
      std::vector<ConstraintId> cost_ids, dyn_ids;

      // Sparse cost Q_cost.
      std::vector<Eigen::Triplet<double>> qt;
      for (int t = 0; t < T; ++t) {
        auto xi = x_idx(t), ui = u_idx(t);
        for (int i = 0; i < nx; ++i)
          for (int j = 0; j < nx; ++j)
            if (Q(i,j) != 0) qt.emplace_back(xi[i], xi[j], Q(i,j));
        for (int i = 0; i < nu; ++i)
          for (int j = 0; j < nu; ++j)
            if (R(i,j) != 0) qt.emplace_back(ui[i], ui[j], R(i,j));
      }
      auto xTi = xT_idx();
      for (int i = 0; i < nx; ++i)
        for (int j = 0; j < nx; ++j)
          if (Qf(i,j) != 0) qt.emplace_back(xTi[i], xTi[j], Qf(i,j));
      Eigen::SparseMatrix<double> Q_cost(n_vars, n_vars);
      Q_cost.setFromTriplets(qt.begin(), qt.end());

      std::vector<int> all_vars(n_vars);
      std::iota(all_vars.begin(), all_vars.end(), 0);
      problem.AddQuadraticCost(Q_cost, all_vars);

      // Dynamics + IC as one equality constraint.
      int n_eq = (T + 1) * nx;
      std::vector<Eigen::Triplet<double>> ct;
      for (int t = 0; t < T; ++t) {
        int rb = t * nx;
        auto xi = x_idx(t), ui = u_idx(t);
        auto xi1 = (t < T-1) ? x_idx(t+1) : xT_idx();
        for (int r = 0; r < nx; ++r)
          for (int c = 0; c < nx; ++c)
            if (Ad(r,c) != 0) ct.emplace_back(rb+r, xi[c], -Ad(r,c));
        for (int r = 0; r < nx; ++r)
          for (int c = 0; c < nu; ++c)
            if (Bd(r,c) != 0) ct.emplace_back(rb+r, ui[c], -Bd(r,c));
        for (int i = 0; i < nx; ++i)
          ct.emplace_back(rb+i, xi1[i], 1.0);
      }
      int ic_row = T * nx;
      for (int i = 0; i < nx; ++i)
        ct.emplace_back(ic_row+i, x_idx(0)[i], 1.0);
      Eigen::SparseMatrix<double> C_eq(n_eq, n_vars);
      C_eq.setFromTriplets(ct.begin(), ct.end());

      VectorXd d_eq = VectorXd::Zero(n_eq);
      d_eq.tail(nx) = x0;

      auto c_eq = problem.AddEqualityConstraint(C_eq, d_eq, all_vars);
      return std::make_pair(problem, c_eq);
    };

    // --- Custom tree path ---
    auto [prob_c, ceq_c] = build_problem();

    // TreeSpec: chain 0 → 1 → ... → T.
    // Each clique t gets cost(t) + dynamics(t).
    // But we have ONE big Q and ONE big C, not per-timestep.
    // For custom tree, we need per-timestep constraints.
    // Build a separate problem with per-timestep blocks.
    Problem prob_custom;
    TreeSpec tree;
    std::vector<int> cliques(T + 1);
    cliques[T] = tree.AddClique();
    for (int t = T-1; t >= 0; --t)
      cliques[t] = tree.AddClique(cliques[t+1]);

    MatrixXd QR = MatrixXd::Zero(nx+nu, nx+nu);
    QR.topLeftCorner(nx,nx) = Q;
    QR.bottomRightCorner(nu,nu) = R;

    MatrixXd C_dyn(nx, nx+nu+nx);
    C_dyn << -Ad, -Bd, MatrixXd::Identity(nx,nx);
    VectorXd d_zero = VectorXd::Zero(nx);

    for (int t = 0; t < T; ++t) {
      std::vector<int> xu;
      auto xi = x_idx(t), ui = u_idx(t);
      xu.insert(xu.end(), xi.begin(), xi.end());
      xu.insert(xu.end(), ui.begin(), ui.end());
      tree.Assign(prob_custom.AddQuadraticCost(QR, xu), cliques[t]);

      std::vector<int> dyn_p;
      auto xi1 = (t < T-1) ? x_idx(t+1) : xT_idx();
      dyn_p.insert(dyn_p.end(), xi.begin(), xi.end());
      dyn_p.insert(dyn_p.end(), ui.begin(), ui.end());
      dyn_p.insert(dyn_p.end(), xi1.begin(), xi1.end());
      tree.Assign(prob_custom.AddEqualityConstraint(
          Eigen::SparseMatrix<double>(C_dyn.sparseView()),
          d_zero, dyn_p), cliques[t]);
    }
    tree.Assign(prob_custom.AddQuadraticCost(Qf, xT_idx()), cliques[T]);
    auto c_ic = prob_custom.AddEqualityConstraint(
        Eigen::SparseMatrix<double>(MatrixXd::Identity(nx,nx).sparseView()),
        d_zero, x_idx(0));
    tree.Assign(c_ic, cliques[0]);

    auto tc0 = clock::now();
    auto solver_c = Solver::Build(prob_custom, tree);
    auto tc1 = clock::now();
    solver_c.AssembleAndFactor();
    auto tc2 = clock::now();
    const auto& ic_duals = solver_c.dual_variables(c_ic);
    VectorXd rhs_c = VectorXd::Zero(solver_c.num_variables());
    for (int i = 0; i < nx; ++i) rhs_c(ic_duals[i]) = x0(i);
    VectorXd sol_c = solver_c.Solve(rhs_c);
    auto tc3 = clock::now();
    double custom_us = std::chrono::duration<double, std::micro>(tc3 - tc0).count();

    // --- Automatic path ---
    auto [prob_a, ceq_a] = build_problem();
    auto ta0 = clock::now();
    auto solver_a = Solver::Build(prob_a);
    auto ta1 = clock::now();
    solver_a.AssembleAndFactor();
    auto ta2 = clock::now();
    const auto& duals_a = solver_a.dual_variables(ceq_a);
    VectorXd rhs_a = VectorXd::Zero(solver_a.num_variables());
    int n_eq = (T+1)*nx;
    VectorXd d_eq = VectorXd::Zero(n_eq);
    d_eq.tail(nx) = x0;
    for (int i = 0; i < n_eq; ++i) rhs_a(duals_a[i]) = d_eq(i);
    VectorXd sol_a = solver_a.Solve(rhs_a);
    auto ta3 = clock::now();
    double auto_us = std::chrono::duration<double, std::micro>(ta3 - ta0).count();

    // Verify both produce reasonable solutions.
    VectorXd x0_c(nx), x0_a(nx);
    auto xi0 = x_idx(0);
    for (int i = 0; i < nx; ++i) x0_c(i) = sol_c(xi0[i]);
    for (int i = 0; i < nx; ++i) x0_a(i) = sol_a(xi0[i]);
    EXPECT_LT((x0_c - x0).norm(), 1e-4) << "Custom IC at T=" << T;
    EXPECT_LT((x0_a - x0).norm(), 1e-4) << "Auto IC at T=" << T;

    printf("%-6d %12.0f          %12.0f          %7.1fx\n",
           T, custom_us, auto_us, auto_us / custom_us);
  }
}

// =====================================================================
// Gaussian MRF on a tree: purely PD, no equality constraints.
// =====================================================================

struct TreeGraph {
  int num_nodes;
  std::vector<std::pair<int, int>> edges;
  std::vector<int> parent;
};

TreeGraph MakeBalancedTree(int B, int D) {
  TreeGraph g;
  g.parent.push_back(-1);
  for (int d = 1; d < D; ++d) {
    int prev_start = 0, prev_end = static_cast<int>(g.parent.size());
    for (int i = prev_start; i < prev_end; ++i) {
      if (static_cast<int>(g.parent.size()) - i >
          prev_end - prev_start) continue;
      bool is_leaf = true;
      for (int j = prev_end; j < static_cast<int>(g.parent.size()); ++j)
        if (g.parent[j] == i) { is_leaf = false; break; }
      if (!is_leaf) continue;
      for (int b = 0; b < B; ++b) {
        int child = static_cast<int>(g.parent.size());
        g.edges.push_back({i, child});
        g.parent.push_back(i);
      }
    }
  }
  g.num_nodes = static_cast<int>(g.parent.size());
  g.edges.clear();
  for (int i = 1; i < g.num_nodes; ++i)
    g.edges.push_back({g.parent[i], i});
  return g;
}

TEST(ProblemSolver, GaussianMRF) {
  using clock = std::chrono::high_resolution_clock;
  srand(42);

  const int d = 4;

  auto var_idx = [&](int v) -> std::vector<int> {
    std::vector<int> idx(d);
    std::iota(idx.begin(), idx.end(), v * d);
    return idx;
  };

  auto make_node_potential = [&]() {
    MatrixXd R = MatrixXd::Random(d, d) * 0.5;
    MatrixXd RtR = R.transpose() * R;
    return MatrixXd(MatrixXd::Identity(d, d) + RtR);
  };

  auto make_edge_potential = [&]() {
    MatrixXd R = MatrixXd::Random(2 * d, 2 * d) * 0.3;
    MatrixXd RtR = R.transpose() * R;
    return MatrixXd(MatrixXd::Identity(2 * d, 2 * d) + RtR);
  };

  printf("\n  Gaussian MRF (d=%d): Problem + Solver + TreeSpec\n", d);
  printf("%-6s %-6s %6s %7s  %8s %8s %8s  %10s\n",
         "B", "D", "nodes", "n_vars", "build", "factor", "solve", "residual");
  printf("------  ------ ------ -------  -------- -------- --------"
         "  ----------\n");

  for (auto [B, D] : std::vector<std::pair<int,int>>{{2,5},{3,4},{2,8},{3,5},{2,10}}) {
    auto graph = MakeBalancedTree(B, D);
    int N = graph.num_nodes;
    int n_vars = N * d;

    auto t0 = clock::now();

    Problem problem;
    TreeSpec tree;

    std::vector<int> cliques(N);
    cliques[0] = tree.AddClique();
    for (int c = 1; c < N; ++c)
      cliques[c] = tree.AddClique(cliques[graph.parent[c]]);

    MatrixXd Q0 = make_node_potential();
    auto c_root = problem.AddQuadraticCost(Q0, var_idx(0));
    tree.Assign(c_root, cliques[0]);

    std::vector<MatrixXd> node_pots(N), edge_pots(N);
    node_pots[0] = Q0;

    for (int c = 1; c < N; ++c) {
      int p = graph.parent[c];
      std::vector<int> edge_vars;
      auto vp = var_idx(p), vc = var_idx(c);
      edge_vars.insert(edge_vars.end(), vp.begin(), vp.end());
      edge_vars.insert(edge_vars.end(), vc.begin(), vc.end());
      MatrixXd Qe = make_edge_potential();
      edge_pots[c] = Qe;
      auto ce = problem.AddQuadraticCost(Qe, edge_vars);
      tree.Assign(ce, cliques[c]);

      MatrixXd Qn = make_node_potential();
      node_pots[c] = Qn;
      auto cn = problem.AddQuadraticCost(Qn, var_idx(c));
      tree.Assign(cn, cliques[c]);
    }

    auto solver = Solver::Build(problem, tree);
    auto t1 = clock::now();
    ASSERT_TRUE(solver.AssembleAndFactor());
    auto t2 = clock::now();

    VectorXd h = VectorXd::Random(n_vars);
    auto rhs_bv = solver.MakeBlockVariable(h);
    auto x_bv = solver.MakeBlockVariable();
    solver.SolveInto(rhs_bv, x_bv);
    VectorXd x = x_bv.Gather().col(0);
    auto t3 = clock::now();

    // Verify: J*x = h.
    VectorXd Jx = VectorXd::Zero(n_vars);
    for (int v = 0; v < N; ++v) {
      auto vi = var_idx(v);
      VectorXd xv(d);
      for (int i = 0; i < d; ++i) xv(i) = x(vi[i]);
      VectorXd contrib = node_pots[v] * xv;
      for (int i = 0; i < d; ++i) Jx(vi[i]) += contrib(i);
    }
    for (int c = 1; c < N; ++c) {
      int p = graph.parent[c];
      auto vp = var_idx(p), vc = var_idx(c);
      VectorXd xpxc(2 * d);
      for (int i = 0; i < d; ++i) xpxc(i) = x(vp[i]);
      for (int i = 0; i < d; ++i) xpxc(d + i) = x(vc[i]);
      VectorXd contrib = edge_pots[c] * xpxc;
      for (int i = 0; i < d; ++i) Jx(vp[i]) += contrib(i);
      for (int i = 0; i < d; ++i) Jx(vc[i]) += contrib(d + i);
    }

    double residual = (Jx - h).norm() / h.norm();
    EXPECT_LT(residual, 1e-10) << "J*x != h";

    printf("%-6d %-6d %6d %7d  %7.0fus %7.0fus %7.0fus  %.2e\n",
           B, D, N, n_vars,
           std::chrono::duration<double, std::micro>(t1 - t0).count(),
           std::chrono::duration<double, std::micro>(t2 - t1).count(),
           std::chrono::duration<double, std::micro>(t3 - t2).count(),
           residual);
  }
}

}  // namespace
}  // namespace conex
