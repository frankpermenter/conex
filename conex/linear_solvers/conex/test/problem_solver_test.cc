#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/lqr_tree_solver.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

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

}  // namespace
}  // namespace conex
