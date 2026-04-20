#include <gtest/gtest.h>

#include <chrono>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/barrier_qp.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/irls.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/common/eja_ops.h"
#include "conex/algorithms/lqr_tree_solver.h"
#include "conex/common/clique_ordering.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/tree_solver/kkt_solver_factory.h"

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

// Common result type for parameterized QP solver tests.
struct QPSolution {
  Eigen::VectorXd x;
  double objective = 0;
  double gap = 0;
};

// Solver function: (Model, x0) -> QPSolution.
using QPSolverFn = std::function<QPSolution(const Model&, const VectorXd&)>;

static QPSolution SolveBarrierFromProblem(
    const Model& problem, const Eigen::VectorXd& x0) {
  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto c_rhs = solver.MakeCostRHS();
  auto x = kkt->MakeSolverRHS();
  x = kkt->MakeBlockVariable(solver.ReduceVector(x0));

  auto result = SolveBarrierQP(*kkt, c_rhs, x);
  QPSolution sol;
  sol.x = solver.ExpandSolution(result.x);
  VectorXd c = problem.has_linear_cost()
      ? problem.linear_cost() : VectorXd::Zero(problem.num_variables());
  sol.objective = c.dot(sol.x);
  sol.gap = result.duality_gap;
  return sol;
}

// Parameterized test fixture.
class QPSolverTest : public ::testing::TestWithParam<QPSolverFn> {};

TEST_P(QPSolverTest, UnconstrainedInsideFeasible) {
  // min 0.5 x^T I x  s.t.  x1+x2 <= 1, -x1 <= 0, -x2 <= 0
  const int n = 2;
  Eigen::SparseMatrix<double> Q(n, n);
  std::vector<Eigen::Triplet<double>> qt;
  qt.emplace_back(0, 0, 1.0); qt.emplace_back(1, 1, 1.0);
  Q.setFromTriplets(qt.begin(), qt.end());

  std::vector<Eigen::Triplet<double>> at;
  at.emplace_back(0, 0, 1.0); at.emplace_back(0, 1, 1.0);
  at.emplace_back(1, 0, -1.0);
  at.emplace_back(2, 1, -1.0);
  Eigen::SparseMatrix<double> A(3, n);
  A.setFromTriplets(at.begin(), at.end());
  VectorXd b(3); b << 1.0, 0.0, 0.0;

  Model problem;
  problem.AddLinearConstraint(A, b, Sense::LE);
  problem.AddQuadraticCost(Q);
  problem.SetLinearCost(VectorXd::Zero(n));

  VectorXd x0(n); x0 << 0.3, 0.3;
  auto sol = GetParam()(problem, x0);
  EXPECT_NEAR(sol.x(0), 0.0, 0.01);
  EXPECT_NEAR(sol.x(1), 0.0, 0.01);
  printf("  obj=%.6f, x=[%.4f, %.4f], gap=%.2e\n",
         sol.objective, sol.x(0), sol.x(1), sol.gap);
}

TEST_P(QPSolverTest, ActiveConstraint) {
  const int n = 2;
  std::vector<Eigen::Triplet<double>> qt;
  qt.emplace_back(0, 0, 0.001); qt.emplace_back(1, 1, 0.001);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());
  VectorXd c(n); c << 1.0, 0.0;

  std::vector<Eigen::Triplet<double>> at;
  at.emplace_back(0, 0, 1.0); at.emplace_back(0, 1, 1.0);
  at.emplace_back(1, 0, -1.0);
  at.emplace_back(2, 1, -1.0);
  Eigen::SparseMatrix<double> A(3, n);
  A.setFromTriplets(at.begin(), at.end());
  VectorXd b(3); b << 1.0, 0.0, 0.0;

  Model problem;
  problem.AddLinearConstraint(A, b, Sense::LE);
  problem.AddQuadraticCost(Q);
  problem.SetLinearCost(c);

  VectorXd x0(n); x0 << 0.3, 0.3;
  auto sol = GetParam()(problem, x0);
  EXPECT_LT(sol.x(0), 0.05);
  EXPECT_GE(sol.x(0), -0.01);
  printf("  obj=%.6f, x=[%.4f, %.4f], gap=%.2e\n",
         sol.objective, sol.x(0), sol.x(1), sol.gap);
}

TEST_P(QPSolverTest, MultipleConstraints) {
  // Same QP as UnconstrainedInsideFeasible but split into two constraints.
  const int n = 2;
  Eigen::SparseMatrix<double> Q(n, n);
  std::vector<Eigen::Triplet<double>> qt;
  qt.emplace_back(0, 0, 1.0); qt.emplace_back(1, 1, 1.0);
  Q.setFromTriplets(qt.begin(), qt.end());

  Eigen::SparseMatrix<double> A1(1, n);
  std::vector<Eigen::Triplet<double>> a1t;
  a1t.emplace_back(0, 0, 1.0); a1t.emplace_back(0, 1, 1.0);
  A1.setFromTriplets(a1t.begin(), a1t.end());
  VectorXd b1(1); b1 << 1.0;

  Eigen::SparseMatrix<double> A2(2, n);
  std::vector<Eigen::Triplet<double>> a2t;
  a2t.emplace_back(0, 0, -1.0);
  a2t.emplace_back(1, 1, -1.0);
  A2.setFromTriplets(a2t.begin(), a2t.end());
  VectorXd b2(2); b2 << 0.0, 0.0;

  std::vector<int> vars = {0, 1};
  Model problem;
  problem.AddLinearConstraint(A1, b1, Sense::LE, vars);
  problem.AddLinearConstraint(A2, b2, Sense::LE, vars);
  problem.AddQuadraticCost(Q, vars);
  problem.SetLinearCost(VectorXd::Zero(n));

  VectorXd x0 = VectorXd::Constant(n, 0.3);
  auto sol = GetParam()(problem, x0);
  EXPECT_NEAR(sol.x(0), 0.0, 0.01);
  EXPECT_NEAR(sol.x(1), 0.0, 0.01);
  printf("  obj=%.6f, x=[%.4f, %.4f], gap=%.2e\n",
         sol.objective, sol.x(0), sol.x(1), sol.gap);
}

static QPSolution SolveGeodesicFromProblem(
    const Model& problem, const Eigen::VectorXd& x0) {
  (void)x0;  // geodesic IPM initializes at W=ones, ignores x0
  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto cost_rhs = solver.MakeCostRHS();

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 0, 1e-8);

  QPSolution sol;
  sol.x = solver.ExpandSolution(result.x);
  VectorXd c = problem.has_linear_cost()
      ? problem.linear_cost() : VectorXd::Zero(problem.num_variables());
  sol.objective = c.dot(sol.x);
  sol.gap = result.complementarity;
  return sol;
}

INSTANTIATE_TEST_SUITE_P(BarrierQP, QPSolverTest,
    ::testing::Values(SolveBarrierFromProblem));
INSTANTIATE_TEST_SUITE_P(GeodesicIPM, QPSolverTest,
    ::testing::Values(SolveGeodesicFromProblem));

// Barrier-specific tests (not parameterized).
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

  Model problem;
  problem.AddLinearConstraint(A, b, Sense::LE);
  problem.AddQuadraticCost(Q);
  problem.SetLinearCost(c);

  VectorXd x0 = VectorXd::Zero(n);
  auto sol = SolveBarrierFromProblem(problem, x0);
  VectorXd slack = b - A * sol.x;
  EXPECT_GE(slack.minCoeff(), -1e-6);
  printf("QP sparse: obj=%.6f, gap=%.2e, slack_min=%.2e\n",
         sol.objective, sol.gap, slack.minCoeff());
}

TEST(BarrierQP, FeasibilityCheck) {
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

  Model problem;
  problem.AddLinearConstraint(A, b, Sense::LE);
  problem.AddQuadraticCost(Q);
  problem.SetLinearCost(c);

  VectorXd x0 = VectorXd::Zero(n);
  auto sol = SolveBarrierFromProblem(problem, x0);
  EXPECT_LT(sol.gap, 1e-6);
  VectorXd slack = b - A * sol.x;
  EXPECT_GE(slack.minCoeff(), -1e-6);
  printf("QP feasibility: obj=%.6f, gap=%.2e, slack_min=%.2e\n",
         sol.objective, sol.gap, slack.minCoeff());
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

  Model problem;
  auto c1 = problem.AddLinearConstraint(A, b, vars);

  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  VectorXd rhs_dense = Eigen::MatrixXd(A).transpose() *
                        (Eigen::MatrixXd(A) * VectorXd::Random(n));
  auto rhs = solver.kkt()->MakeBlockVariable(rhs_dense);
  auto x = solver.kkt()->MakeBlockVariable();
  solver.kkt()->SolveInto(rhs, x);

  VectorXd x_dense = solver.kkt()->Solve(rhs_dense);
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

  Model problem;
  problem.AddLinearConstraint(A, b, vars);
  problem.AddQuadraticCost(Q, vars);

  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(n);
  VectorXd x_sol = solver.kkt()->Solve(rhs);

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

  Model problem;
  problem.AddLinearConstraint(A, b, vars);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  ASSERT_TRUE(kkt->AssembleAndFactor());
  VectorXd rhs = VectorXd::Random(n);
  VectorXd x1 = kkt->Solve(rhs);

  MatrixXd M1 = A.transpose() * A;
  VectorXd x1_ref = M1.ldlt().solve(rhs);
  EXPECT_LT((x1 - x1_ref).norm() / x1_ref.norm(), 1e-10);

  // Apply non-uniform weights via the generic interface.
  // Use MultiplyA to discover the internal row ordering, then assign
  // weights that depend on row content (self-consistent with ordering).
  auto x_rhs = kkt->MakeSolverRHS();
  x_rhs = kkt->MakeBlockVariable(VectorXd::Ones(n));
  auto row = kkt->MakeRowSpace();
  kkt->MultiplyA(x_rhs, row);
  // weights_i = 1 + |row_i| — non-uniform, but ordering-consistent.
  RowSpace weights = kkt->MakeRowSpace();
  for (int i = 0; i < weights.total_rows(); ++i)
    weights.col()(i) = 1.0 + std::abs(row.col()(i));
  kkt->SetWeights(weights);
  ASSERT_TRUE(kkt->AssembleAndFactor());
  VectorXd x2 = kkt->Solve(rhs);

  // Verify via round-trip: A^T W A x2 should equal rhs.
  auto x2_rhs = kkt->MakeSolverRHS();
  x2_rhs = kkt->MakeBlockVariable(x2);
  auto ax2 = kkt->MakeRowSpace();
  kkt->MultiplyA(x2_rhs, ax2);
  // Compute A^T (W * A * x2) via generic interface.
  RowSpace wax2 = kkt->MakeRowSpace();
  for (int i = 0; i < wax2.total_rows(); ++i)
    wax2.col()(i) = weights.col()(i) * ax2.col()(i);
  auto atwa_x2 = kkt->MakeSolverRHS();
  atwa_x2.SetZero();
  kkt->AccumulateAtranspose(wax2, atwa_x2);
  solver.tree_solver()->GatherSeparators(atwa_x2);
  VectorXd atwax2(n);
  atwa_x2.supernodes->GatherInto(atwax2);
  double err = (atwax2 - rhs).norm() / rhs.norm();
  EXPECT_LT(err, 1e-10);

  // Verify solution changed from unweighted.
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

  Model problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);

  auto [reduced, expansion] = Preprocess(problem);
  EXPECT_TRUE(expansion.was_reduced());
  EXPECT_EQ(static_cast<int>(expansion.col_map.size()), 4);

  auto solver = Solver::Build(reduced);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  VectorXd rhs_full = VectorXd::Random(n);
  VectorXd rhs_reduced = expansion.Reduce(rhs_full);
  VectorXd x_reduced = solver.kkt()->Solve(rhs_reduced);
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

  Model problem;
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
  ASSERT_TRUE(solver_custom.kkt()->AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(4);
  VectorXd x_custom = solver_custom.kkt()->Solve(rhs);

  auto solver_auto = Solver::Build(problem);
  ASSERT_TRUE(solver_auto.kkt()->AssembleAndFactor());
  // Auto solver preprocesses; reduce RHS and expand solution to compare.
  VectorXd rhs_r = solver_auto.ReduceVector(rhs);
  VectorXd x_auto_r = solver_auto.kkt()->Solve(rhs_r);
  VectorXd x_auto = solver_auto.ExpandSolution(x_auto_r);

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

  Model problem;
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
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  const auto& ic_duals = solver.dual_variables(c_ic);
  VectorXd rhs = VectorXd::Zero(solver.kkt()->number_of_variables());
  for (int i = 0; i < nx; ++i) rhs(ic_duals[i]) = x0(i);

  auto rhs_bv = solver.kkt()->MakeBlockVariable(rhs);
  auto x_bv = solver.kkt()->MakeBlockVariable();
  solver.kkt()->SolveInto(rhs_bv, x_bv);
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

  Model problem;
  problem.AddLinearConstraint(
      Eigen::SparseMatrix<double>(A.sparseView()),
      VectorXd::Zero(m), primal_vars);
  auto c_eq = problem.AddEqualityConstraint(
      Eigen::SparseMatrix<double>(C.sparseView()),
      d, primal_vars);

  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  const auto& dual_vars = solver.dual_variables(c_eq);
  ASSERT_EQ(static_cast<int>(dual_vars.size()), p);

  int n_total = solver.kkt()->number_of_variables();
  VectorXd rhs = VectorXd::Zero(n_total);
  rhs.head(n) = A.transpose() * b;
  for (int i = 0; i < p; ++i) rhs(dual_vars[i]) = d(i);

  auto rhs_bv = solver.kkt()->MakeBlockVariable(rhs);
  auto x_bv = solver.kkt()->MakeBlockVariable();
  solver.kkt()->SolveInto(rhs_bv, x_bv);
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

  Model problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  problem.AddQuadraticCost(Q, vars);

  auto solver = Solver::BuildDense(problem);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(n);
  VectorXd x_sol = solver.kkt()->Solve(rhs);

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

  Model problem;
  problem.AddLinearConstraint(
      Eigen::SparseMatrix<double>(A.sparseView()),
      VectorXd::Zero(m), primal_vars);
  auto c_eq = problem.AddEqualityConstraint(
      Eigen::SparseMatrix<double>(C.sparseView()),
      d, primal_vars);

  auto solver = Solver::BuildDense(problem);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  const auto& dual_vars = solver.dual_variables(c_eq);
  int n_total = solver.kkt()->number_of_variables();
  VectorXd rhs = VectorXd::Zero(n_total);
  rhs.head(n) = A.transpose() * b;
  for (int i = 0; i < p; ++i) rhs(dual_vars[i]) = d(i);

  // Solve via BlockVariable.
  auto rhs_bv = solver.kkt()->MakeBlockVariable(rhs);
  auto x_bv = solver.kkt()->MakeBlockVariable();
  solver.kkt()->SolveInto(rhs_bv, x_bv);
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

TEST(ProblemSolver, EqualityConstraintVectorOps) {
  // Verify KKT conditions for equality-constrained least squares using the
  // generic SolverRHS interface:
  //   min 0.5 ||Ax - b||^2  s.t. Cx = d
  // KKT: A^T A x + C^T lambda = A^T b,  Cx = d
  srand(42);
  const int m = 15, n = 6, p = 2;
  MatrixXd A = MatrixXd::Random(m, n);
  VectorXd b = VectorXd::Random(m);
  MatrixXd C = MatrixXd::Random(p, n);
  VectorXd d_eq = VectorXd::Random(p);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddLinearConstraint(
      Eigen::SparseMatrix<double>(A.sparseView()),
      b, vars);
  problem.AddEqualityConstraint(
      Eigen::SparseMatrix<double>(C.sparseView()),
      d_eq, vars);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  auto* ts = solver.tree_solver();
  ASSERT_NE(ts, nullptr);
  ASSERT_TRUE(kkt->AssembleAndFactor());

  // Solve the KKT system.
  int n_total = kkt->number_of_variables();
  VectorXd rhs_dense = VectorXd::Zero(n_total);
  // A^T b for primal, d for dual.
  rhs_dense.head(n) = A.transpose() * b;
  // Set dual RHS via dual variable indices.
  const auto& eq_assemblers = ts->equality_sub_assemblers();
  ASSERT_FALSE(eq_assemblers.empty());
  for (const auto* ec : eq_assemblers) {
    const auto& dv = ec->dual_variables();
    const auto& d_local = ec->affine_term();
    for (int i = 0; i < static_cast<int>(dv.size()); ++i)
      rhs_dense(dv[i]) = d_local(i);
  }

  VectorXd sol = kkt->Solve(rhs_dense);
  VectorXd x_sol = sol.head(n);

  // --- Verify using generic interface ---
  // 1. Feasibility: Cx = d.  Check each equality sub-assembler.
  for (const auto* ec : eq_assemblers) {
    VectorXd Cx = ec->constraint_matrix() * x_sol;
    double eq_err = (Cx - ec->affine_term()).norm();
    EXPECT_LT(eq_err, 1e-10) << "Equality constraint violated";
  }

  // 2. Optimality: A^T(Ax) + C^T lambda = A^T b.
  //    All terms computed via generic vector ops on SolverRHS.
  auto sol_rhs = kkt->MakeSolverRHS();
  sol_rhs.supernodes->ScatterFrom(sol);
  sol_rhs.blocks_fully_gathered = true;

  // A * x (via generic interface)
  RowSpace ax = kkt->MakeRowSpace();
  kkt->MultiplyA(sol_rhs, ax);

  // Accumulate A^T(Ax) into grad.
  auto grad = kkt->MakeSolverRHS();
  grad.SetZero();
  kkt->AccumulateAtranspose(ax, grad);

  // Accumulate C^T lambda (via AccumulateCtranspose on tree solver).
  // sol_rhs contains both x and lambda; the saddle-point product
  // [0 C'; C 0] * [x; lambda] = [C^T lambda; Cx] is accumulated.
  ts->AccumulateCtranspose(sol_rhs, grad);
  ts->GatherSeparators(grad);

  VectorXd grad_dense(n_total);
  grad.supernodes->GatherInto(grad_dense);

  // Full KKT residual (primal block): A^T(Ax) + C^T lambda - A^T b = 0.
  VectorXd kkt_residual = grad_dense.head(n) - A.transpose() * b;
  double opt_err = kkt_residual.norm();
  EXPECT_LT(opt_err, 1e-10) << "KKT optimality residual nonzero";

  // 3. Dense reference.
  MatrixXd K = MatrixXd::Zero(n + p, n + p);
  K.topLeftCorner(n, n) = A.transpose() * A;
  K.topRightCorner(n, p) = C.transpose();
  K.bottomLeftCorner(p, n) = C;
  VectorXd kkt_rhs(n + p);
  kkt_rhs.head(n) = A.transpose() * b;
  kkt_rhs.tail(p) = d_eq;
  VectorXd kkt_ref = K.fullPivLu().solve(kkt_rhs);
  double sol_err = (x_sol - kkt_ref.head(n)).norm() / kkt_ref.head(n).norm();
  EXPECT_LT(sol_err, 1e-10);
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

  Model problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  problem.AddEqualityConstraint(C, d, vars);

  // Call Preprocess explicitly so we can inspect the reduced equality.
  auto [reduced, expansion] = Preprocess(problem);
  // Solver::Build will preprocess again (idempotent on already-reduced input).
  auto solver = Solver::Build(reduced);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  // Build RHS in reduced space.
  VectorXd rhs = VectorXd::Zero(solver.kkt()->number_of_variables());
  VectorXd rhs_primal = expansion.Reduce(
      Eigen::MatrixXd(A).transpose() * b);
  rhs.head(rhs_primal.size()) = rhs_primal;

  // Set equality RHS via dual variables.
  // The equality constraint is the second constraint (id=1).
  const auto& duals = solver.dual_variables(1);
  // d was reduced by Preprocess (dropped rows).
  // The reduced equality constraint has fewer rows.
  auto* eq = std::get_if<Model::EqualityConstraintData>(
      &reduced.constraint(1));
  ASSERT_TRUE(eq != nullptr);
  for (int i = 0; i < static_cast<int>(duals.size()); ++i)
    rhs(duals[i]) = eq->d(i);

  VectorXd sol = solver.kkt()->Solve(rhs);
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

  Model problem;
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

  Model problem;
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

  Model problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  VectorXd x_sol = solver.kkt()->Solve(rhs);
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

  Model problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  VectorXd x_sol = solver.kkt()->Solve(rhs);
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

  Model problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);
  problem.AddQuadraticCost(Q, vars);

  SolverConfiguration cfg;
  cfg.use_quotient_amd = true;
  auto solver_q = Solver::Build(problem, cfg);
  ASSERT_TRUE(solver_q.kkt()->AssembleAndFactor());

  cfg.use_quotient_amd = false;
  auto solver_v = Solver::Build(problem, cfg);
  ASSERT_TRUE(solver_v.kkt()->AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(n);
  VectorXd x_q = solver_q.kkt()->Solve(rhs);
  VectorXd x_v = solver_v.kkt()->Solve(rhs);

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

  Model problem;
  for (int g = 0; g < num_groups; ++g) {
    MatrixXd A_block = MatrixXd::Random(rows_per, bw);
    std::vector<int> block_vars(bw);
    std::iota(block_vars.begin(), block_vars.end(), g);
    problem.AddLinearConstraint(A_block, VectorXd::Zero(rows_per), block_vars);
  }

  SolverConfiguration cfg;
  cfg.use_quotient_amd = true;
  auto solver = Solver::Build(problem, cfg);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  // Build reference via dense.
  auto solver_d = Solver::BuildDense(problem);
  ASSERT_TRUE(solver_d.kkt()->AssembleAndFactor());

  VectorXd rhs = VectorXd::Random(n);
  VectorXd x_q = solver.kkt()->Solve(rhs);
  VectorXd x_d = solver_d.kkt()->Solve(rhs);

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
    Model problem;
    problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);

    SolverConfiguration cfg;
    cfg.num_threads = threads;
    auto solver = Solver::Build(problem, cfg);
    ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

    VectorXd x_sol = solver.kkt()->Solve(rhs);
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
    Model problem;
    problem.AddLinearConstraint(A, VectorXd::Zero(m), vars);

    SolverConfiguration cfg;
    cfg.tree.supernode_reorder_method = method;
    auto solver = Solver::Build(problem, cfg);
    ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

    VectorXd x_sol = solver.kkt()->Solve(rhs);
    if (method == 0) {
      x_ref = x_sol;
    }
    double err = (x_sol - x_true).norm() / x_true.norm();
    EXPECT_LT(err, 1e-8) << "Failed with reorder method " << names[method];
  }
  printf("ProblemSolver.PQTreeReorder: all 4 methods match (err<1e-8)\n");
}

TEST(ProblemSolver, AggressiveCliqueMerging) {
  srand(55);
  const int n = 30, bw = 5, rpg = 6;
  int ng = n - bw + 1, nr = rpg * ng;
  std::vector<Eigen::Triplet<double>> trips;
  for (int g = 0; g < ng; ++g)
    for (int r = 0; r < rpg; ++r)
      for (int j = 0; j < bw; ++j)
        trips.emplace_back(g * rpg + r, g + j,
                           0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = MatrixXd(A).transpose() * (MatrixXd(A) * x_true);

  // Solve with aggressive merging.
  for (int merge_size : {0, 5, 10, 20}) {
    Model problem;
    problem.AddLinearConstraint(A, VectorXd::Zero(nr), vars);
    SolverConfiguration cfg;
    cfg.tree.max_merge_supernode_size = merge_size;
    auto solver = Solver::Build(problem, cfg);
    ASSERT_TRUE(solver.kkt()->AssembleAndFactor());
    VectorXd sol = solver.kkt()->Solve(rhs);
    double err = (sol - x_true).norm() / x_true.norm();
    EXPECT_LT(err, 1e-8) << "merge_size=" << merge_size;
  }
  printf("ProblemSolver.AggressiveCliqueMerging: all merge sizes pass\n");
}

TEST(ProblemSolver, LeftLookingVsRightLooking) {
  srand(42);
  const int n = 40, bw = 8, rpg = 5;
  int ng = n - bw + 1, nr = rpg * ng;
  std::vector<Eigen::Triplet<double>> trips;
  for (int g = 0; g < ng; ++g)
    for (int r = 0; r < rpg; ++r)
      for (int j = 0; j < bw; ++j)
        trips.emplace_back(g * rpg + r, g + j,
                           0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = MatrixXd(A).transpose() * (MatrixXd(A) * x_true);

  for (bool left_looking : {true, false}) {
    Model problem;
    problem.AddLinearConstraint(A, VectorXd::Zero(nr), vars);
    SolverConfiguration cfg;
    cfg.tree.left_looking = left_looking;
    auto solver = Solver::Build(problem, cfg);
    ASSERT_TRUE(solver.kkt()->AssembleAndFactor());
    VectorXd sol = solver.kkt()->Solve(rhs);
    double err = (sol - x_true).norm() / x_true.norm();
    EXPECT_LT(err, 1e-8) << (left_looking ? "left" : "right") << "-looking";
  }
  printf("ProblemSolver.LeftLookingVsRightLooking: both pass\n");
}

TEST(ProblemSolver, GenericFactorization) {
  srand(42);
  const int n = 30, bw = 6, rpg = 5;
  int ng = n - bw + 1, nr = rpg * ng;
  std::vector<Eigen::Triplet<double>> trips;
  for (int g = 0; g < ng; ++g)
    for (int r = 0; r < rpg; ++r)
      for (int j = 0; j < bw; ++j)
        trips.emplace_back(g * rpg + r, g + j,
                           0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = MatrixXd(A).transpose() * (MatrixXd(A) * x_true);

  for (bool generic : {false, true}) {
    Model problem;
    problem.AddLinearConstraint(A, VectorXd::Zero(nr), vars);
    SolverConfiguration cfg;
    cfg.tree.use_generic_factorization = generic;
    auto solver = Solver::Build(problem, cfg);
    ASSERT_TRUE(solver.kkt()->AssembleAndFactor());
    VectorXd sol = solver.kkt()->Solve(rhs);
    double err = (sol - x_true).norm() / x_true.norm();
    EXPECT_LT(err, 1e-8) << (generic ? "generic" : "llt");
  }
  printf("ProblemSolver.GenericFactorization: both modes pass\n");
}

TEST(ProblemSolver, MultiColumnSolve) {
  srand(44);
  const int n = 12, m = 32;
  MatrixXd A = MatrixXd::Random(m, n);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddLinearConstraint(
      Eigen::SparseMatrix<double>(A.sparseView()), VectorXd::Zero(m), vars);
  SolverConfiguration cfg;
  cfg.rhs_cols = 3;
  auto solver = Solver::Build(problem, cfg);
  auto* kkt = solver.kkt();
  ASSERT_TRUE(kkt->AssembleAndFactor());

  MatrixXd ATA = A.transpose() * A;

  for (int ncols : {1, 2, 3}) {
    MatrixXd X_true = MatrixXd::Random(n, ncols);
    MatrixXd rhs = ATA * X_true;
    MatrixXd sol = kkt->Solve(rhs);
    double err = (sol - X_true).norm() / X_true.norm();
    EXPECT_LT(err, 1e-8) << "ncols=" << ncols;
  }
  printf("ProblemSolver.MultiColumnSolve: 1,2,3 columns pass\n");
}

TEST(ProblemSolver, RepeatedAssembleAndFactor) {
  srand(55);
  const int n = 30, bw = 6, rpg = 5;
  int ng = n - bw + 1, nr = rpg * ng;
  std::vector<Eigen::Triplet<double>> trips;
  for (int g = 0; g < ng; ++g)
    for (int r = 0; r < rpg; ++r)
      for (int j = 0; j < bw; ++j)
        trips.emplace_back(g * rpg + r, g + j,
                           0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = MatrixXd(A).transpose() * (MatrixXd(A) * x_true);

  Model problem;
  problem.AddLinearConstraint(A, VectorXd::Zero(nr), vars);
  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  // Factor and solve multiple times — should produce identical results.
  for (int iter = 0; iter < 3; ++iter) {
    ASSERT_TRUE(kkt->AssembleAndFactor());
    VectorXd sol = kkt->Solve(rhs);
    double err = (sol - x_true).norm() / x_true.norm();
    EXPECT_LT(err, 1e-8) << "iter=" << iter;
  }
  printf("ProblemSolver.RepeatedAssembleAndFactor: 3 iterations match\n");
}

TEST(ProblemSolver, LUForIndefinite) {
  srand(42);
  const int n = 20, bw = 5, rpg = 4;
  int ng = n - bw + 1, nr = rpg * ng;
  std::vector<Eigen::Triplet<double>> trips;
  for (int g = 0; g < ng; ++g)
    for (int r = 0; r < rpg; ++r)
      for (int j = 0; j < bw; ++j)
        trips.emplace_back(g * rpg + r, g + j,
                           0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = MatrixXd(A).transpose() * (MatrixXd(A) * x_true);

  // PD system: use_lu_for_indefinite shouldn't affect PD cliques.
  for (bool lu : {false, true}) {
    Model problem;
    problem.AddLinearConstraint(A, VectorXd::Zero(nr), vars);
    SolverConfiguration cfg;
    cfg.tree.use_generic_factorization = true;
    cfg.tree.use_lu_for_indefinite = lu;
    auto solver = Solver::Build(problem, cfg);
    ASSERT_TRUE(solver.kkt()->AssembleAndFactor());
    VectorXd sol = solver.kkt()->Solve(rhs);
    double err = (sol - x_true).norm() / x_true.norm();
    EXPECT_LT(err, 1e-8) << (lu ? "lu" : "rldlt");
  }
  printf("ProblemSolver.LUForIndefinite: PD system unaffected by LU flag\n");
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
    Model prob_c;
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
    solver_c.kkt()->AssembleAndFactor();
    const auto& ic_duals = solver_c.dual_variables(c_ic);
    VectorXd rhs_c = VectorXd::Zero(solver_c.kkt()->number_of_variables());
    for (int j = 0; j < nx; ++j) rhs_c(ic_duals[j]) = x0(j);
    VectorXd sol_c = solver_c.kkt()->Solve(rhs_c);
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

    Model prob_a;
    prob_a.AddQuadraticCost(Q_cost, all_vars);
    auto ceq_a = prob_a.AddEqualityConstraint(C_eq, d_eq, all_vars);

    auto ta0 = clock::now();
    auto solver_a = Solver::Build(prob_a);
    solver_a.kkt()->AssembleAndFactor();
    const auto& duals_a = solver_a.dual_variables(ceq_a);
    VectorXd rhs_a = VectorXd::Zero(solver_a.kkt()->number_of_variables());
    for (int j = 0; j < n_eq; ++j) rhs_a(duals_a[j]) = d_eq(j);
    VectorXd sol_a = solver_a.kkt()->Solve(rhs_a);
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

    // Build Model (shared by both paths).
    auto build_problem = [&]() {
      Model problem;
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
    Model prob_custom;
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
    solver_c.kkt()->AssembleAndFactor();
    auto tc2 = clock::now();
    const auto& ic_duals = solver_c.dual_variables(c_ic);
    VectorXd rhs_c = VectorXd::Zero(solver_c.kkt()->number_of_variables());
    for (int i = 0; i < nx; ++i) rhs_c(ic_duals[i]) = x0(i);
    VectorXd sol_c = solver_c.kkt()->Solve(rhs_c);
    auto tc3 = clock::now();
    double custom_us = std::chrono::duration<double, std::micro>(tc3 - tc0).count();

    // --- Automatic path ---
    auto [prob_a, ceq_a] = build_problem();
    auto ta0 = clock::now();
    auto solver_a = Solver::Build(prob_a);
    auto ta1 = clock::now();
    solver_a.kkt()->AssembleAndFactor();
    auto ta2 = clock::now();
    const auto& duals_a = solver_a.dual_variables(ceq_a);
    VectorXd rhs_a = VectorXd::Zero(solver_a.kkt()->number_of_variables());
    int n_eq = (T+1)*nx;
    VectorXd d_eq = VectorXd::Zero(n_eq);
    d_eq.tail(nx) = x0;
    for (int i = 0; i < n_eq; ++i) rhs_a(duals_a[i]) = d_eq(i);
    VectorXd sol_a = solver_a.kkt()->Solve(rhs_a);
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

  printf("\n  Gaussian MRF (d=%d): Model + Solver + TreeSpec\n", d);
  printf("%-6s %-6s %6s %7s  %8s %8s %8s  %10s\n",
         "B", "D", "nodes", "n_vars", "build", "factor", "solve", "residual");
  printf("------  ------ ------ -------  -------- -------- --------"
         "  ----------\n");

  for (auto [B, D] : std::vector<std::pair<int,int>>{{2,5},{3,4},{2,8},{3,5},{2,10}}) {
    auto graph = MakeBalancedTree(B, D);
    int N = graph.num_nodes;
    int n_vars = N * d;

    auto t0 = clock::now();

    Model problem;
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
    ASSERT_TRUE(solver.kkt()->AssembleAndFactor());
    auto t2 = clock::now();

    VectorXd h = VectorXd::Random(n_vars);
    auto rhs_bv = solver.kkt()->MakeBlockVariable(h);
    auto x_bv = solver.kkt()->MakeBlockVariable();
    solver.kkt()->SolveInto(rhs_bv, x_bv);
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

// Test the generic KKTSolverBase interface (MultiplyA, AccumulateAtranspose,
// AccumulateQx, SetWeights, SolveSolverRHS) with multiple linear constraints
// and multiple quadratic costs.  Compares against a dense reference.
TEST(ProblemSolver, DISABLED_MultipleConstraintsGenericInterface) {
  srand(123);
  const int n = 8;

  // Two linear constraints with overlapping variables.
  MatrixXd A1 = MatrixXd::Random(5, n);
  MatrixXd A2 = MatrixXd::Random(4, n);
  VectorXd b1 = VectorXd::Zero(5);
  VectorXd b2 = VectorXd::Zero(4);

  // Two quadratic costs.
  MatrixXd Q1_half = MatrixXd::Random(3, n);
  MatrixXd Q1 = Q1_half.transpose() * Q1_half;  // PSD
  MatrixXd Q2 = MatrixXd::Identity(n, n) * 0.5;

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // Build problem with multiple constraints/costs.
  Model problem;
  problem.AddLinearConstraint(Eigen::SparseMatrix<double>(A1.sparseView()),
                               b1, vars);
  problem.AddLinearConstraint(Eigen::SparseMatrix<double>(A2.sparseView()),
                               b2, vars);
  problem.AddQuadraticCost(Eigen::SparseMatrix<double>(Q1.sparseView()), vars);
  problem.AddQuadraticCost(Eigen::SparseMatrix<double>(Q2.sparseView()), vars);

  // Dense reference: M = Q1 + Q2 + A1'A1 + A2'A2
  MatrixXd M_ref = Q1 + Q2 + A1.transpose() * A1 + A2.transpose() * A2;
  VectorXd rhs_ref = VectorXd::Random(n);
  VectorXd x_ref = M_ref.ldlt().solve(rhs_ref);

  // --- Test via Consolidate + Build ---
  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  ASSERT_TRUE(kkt->AssembleAndFactor());
  VectorXd x_sol = kkt->Solve(rhs_ref);
  double err_solve = (x_sol - x_ref).norm() / x_ref.norm();
  EXPECT_LT(err_solve, 1e-10);

  // --- Test generic interface: MultiplyA ---
  auto x_rhs = kkt->MakeSolverRHS();
  x_rhs = kkt->MakeBlockVariable(x_ref);
  auto row = kkt->MakeRowSpace();
  kkt->MultiplyA(x_rhs, row);

  // Dense reference: A_stacked * x
  VectorXd Ax_ref(9);
  Ax_ref.head(5) = A1 * x_ref;
  Ax_ref.tail(4) = A2 * x_ref;
  double err_multiply_a = (row.col() - Ax_ref).norm() / Ax_ref.norm();
  EXPECT_LT(err_multiply_a, 1e-10);

  // --- Test generic interface: AccumulateAtranspose ---
  VectorXd v = VectorXd::Random(9);
  RowSpace v_row = kkt->MakeRowSpace();
  v_row.col() = v;
  auto atv_rhs = kkt->MakeSolverRHS();
  atv_rhs.SetZero();
  kkt->AccumulateAtranspose(v_row, atv_rhs);
  solver.tree_solver()->GatherSeparators(atv_rhs);
  VectorXd atv_sol(n);
  atv_rhs.supernodes->GatherInto(atv_sol);

  // Dense reference: A1' * v1 + A2' * v2
  VectorXd atv_ref = A1.transpose() * v.head(5) + A2.transpose() * v.tail(4);
  double err_at = (atv_sol - atv_ref).norm() / atv_ref.norm();
  EXPECT_LT(err_at, 1e-10);

  // --- Test multi-column MultiplyA ---
  {
    VectorXd x2 = VectorXd::Random(n);
    auto x_mc = kkt->MakeSolverRHS(2);
    // Pack two columns: col0 = x_ref, col1 = x2.
    auto tmp0 = kkt->MakeSolverRHS();
    tmp0 = kkt->MakeBlockVariable(x_ref);
    auto tmp1 = kkt->MakeSolverRHS();
    tmp1 = kkt->MakeBlockVariable(x2);
    // Gather into dense, build 2-col, scatter back.
    Eigen::MatrixXd dense_x(n, 2);
    tmp0.supernodes->GatherInto(dense_x.col(0));
    tmp1.supernodes->GatherInto(dense_x.col(1));
    x_mc.supernodes->ScatterFrom(dense_x);
    x_mc.blocks_fully_gathered = true;

    auto row_mc = kkt->MakeRowSpace(2);
    kkt->MultiplyA(x_mc, row_mc);

    // Dense reference.
    Eigen::MatrixXd A_stacked(9, n);
    A_stacked.topRows(5) = A1;
    A_stacked.bottomRows(4) = A2;
    Eigen::MatrixXd Ax_mc_ref = A_stacked * dense_x;
    double err_mc_a = (row_mc.segment(0) - Ax_mc_ref).norm() / Ax_mc_ref.norm();
    EXPECT_LT(err_mc_a, 1e-10);

    // --- Test multi-column AccumulateAtranspose ---
    auto atv_mc = kkt->MakeSolverRHS(2);
    atv_mc.SetZero();
    kkt->AccumulateAtranspose(row_mc, atv_mc);
    solver.tree_solver()->GatherSeparators(atv_mc);
    Eigen::MatrixXd atv_mc_sol(n, 2);
    atv_mc.supernodes->GatherInto(atv_mc_sol);
    Eigen::MatrixXd atv_mc_ref = A_stacked.transpose() * Ax_mc_ref;
    double err_mc_at = (atv_mc_sol - atv_mc_ref).norm() / atv_mc_ref.norm();
    EXPECT_LT(err_mc_at, 1e-10);

    printf("  multi-col: A*x=%.2e, A'v=%.2e\n", err_mc_a, err_mc_at);
  }

  // --- Test generic interface: AccumulateQx ---
  auto qx_rhs = kkt->MakeSolverRHS();
  qx_rhs.SetZero();
  kkt->AccumulateQx(x_rhs, qx_rhs);
  solver.tree_solver()->GatherSeparators(qx_rhs);
  VectorXd qx_sol(n);
  qx_rhs.supernodes->GatherInto(qx_sol);

  // Dense reference: (Q1 + Q2) * x
  VectorXd qx_ref = (Q1 + Q2) * x_ref;
  double err_qx = (qx_sol - qx_ref).norm() / qx_ref.norm();
  EXPECT_LT(err_qx, 1e-10);

  // --- Test generic interface: SetWeights + re-solve ---
  RowSpace weights = kkt->MakeRowSpace();
  weights.col().head(5).setConstant(2.0);
  weights.col().tail(4).setConstant(3.0);
  kkt->SetWeights(weights);
  ASSERT_TRUE(kkt->AssembleAndFactor());

  MatrixXd M_w = Q1 + Q2 +
      2.0 * A1.transpose() * A1 + 3.0 * A2.transpose() * A2;
  VectorXd x_w_ref = M_w.ldlt().solve(rhs_ref);
  VectorXd x_w_sol = kkt->Solve(rhs_ref);
  double err_w = (x_w_sol - x_w_ref).norm() / x_w_ref.norm();
  EXPECT_LT(err_w, 1e-10);

  printf("MultipleConstraints (consolidated): solve=%.2e, A*x=%.2e, A'v=%.2e, "
         "Q*x=%.2e, weighted=%.2e\n",
         err_solve, err_multiply_a, err_at, err_qx, err_w);

  // --- Test WITHOUT Consolidate ---
  // Build with multiple assemblers registered directly on the tree solver.
  // This exercises the multi-assembler loop in MultiplyA/AccumulateAtranspose/
  // AccumulateQx.
  {
    // Build the tree solver via ConstraintManager (no Consolidate).
    ConstraintManager cm(n);

    auto slc1 = std::make_unique<SparseLinearConstraint>(
        Eigen::SparseMatrix<double>(A1.sparseView()), b1);
    auto asm1 = std::make_unique<SparseLinearConstraintAssembler>(
        std::move(slc1), vars);
    auto* asm1_ptr = asm1.get();
    cm.AddCustomAssembler(std::move(asm1));

    auto slc2 = std::make_unique<SparseLinearConstraint>(
        Eigen::SparseMatrix<double>(A2.sparseView()), b2);
    auto asm2 = std::make_unique<SparseLinearConstraintAssembler>(
        std::move(slc2), vars);
    auto* asm2_ptr = asm2.get();
    cm.AddCustomAssembler(std::move(asm2));

    Eigen::SparseMatrix<double> Q1s = Q1.sparseView();
    auto qasm1 = std::make_unique<SparseQuadraticTermAssembler>(Q1s, vars);
    auto* qasm1_ptr = qasm1.get();
    cm.AddCustomAssembler(std::move(qasm1));

    Eigen::SparseMatrix<double> Q2s = Q2.sparseView();
    auto qasm2 = std::make_unique<SparseQuadraticTermAssembler>(Q2s, vars);
    auto* qasm2_ptr = qasm2.get();
    cm.AddCustomAssembler(std::move(qasm2));

    SolverConfiguration config;
    auto ts = MakeTreeSolver(&cm, config);

    // Register decomposed sub-assemblers (not top-level assemblers).
    for (const auto& lc : asm1_ptr->constraints())
      ts->RegisterLinearSubAssembler(lc.get());
    for (const auto& lc : asm2_ptr->constraints())
      ts->RegisterLinearSubAssembler(lc.get());
    for (auto& qc : qasm1_ptr->constraints())
      ts->RegisterQuadraticSubAssembler(&qc);
    for (auto& qc : qasm2_ptr->constraints())
      ts->RegisterQuadraticSubAssembler(&qc);

    ASSERT_TRUE(ts->AssembleAndFactor());

    // Verify solve (dense path).
    VectorXd x_sol2 = ts->Solve(rhs_ref);
    double err_s2 = (x_sol2 - x_ref).norm() / x_ref.norm();
    EXPECT_LT(err_s2, 1e-10);

    // Test round-trip: A^T(A*x) should equal (A1^T A1 + A2^T A2) * x.
    auto x_rhs2 = ts->MakeSolverRHS();
    x_rhs2 = ts->MakeBlockVariable(x_ref);
    auto row2 = ts->MakeRowSpace();
    ts->MultiplyA(x_rhs2, row2);

    auto atax_rhs = ts->MakeSolverRHS();
    atax_rhs.SetZero();
    ts->AccumulateAtranspose(row2, atax_rhs);
    ts->GatherSeparators(atax_rhs);
    VectorXd atax_sol(n);
    atax_rhs.supernodes->GatherInto(atax_sol);
    VectorXd atax_ref = (A1.transpose() * A1 + A2.transpose() * A2) * x_ref;
    double err_ata = (atax_sol - atax_ref).norm() / atax_ref.norm();
    EXPECT_LT(err_ata, 1e-10);

    // Test AccumulateQx.
    auto qx_rhs2 = ts->MakeSolverRHS();
    qx_rhs2.SetZero();
    ts->AccumulateQx(x_rhs2, qx_rhs2);
    ts->GatherSeparators(qx_rhs2);
    VectorXd qx_sol2(n);
    qx_rhs2.supernodes->GatherInto(qx_sol2);
    double err_qx2 = (qx_sol2 - qx_ref).norm() / qx_ref.norm();
    EXPECT_LT(err_qx2, 1e-10);

    // Test GetAffineTerm.
    auto b_row = ts->GetAffineTerm();
    // Each sub-constraint has its own affine term — total rows should
    // equal total rows of A1 + A2.
    EXPECT_EQ(b_row.total_rows(), 9);

    printf("MultipleConstraints (sub-assemblers): solve=%.2e, "
           "A'Ax=%.2e, Qx=%.2e\n", err_s2, err_ata, err_qx2);
  }
}

TEST(CliqueTree, RunningIntersectionProperty) {
  // Build a valid clique tree from a banded pattern and verify RIP holds.
  const int n = 20;
  std::vector<std::vector<int>> supports;
  for (int i = 0; i < n - 2; ++i)
    supports.push_back({i, i + 1, i + 2});

  auto tree = MakeCliqueTreeMinDegreeFromRowSupports(supports);
  EXPECT_TRUE(tree.CheckRunningIntersectionProperty());

  // Violates check 2: variable 0 is a supernode of two cliques.
  CliqueTree bad_dup;
  bad_dup.supernodes = {{0, 1}, {2, 3}, {0, 3}};
  bad_dup.separators = {{}, {}, {}};
  bad_dup.node_to_parent = {-1, 0, 1};
  EXPECT_FALSE(bad_dup.CheckRunningIntersectionProperty());

  // Violates check 1: separator {5} of clique 1 is not in parent clique 0.
  CliqueTree bad_sep;
  bad_sep.supernodes = {{0, 1}, {2, 3}};
  bad_sep.separators = {{}, {5}};
  bad_sep.node_to_parent = {-1, 0};
  EXPECT_FALSE(bad_sep.CheckRunningIntersectionProperty());
}

TEST(CliqueTree, MinDegreeOrderingQuality) {
  // Verify that MakeCliqueTreeMinDegreeFromRowSupports produces a
  // good-quality ordering by checking fill-in count and max clique size.
  // These are regression values — if an optimization changes tie-breaking,
  // fill-in may change but should not get worse.

  // Banded pattern: n=50, bandwidth=3. Optimal ordering is near-trivial.
  {
    const int n = 50;
    std::vector<std::vector<int>> supports;
    for (int i = 0; i < n - 2; ++i)
      supports.push_back({i, i + 1, i + 2});

    std::vector<std::vector<int>> cliques;
    auto tree = MakeCliqueTreeMinDegreeFromRowSupports(
        supports, &cliques, /*max_merge=*/0, SUPERNODE_REORDER_NONE);

    EXPECT_TRUE(tree.CheckRunningIntersectionProperty());

    // Compute total fill-in: sum of |later[v]| over all v gives
    // the number of edges in the filled graph.  For a banded(3) matrix,
    // optimal fill-in is 0 (no new edges).  Check via clique sizes:
    // each clique should be at most 3 variables.
    int max_clique = 0;
    int total_clique_storage = 0;
    for (const auto& c : cliques) {
      max_clique = std::max(max_clique, static_cast<int>(c.size()));
      total_clique_storage += static_cast<int>(c.size());
    }
    EXPECT_LE(max_clique, 3) << "Banded(3) should have max clique size 3";
    // Total storage should be roughly 3*num_cliques.
    EXPECT_LE(total_clique_storage, 3 * static_cast<int>(cliques.size()) + 5);
  }

  // Arrow pattern: one dense column plus diagonal.  Min-degree should
  // eliminate the diagonal first, leaving the dense column for last.
  {
    const int n = 30;
    std::vector<std::vector<int>> supports;
    // Each row connects variable i to variable 0.
    for (int i = 1; i < n; ++i)
      supports.push_back({0, i});

    std::vector<std::vector<int>> cliques;
    auto tree = MakeCliqueTreeMinDegreeFromRowSupports(
        supports, &cliques, /*max_merge=*/0, SUPERNODE_REORDER_NONE);

    EXPECT_TRUE(tree.CheckRunningIntersectionProperty());

    // Optimal: eliminate leaves first (degree 1), no fill-in.
    // Variable 0 is eliminated last.  Result should be a single clique
    // of size n (the star graph is already a clique after eliminating leaves).
    // But min-degree eliminates leaves first, so only the root node creates
    // a large clique.
    int max_clique = 0;
    for (const auto& c : cliques)
      max_clique = std::max(max_clique, static_cast<int>(c.size()));
    // Star graph: eliminating degree-1 leaves first creates no fill-in.
    // Each leaf forms a 2-element clique {0, i}.  No merge (max_merge=0)
    // so cliques stay small.
    EXPECT_LE(max_clique, n);
    // Total fill-in should be 0 (star graph has perfect elimination).
    // Check total storage is at most 2*(n-1) + 1 (leaf cliques + root).
    int total = 0;
    for (const auto& c : cliques) total += static_cast<int>(c.size());
    EXPECT_LE(total, 2 * n);
  }

  // Grid-like pattern: 10x5 grid connectivity.  Check fill-in doesn't
  // regress from known good value.
  {
    const int rows = 10, cols = 5;
    const int n = rows * cols;
    std::vector<std::vector<int>> supports;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        int v = r * cols + c;
        if (c + 1 < cols) supports.push_back({v, v + 1});
        if (r + 1 < rows) supports.push_back({v, v + cols});
      }
    }

    std::vector<std::vector<int>> cliques;
    auto tree = MakeCliqueTreeMinDegreeFromRowSupports(
        supports, &cliques, /*max_merge=*/5, SUPERNODE_REORDER_NONE);

    EXPECT_TRUE(tree.CheckRunningIntersectionProperty());

    // Compute total clique storage as a quality metric.
    int total_storage = 0;
    int max_clique = 0;
    for (const auto& c : cliques) {
      total_storage += static_cast<int>(c.size());
      max_clique = std::max(max_clique, static_cast<int>(c.size()));
    }

    // For a 10x5 grid, nested dissection gives max clique ~cols.
    // Min-degree should be comparable.  Record current values as baseline.
    printf("CliqueTree.MinDegreeOrderingQuality grid 10x5: "
           "%d cliques, max_size=%d, total_storage=%d\n",
           static_cast<int>(cliques.size()), max_clique, total_storage);
    EXPECT_LE(max_clique, 16) << "Grid max clique too large (regression)";
    EXPECT_LE(total_storage, 80) << "Grid total storage too large (regression)";
  }
}

TEST(ProblemSolver, LinearConstraintNonIdentityVars) {
  // Regression: AddLinearConstraint with non-identity variable mapping.
  srand(42);
  Eigen::MatrixXd A_dense(3, 2);
  A_dense << 1, 2, 3, 4, 5, 6;
  Eigen::SparseMatrix<double> A = A_dense.sparseView();
  VectorXd b = VectorXd::Zero(3);
  std::vector<int> vars = {5, 10};

  int n = 11;
  Model problem;
  for (int i = 0; i < n; ++i) {
    Eigen::MatrixXd Qi(1,1); Qi << 0.01;
    problem.AddQuadraticCost(Qi, {i});
  }
  problem.AddLinearConstraint(A, b, vars);

  auto solver = Solver::Build(problem);
  ASSERT_TRUE(solver.kkt()->AssembleAndFactor());

  VectorXd b_test = VectorXd::Ones(3);
  VectorXd rhs = VectorXd::Zero(solver.kkt()->number_of_variables());
  VectorXd Atb = A_dense.transpose() * b_test;
  rhs(5) = Atb(0);
  rhs(10) = Atb(1);
  VectorXd sol = solver.kkt()->Solve(rhs);

  Eigen::MatrixXd M = A_dense.transpose() * A_dense + 0.01 * Eigen::MatrixXd::Identity(2, 2);
  VectorXd x_ref = M.ldlt().solve(Atb);
  double err = std::abs(sol(5) - x_ref(0)) + std::abs(sol(10) - x_ref(1));
  printf("  LinearConstraintNonIdentityVars: err=%.2e\n", err);
  EXPECT_LT(err, 1e-8);
}

}  // namespace
}  // namespace conex

#if 0  // SolverSolve tests moved to solver_solve_test.cc.
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
  EXPECT_GE(s.minCoeff(), -1e-3);

  // Dual non-negativity: lambda >= 0.
  printf("  min_lambda = %.2e\n", lam.minCoeff());
  EXPECT_GE(lam.minCoeff(), -1e-3);

  // Complementarity: lambda . s ≈ 0.
  double cs = lam.dot(s);
  printf("  complementarity = %.2e\n", cs);
  EXPECT_LT(cs, 1e-2);

  // Dual feasibility: A' lambda ≈ c.
  VectorXd dual_res = A_dense.transpose() * lam - c;
  printf("  dual_residual = %.2e\n", dual_res.norm());
  EXPECT_LT(dual_res.norm(), 1e-2);
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
  auto result = solver.Solve(conex::PhaseOneHybrid());

  printf("  mu = %.2e\n", result.mu);
  ASSERT_EQ(result.duals.lambda.size(), 1u);
  ASSERT_EQ(result.duals.slack.size(), 1u);

  const auto& lam = result.duals.lambda[0];
  const auto& s = result.duals.slack[0];

  // Primal feasibility.
  printf("  min_slack = %.2e\n", s.minCoeff());
  EXPECT_GE(s.minCoeff(), -1e-3);

  // Dual non-negativity.
  printf("  min_lambda = %.2e\n", lam.minCoeff());
  EXPECT_GE(lam.minCoeff(), -1e-3);

  // Complementarity.
  double cs = lam.dot(s);
  printf("  complementarity = %.2e\n", cs);
  EXPECT_LT(cs, 1e-2);

  // Stationarity: Qx + c = A' lambda.
  VectorXd grad = Q * result.x + c_cost;
  VectorXd dual_res = grad - A_dense.transpose() * lam;
  printf("  stationarity_residual = %.2e\n", dual_res.norm());
  EXPECT_LT(dual_res.norm(), 1e-2);
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
  EXPECT_GE(result.duals.slack[0].minCoeff(), -1e-3);
  EXPECT_GE(result.duals.slack[1].minCoeff(), -1e-3);

  // Verify slacks match A*x + b directly.
  VectorXd s1_check = A1d * result.x + b1;
  VectorXd s2_check = A2d * result.x + b2;
  EXPECT_LT((result.duals.slack[0] - s1_check).norm(), 1e-6);
  EXPECT_LT((result.duals.slack[1] - s2_check).norm(), 1e-6);

  // --- Dual non-negativity ---
  EXPECT_GE(result.duals.lambda[0].minCoeff(), -1e-3);
  EXPECT_GE(result.duals.lambda[1].minCoeff(), -1e-3);

  // --- Per-constraint complementarity ---
  double cs0 = result.duals.lambda[0].dot(result.duals.slack[0]);
  double cs1 = result.duals.lambda[1].dot(result.duals.slack[1]);
  printf("  complementarity: %.2e, %.2e\n", cs0, cs1);
  EXPECT_LT(cs0, 1e-2);
  EXPECT_LT(cs1, 1e-2);

  // --- Stationarity: c = A1' lambda[0] + A2' lambda[1] ---
  VectorXd dual_res = c - A1d.transpose() * result.duals.lambda[0]
                         - A2d.transpose() * result.duals.lambda[1];
  printf("  stationarity_residual = %.2e\n", dual_res.norm());
  EXPECT_LT(dual_res.norm(), 1e-2);
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
  EXPECT_GE(result.duals.slack[0].minCoeff(), -1e-3);

  // Equality: x0+x1 ≈ 0.5, x2+x3 ≈ 0.5.
  double eq1_err = std::abs(result.x(0) + result.x(1) - 0.5);
  double eq2_err = std::abs(result.x(2) + result.x(3) - 0.5);
  printf("  equality errors: %.2e, %.2e\n", eq1_err, eq2_err);
  EXPECT_LT(eq1_err, 1e-2);
  EXPECT_LT(eq2_err, 1e-2);

  // --- Complementarity ---
  double cs = result.duals.lambda[0].dot(result.duals.slack[0]);
  printf("  complementarity = %.2e\n", cs);
  EXPECT_LT(cs, 1e-2);

  // --- Stationarity: c = A' lambda + C1' nu1 + C2' nu2 ---
  VectorXd At_lam = Ad.transpose() * result.duals.lambda[0];
  VectorXd Ct_nu = Eigen::MatrixXd(C1).transpose() * result.duals.nu[0] +
                   Eigen::MatrixXd(C2).transpose() * result.duals.nu[1];
  VectorXd stat_res = c - At_lam - Ct_nu;
  printf("  stationarity_residual = %.2e\n", stat_res.norm());
  EXPECT_LT(stat_res.norm(), 1e-2);
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
  EXPECT_GE(min_eig_s, -1e-3);

  // --- Dual feasibility: Λ ≽ 0 ---
  Eigen::SelfAdjointEigenSolver<MatrixXd> eig_l(Lambda);
  double min_eig_l = eig_l.eigenvalues().minCoeff();
  printf("  min_eig(Lambda) = %.2e\n", min_eig_l);
  EXPECT_GE(min_eig_l, -1e-3);

  // --- Complementarity: tr(S · Λ) ≈ 0 ---
  double cs = (S * Lambda).trace();
  printf("  tr(S*Lambda) = %.2e\n", cs);
  EXPECT_LT(std::abs(cs), 1e-2);

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
  EXPECT_LT(stat_res.norm(), 1e-2);
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
  EXPECT_GE(s0 + 1e-3, s1_norm);

  // Dual SOC membership: λ₀ >= ||λ₁||.
  double l0 = lam(0);
  double l1_norm = lam.tail(vec_dim).norm();
  printf("  lam0 = %.4e, ||lam1|| = %.4e\n", l0, l1_norm);
  EXPECT_GE(l0 + 1e-3, l1_norm);

  // Complementarity: λ · s ≈ 0.
  double cs = lam.dot(s);
  printf("  complementarity = %.2e\n", cs);
  EXPECT_LT(std::abs(cs), 1e-2);

  // Stationarity: c = A' λ.
  VectorXd stat_res = c - A_dense.transpose() * lam;
  printf("  stationarity_residual = %.2e\n", stat_res.norm());
  EXPECT_LT(stat_res.norm(), 1e-2);
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
  conex::ThetaContinuation algo;
  algo.verbose = false;
  auto result = solver.Solve(algo);
  //auto result = solver.Solve(conex::ThetaContinuation());

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
      EXPECT_GE(result.duals.slack[idx].minCoeff(), -1e-3);
      EXPECT_GE(result.duals.lambda[idx].minCoeff(), -1e-3);
    }
  }

  // --- SOC membership ---
  for (int idx : {1, 3}) {
    const auto& s = result.duals.slack[idx];
    const auto& l = result.duals.lambda[idx];
    EXPECT_GE(s(0) + 1e-3, s.tail(s.size() - 1).norm());
    EXPECT_GE(l(0) + 1e-3, l.tail(l.size() - 1).norm());
  }

  // --- PSD feasibility ---
  for (int p = 0; p < 2; ++p) {
    Eigen::SelfAdjointEigenSolver<MatrixXd> eig_s(result.duals.psd_slack[p]);
    Eigen::SelfAdjointEigenSolver<MatrixXd> eig_l(result.duals.psd_lambda[p]);
    printf("  PSD[%d]: min_eig(S)=%.2e, min_eig(L)=%.2e\n", p,
           eig_s.eigenvalues().minCoeff(), eig_l.eigenvalues().minCoeff());
    EXPECT_GE(eig_s.eigenvalues().minCoeff(), -1e-3);
    EXPECT_GE(eig_l.eigenvalues().minCoeff(), -1e-3);
  }

  // --- Equality ---
  double eq1_err = std::abs(result.x(0) + result.x(1) - 0.5);
  double eq2_err = std::abs(result.x(8) + result.x(9) - 0.3);
  printf("  equality errors: %.2e, %.2e\n", eq1_err, eq2_err);
  EXPECT_LT(eq1_err, 1e-2);
  EXPECT_LT(eq2_err, 1e-2);

  // --- Per-constraint complementarity ---
  double total_cs = 0;
  for (int idx = 0; idx < 4; ++idx)
    total_cs += std::abs(result.duals.lambda[idx].dot(result.duals.slack[idx]));
  for (int p = 0; p < 2; ++p)
    total_cs += std::abs((result.duals.psd_slack[p] * result.duals.psd_lambda[p]).trace());
  printf("  total complementarity = %.2e\n", total_cs);
  EXPECT_LT(total_cs, 1e-1);

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
  EXPECT_LT(stat_res.norm(), 1e-1);
}
#endif
