#include "conex/algorithms/barrier_qp.h"
#include "conex/algorithms/equality_constrained_least_squares.h"
#include "conex/algorithms/finite_horizon.h"
#include "conex/algorithms/irls.h"
#include "conex/algorithms/lqr_tree_solver.h"
#include "conex/algorithms/tree_solver_builder.h"
#include "conex/common/clique_ordering.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/sparse_equality_constraint.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/tree_solver/kkt_solver_factory.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <numeric>
#include <set>

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  MatrixXd Ad(A);
  VectorXd b = Ad * x_true;
  // Add outliers.
  for (int i = 0; i < m / 10; i++)
    b(rand() % m) += 10.0 * ((double)rand() / RAND_MAX - 0.5);

  auto result = SolveIRLS(A, b, 50, 1e-6, 1e-8);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 0.5) << "IRLS L1 error: " << err;
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
// Barrier QP tests: min 0.5 x^T Q x + c^T x  s.t.  A x <= b
// =====================================================================

// Simple 2D QP with known solution.
// min 0.5 (x1^2 + x2^2) s.t. x1 + x2 <= 1, x1 >= 0, x2 >= 0.
// Written as: A x <= b with A = [1 1; -1 0; 0 -1], b = [1; 0; 0].
// Unconstrained min at (0,0) which is feasible. Solution: (0,0).
TEST(BarrierQP, UnconstrainedInsideFeasible) {
  const int n = 2, m = 3;
  std::vector<Eigen::Triplet<double>> qt;
  qt.emplace_back(0, 0, 1.0);
  qt.emplace_back(1, 1, 1.0);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());

  VectorXd c = VectorXd::Zero(n);

  std::vector<Eigen::Triplet<double>> at;
  at.emplace_back(0, 0, 1.0); at.emplace_back(0, 1, 1.0);   // x1+x2 <= 1
  at.emplace_back(1, 0, -1.0);                                 // -x1 <= 0
  at.emplace_back(2, 1, -1.0);                                 // -x2 <= 0
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(at.begin(), at.end());
  VectorXd b(m); b << 1.0, 0.0, 0.0;

  // Strictly feasible start.
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

// Active constraint: min x1  s.t.  x1+x2<=1, x1>=0, x2>=0.
// Q=0, c=[1,0]. Optimal at (0, anything in [0,1]). min obj = 0.
// Add small Q for strict convexity: Q = 0.001*I.
TEST(BarrierQP, ActiveConstraint) {
  const int n = 2, m = 3;
  std::vector<Eigen::Triplet<double>> qt;
  qt.emplace_back(0, 0, 0.001);
  qt.emplace_back(1, 1, 0.001);
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

  // x1 should be near 0 (active lower bound).
  EXPECT_LT(result.x(0), 0.05) << "x1 should be near lower bound";
  EXPECT_GE(result.x(0), -0.01) << "x1 should be feasible";

  printf("QP active: obj=%.6f, x=[%.4f, %.4f], gap=%.2e, "
         "%d outer, %d newton, %.0fus\n",
         result.objective, result.x(0), result.x(1), result.duality_gap,
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

// Larger sparse QP: banded Q, random inequality constraints.
TEST(BarrierQP, SparseQP) {
  srand(42);
  const int n = 20, m = 15;

  // Banded Q.
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

  // Random sparse A (each row touches 3 variables).
  std::vector<Eigen::Triplet<double>> at;
  for (int r = 0; r < m; r++)
    for (int j = 0; j < 3; j++)
      at.emplace_back(r, rand() % n, (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(at.begin(), at.end());

  // b chosen so x=0 is strictly feasible.
  VectorXd b = VectorXd::Ones(m);

  VectorXd x0 = VectorXd::Zero(n);

  auto result = SolveBarrierQP(Q, c, A, b, x0, 30, 50, 10.0, 1e-6);

  // Check feasibility: A x <= b.
  VectorXd slack = b - A * result.x;
  EXPECT_GE(slack.minCoeff(), -1e-6) << "Solution not feasible";

  // Check optimality: gradient should be in normal cone.
  // For interior points, grad = Q x + c should be small.
  // For boundary points, grad has component along active constraint normals.
  VectorXd grad = Q * result.x + c;

  printf("QP sparse: obj=%.6f, gap=%.2e, slack_min=%.2e, grad_norm=%.2e, "
         "%d outer, %d newton, %.0fus\n",
         result.objective, result.duality_gap, slack.minCoeff(), grad.norm(),
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

// Verify solver reuse: the tree solver is built once and reused across
// all Newton steps (SetWeights changes only numerical values).
TEST(BarrierQP, SolverReuse) {
  const int n = 10, m = 5;
  std::vector<Eigen::Triplet<double>> qt;
  for (int i = 0; i < n; i++) qt.emplace_back(i, i, 2.0);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());

  VectorXd c = VectorXd::Random(n);

  std::vector<Eigen::Triplet<double>> at;
  for (int r = 0; r < m; r++)
    at.emplace_back(r, r, 1.0);  // x_i <= b_i for first m vars
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(at.begin(), at.end());
  VectorXd b = VectorXd::Ones(m) * 5.0;

  VectorXd x0 = VectorXd::Zero(n);

  auto result = SolveBarrierQP(Q, c, A, b, x0, 20, 30, 10.0, 1e-8);

  // Should converge in multiple Newton steps (using solver reuse).
  EXPECT_GT(result.total_newton_steps, 1);
  EXPECT_LT(result.duality_gap, 1e-6);

  // Verify feasibility.
  VectorXd slack = b - A * result.x;
  EXPECT_GE(slack.minCoeff(), -1e-6);

  printf("QP reuse: obj=%.6f, gap=%.2e, %d outer, %d newton, %.0fus\n",
         result.objective, result.duality_gap,
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

// =====================================================================
// Equality-constrained least squares tests:
//   min ||Ax - b||^2  subject to  Cx = d
// =====================================================================

// Helper: build sparse matrix from triplets.
Eigen::SparseMatrix<double> MakeSparse(
    int rows, int cols,
    const std::vector<Eigen::Triplet<double>>& trips) {
  Eigen::SparseMatrix<double> M(rows, cols);
  M.setFromTriplets(trips.begin(), trips.end());
  return M;
}

// Small dense problem with known solution.
// A = [1 0; 0 1; 1 1], b = [1; 2; 0], C = [1 1], d = [3].
TEST(EqualityConstrainedLS, SmallDense) {
  const int m = 3, n = 2, p = 1;
  Eigen::SparseMatrix<double> A = MakeSparse(m, n, {
      {0, 0, 1.0}, {1, 1, 1.0}, {2, 0, 1.0}, {2, 1, 1.0}});

  VectorXd b(m);
  b << 1, 2, 0;

  Eigen::SparseMatrix<double> C = MakeSparse(p, n, {{0, 0, 1.0}, {0, 1, 1.0}});
  VectorXd d(p);
  d << 3;

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-10) << "Equality constraint violated";

  // Verify against closed-form KKT solution.
  MatrixXd Cd(C);
  MatrixXd Ad(A);
  MatrixXd AtA = Ad.transpose() * Ad;
  VectorXd Atb = Ad.transpose() * b;
  MatrixXd KKT = MatrixXd::Zero(n + p, n + p);
  KKT.topLeftCorner(n, n) = AtA;
  KKT.topRightCorner(n, p) = Cd.transpose();
  KKT.bottomLeftCorner(p, n) = Cd;
  VectorXd rhs(n + p);
  rhs << Atb, d;
  VectorXd x_expected = KKT.lu().solve(rhs).head(n);

  double err = (result.x - x_expected).norm();
  EXPECT_LT(err, 1e-10) << "Solution does not match KKT reference";

  printf("ECLS small: x=[%.4f, %.4f], constraint_err=%.2e, "
         "construct=%.0fus, factor=%.0fus, solve=%.0fus\n",
         result.x(0), result.x(1), constraint_err,
         result.construction_time_us, result.assemble_and_factor_time_us,
         result.solve_time_us);
}

// Larger sparse problem: banded A, random equality constraints.
TEST(EqualityConstrainedLS, SparseBanded) {
  srand(42);
  const int n = 50, m = 80, p = 5;

  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r) {
    int col_start = (r * n) / m;
    for (int j = 0; j < 3 && col_start + j < n; ++j)
      trips.emplace_back(r, col_start + j, (double)rand() / RAND_MAX + 0.1);
  }
  Eigen::SparseMatrix<double> A = MakeSparse(m, n, trips);

  // Random sparse C: each row touches 4 variables.
  std::vector<Eigen::Triplet<double>> ct;
  for (int r = 0; r < p; ++r)
    for (int j = 0; j < 4; ++j)
      ct.emplace_back(r, rand() % n, (double)rand() / RAND_MAX - 0.5);
  Eigen::SparseMatrix<double> C = MakeSparse(p, n, ct);

  VectorXd x_true = VectorXd::Random(n);
  VectorXd d = C * x_true;
  VectorXd b = MatrixXd(A) * x_true + 0.01 * VectorXd::Random(m);

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-8) << "Equality constraint violated";

  // Verify KKT optimality: projected gradient onto null(C) is zero.
  VectorXd grad = A.transpose() * (MatrixXd(A) * result.x - b);
  MatrixXd Cd(C);
  MatrixXd CCt = Cd * Cd.transpose();
  MatrixXd P = MatrixXd::Identity(n, n) - Cd.transpose() * CCt.lu().solve(Cd);
  double projected_grad_norm = (P * grad).norm();
  EXPECT_LT(projected_grad_norm, 1e-8)
      << "Projected gradient not zero at solution";

  printf("ECLS sparse: n=%d, m=%d, p=%d, constraint_err=%.2e, "
         "proj_grad=%.2e, construct=%.0fus, factor=%.0fus, solve=%.0fus\n",
         n, m, p, constraint_err, projected_grad_norm,
         result.construction_time_us, result.assemble_and_factor_time_us,
         result.solve_time_us);
}

// When the unconstrained solution already satisfies the constraints,
// the constrained and unconstrained solutions should match.
TEST(EqualityConstrainedLS, MatchesUnconstrainedWhenFeasible) {
  srand(123);
  const int n = 10, m = 20, p = 2;

  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r)
    for (int c = 0; c < n; ++c)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX - 0.5);
  Eigen::SparseMatrix<double> A = MakeSparse(m, n, trips);

  VectorXd x_true = VectorXd::Random(n);
  VectorXd b = MatrixXd(A) * x_true;

  // Dense C converted to sparse — constraints satisfied by x_true.
  MatrixXd Cd = MatrixXd::Random(p, n);
  Eigen::SparseMatrix<double> C = Cd.sparseView();
  VectorXd d = C * x_true;

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-8);

  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-10);

  printf("ECLS feasible: err=%.2e, constraint_err=%.2e\n", err, constraint_err);
}

// Benchmark: larger problem for timing.
TEST(EqualityConstrainedLS, Benchmark) {
  srand(42);
  const int n = 500, m = 1000, p = 20;
  const int bandwidth = 5;

  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r) {
    int col_start = (r * n) / m;
    for (int j = 0; j < bandwidth && col_start + j < n; ++j)
      trips.emplace_back(r, col_start + j, (double)rand() / RAND_MAX + 0.1);
  }
  Eigen::SparseMatrix<double> A = MakeSparse(m, n, trips);

  std::vector<Eigen::Triplet<double>> ct;
  for (int r = 0; r < p; ++r) {
    int start = (r * n) / p;
    for (int j = 0; j < 5 && start + j < n; ++j)
      ct.emplace_back(r, start + j, (double)rand() / RAND_MAX - 0.5);
  }
  Eigen::SparseMatrix<double> C = MakeSparse(p, n, ct);

  VectorXd x_true = VectorXd::Random(n);
  VectorXd d = C * x_true;
  VectorXd b = MatrixXd(A) * x_true + 0.01 * VectorXd::Random(m);

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-6);

  double total_us = result.construction_time_us +
                    result.assemble_and_factor_time_us +
                    result.solve_time_us;
  printf("ECLS benchmark: n=%d, m=%d, p=%d, constraint_err=%.2e, "
         "total=%.0fus (construct=%.0fus, factor=%.0fus, solve=%.0fus)\n",
         n, m, p, constraint_err, total_us,
         result.construction_time_us, result.assemble_and_factor_time_us,
         result.solve_time_us);
}

// Structurally rank-deficient equality constraints: redundant rows
// should be removed by Preprocess without affecting the solution.
// C has 4 rows but only 2 structurally independent ones.
TEST(EqualityConstrainedLS, RankDeficientEqualities) {
  const int m = 20, n = 5, p_independent = 2;

  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r)
    for (int c = 0; c < n; ++c)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX + 0.1);
  Eigen::SparseMatrix<double> A = MakeSparse(m, n, trips);

  // Two independent constraints: row 0 touches {0,1}, row 1 touches {2,3}.
  // Two redundant rows: row 2 same support as row 0, row 3 same as row 1.
  Eigen::SparseMatrix<double> C = MakeSparse(4, n, {
      {0, 0, 1.0}, {0, 1, 2.0},
      {1, 2, 3.0}, {1, 3, -1.0},
      {2, 0, 0.5}, {2, 1, -1.0},
      {3, 2, 2.0}, {3, 3, 1.0}});

  VectorXd x_true = VectorXd::Random(n);
  VectorXd d = C * x_true;
  VectorXd b = MatrixXd(A) * x_true;

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-8)
      << "Equality constraints violated after rank reduction";

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-8);

  printf("ECLS rank-deficient: p_orig=4, p_indep=%d, err=%.2e, "
         "constraint_err=%.2e\n",
         p_independent, err, constraint_err);
}

// Larger problem with many redundant equality rows.
TEST(EqualityConstrainedLS, RankDeficientLarger) {
  srand(42);
  const int n = 50, m = 80;
  const int p_indep = 5, p_redundant = 10, p_total = p_indep + p_redundant;

  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r) {
    int col_start = (r * n) / m;
    for (int j = 0; j < 3 && col_start + j < n; ++j)
      trips.emplace_back(r, col_start + j, (double)rand() / RAND_MAX + 0.1);
  }
  Eigen::SparseMatrix<double> A = MakeSparse(m, n, trips);

  // p_indep independent rows, each touching a unique pair of vars.
  // p_redundant rows that duplicate the support of existing rows.
  std::vector<Eigen::Triplet<double>> ct;
  for (int r = 0; r < p_indep; ++r) {
    int base = r * 2;
    ct.emplace_back(r, base, (double)rand() / RAND_MAX + 0.5);
    ct.emplace_back(r, base + 1, (double)rand() / RAND_MAX + 0.5);
  }
  for (int r = 0; r < p_redundant; ++r) {
    int src = r % p_indep;
    int base = src * 2;
    ct.emplace_back(p_indep + r, base, (double)rand() / RAND_MAX + 0.5);
    ct.emplace_back(p_indep + r, base + 1, (double)rand() / RAND_MAX + 0.5);
  }
  Eigen::SparseMatrix<double> C = MakeSparse(p_total, n, ct);

  VectorXd x_true = VectorXd::Random(n);
  VectorXd d = C * x_true;
  VectorXd b = MatrixXd(A) * x_true + 0.01 * VectorXd::Random(m);

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-7);

  printf("ECLS rank-deficient larger: p_total=%d, p_indep=%d, "
         "constraint_err=%.2e, construct=%.0fus, factor=%.0fus\n",
         p_total, p_indep, constraint_err,
         result.construction_time_us, result.assemble_and_factor_time_us);
}

// Inconsistent redundant equalities should be detected and rejected.
// Three rows all touching only {x0}: structural rank 1, so 2 are dropped.
// Row 2 is numerically inconsistent with row 0.
TEST(EqualityConstrainedLS, InconsistentEqualitiesDetected) {
  const int m = 10, n = 3;

  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r)
    for (int c = 0; c < n; ++c)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX + 0.1);
  Eigen::SparseMatrix<double> A = MakeSparse(m, n, trips);

  VectorXd b = VectorXd::Random(m);

  //   Row 0:  x0 = 3
  //   Row 1: 2x0 = 6   (consistent)
  //   Row 2: 3x0 = 10  (inconsistent: should be 9)
  Eigen::SparseMatrix<double> C = MakeSparse(3, n, {
      {0, 0, 1.0}, {1, 0, 2.0}, {2, 0, 3.0}});
  VectorXd d(3);
  d << 3.0, 6.0, 10.0;

  EXPECT_THROW(EqualityConstrainedLeastSquares(A, b, C, d),
               std::runtime_error);
}

// Consistent redundant rows should NOT throw.
TEST(EqualityConstrainedLS, ConsistentProportionalRows) {
  const int m = 10, n = 3;

  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r)
    for (int c = 0; c < n; ++c)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX + 0.1);
  Eigen::SparseMatrix<double> A = MakeSparse(m, n, trips);

  VectorXd b = VectorXd::Random(m);

  Eigen::SparseMatrix<double> C = MakeSparse(3, n, {
      {0, 0, 1.0}, {1, 0, 2.0}, {2, 0, 3.0}});
  VectorXd d(3);
  d << 3.0, 6.0, 9.0;

  EXPECT_NO_THROW(EqualityConstrainedLeastSquares(A, b, C, d));
}

// Verify that each decomposed equality assembler's dual variables
// are present in the maximal clique the assembler was assigned to.
TEST(EqualityConstrainedLS, DualVarsInMaximalCliques) {
  const int n = 10;

  // C has 3 rows with different supports:
  //   row 0: cols {0, 1}
  //   row 1: cols {2, 3, 4}
  //   row 2: cols {0, 1}  (same support as row 0)
  Eigen::SparseMatrix<double> C = MakeSparse(3, n, {
      {0, 0, 1.0}, {0, 1, 2.0},
      {1, 2, 3.0}, {1, 3, 1.0}, {1, 4, -1.0},
      {2, 0, 0.5}, {2, 1, -1.0}});
  VectorXd d(3);
  d << 1, 2, 3;

  auto sec = std::make_unique<SparseEqualityConstraint>(C, d);

  // Primal variables that C touches.
  std::set<int> pset;
  for (const auto& s : sec->row_supports()) pset.insert(s.begin(), s.end());
  std::vector<int> primal(pset.begin(), pset.end());

  // Dual variables: one per row, starting at n.
  std::vector<int> dual = {n, n + 1, n + 2};

  auto assembler = std::make_unique<SparseEqualityConstraintAssembler>(
      std::move(sec), primal, dual);

  // --- Check 1: get_cliques() includes dual vars ---
  auto cliques = assembler->get_cliques();

  // Support group {0,1} has rows 0,2 → duals {n, n+2}.
  // Support group {2,3,4} has row 1 → dual {n+1}.
  bool found_01 = false, found_234 = false;
  for (const auto& clique : cliques) {
    std::set<int> cs(clique.begin(), clique.end());
    if (cs.count(0) && cs.count(1)) {
      EXPECT_TRUE(cs.count(n)) << "Dual var " << n << " missing from clique";
      EXPECT_TRUE(cs.count(n + 2))
          << "Dual var " << n + 2 << " missing from clique";
      found_01 = true;
    }
    if (cs.count(2) && cs.count(3) && cs.count(4)) {
      EXPECT_TRUE(cs.count(n + 1))
          << "Dual var " << n + 1 << " missing from clique";
      found_234 = true;
    }
  }
  EXPECT_TRUE(found_01) << "No clique for support {0,1}";
  EXPECT_TRUE(found_234) << "No clique for support {2,3,4}";

  // --- Check 2: Build clique tree and verify all dual vars appear ---
  // Also add SLC cliques (diagonal A) to represent primal structure.
  std::vector<std::vector<int>> all_cliques = cliques;
  for (int i = 0; i < n; ++i) all_cliques.push_back({i});

  std::vector<std::vector<int>> maximal_cliques;
  auto tree = MakeCliqueTreeMinDegreeFromRowSupports(
      all_cliques, &maximal_cliques, 0, 0, dual);

  // Every dual variable must appear in some maximal clique.
  std::set<int> all_dual(dual.begin(), dual.end());
  std::set<int> found_dual;
  for (const auto& mc : maximal_cliques) {
    for (int v : mc) {
      if (all_dual.count(v)) found_dual.insert(v);
    }
  }
  for (int dv : dual) {
    EXPECT_TRUE(found_dual.count(dv))
        << "Dual variable " << dv
        << " not found in any maximal clique";
  }

  // --- Check 3: Decompose assigns dual vars to per-clique assemblers ---
  auto sec2 = std::make_unique<SparseEqualityConstraint>(C, d);
  auto asm2 = std::make_unique<SparseEqualityConstraintAssembler>(
      std::move(sec2), primal, dual);

  auto decomposed = asm2->Decompose(maximal_cliques);

  // Each decomposed assembler's {primal ∪ dual} must be a subset of
  // some maximal clique.
  for (auto* sub : decomposed) {
    auto vars = sub->variables();  // primal + dual
    std::set<int> vset(vars.begin(), vars.end());

    bool contained = false;
    for (const auto& mc : maximal_cliques) {
      std::set<int> mset(mc.begin(), mc.end());
      if (std::includes(mset.begin(), mset.end(),
                        vset.begin(), vset.end())) {
        contained = true;
        break;
      }
    }
    EXPECT_TRUE(contained)
        << "Decomposed assembler variables not contained in any maximal clique";
  }

  printf("DualVarsInMaximalCliques: %d/%d dual vars in clique tree, "
         "%d decomposed assemblers all contained\n",
         static_cast<int>(found_dual.size()),
         static_cast<int>(dual.size()),
         static_cast<int>(decomposed.size()));
}

// =====================================================================
// Finite-horizon optimal control:
//   min  Σ x_t'Q x_t + u_t'R u_t  +  x_T'Qf x_T
//   s.t. x_{t+1} = A x_t + B u_t,  x_0 = x_init
// =====================================================================

namespace {

// Variable layout helper: z = [x_0, u_0, x_1, u_1, ..., u_{T-1}, x_T]
int XIdx(int t, int nx, int nu) { return t * (nx + nu); }
int UIdx(int t, int nx, int nu) { return t * (nx + nu) + nx; }

// Return the timestep that primal variable v belongs to.
int VarTimestep(int v, int nx, int nu, int T) {
  int block = nx + nu;
  if (v >= T * block) return T;  // terminal x_T
  return v / block;
}

}  // namespace

// Benchmark: sweep horizon lengths, call the algorithm, print timing table.
TEST(FiniteHorizon, Benchmark) {
  const int nx = 4, nu = 2;
  Eigen::MatrixXd A = 0.9 * MatrixXd::Identity(nx, nx) +
                       0.1 * MatrixXd::Random(nx, nx);
  Eigen::MatrixXd B = MatrixXd::Random(nx, nu);
  Eigen::MatrixXd Q = MatrixXd::Identity(nx, nx) +
                       0.5 * MatrixXd::Ones(nx, nx);
  Eigen::MatrixXd R = 0.1 * MatrixXd::Identity(nu, nu) +
                       0.05 * MatrixXd::Ones(nu, nu);
  Eigen::MatrixXd Qf = 10.0 * Q;
  VectorXd x0 = VectorXd::Ones(nx);

  printf("\n%-6s %7s %10s %10s %10s %10s %10s\n",
         "T", "n_vars", "build_us", "factor_us", "solve_us",
         "total_us", "dyn_err");
  printf("------  ------- ---------- ---------- ---------- ----------"
         " ----------\n");

  for (int T : {10, 25, 50, 100, 200}) {
    srand(42);
    auto result = SolveLQRFromSparseMatrices(A, B, Q, R, Qf, x0, T);

    // Verify dynamics.
    double dyn_err = 0;
    for (int t = 0; t < T; ++t) {
      VectorXd err = result.x.col(t + 1) - A * result.x.col(t) -
                     B * result.u.col(t);
      dyn_err = std::max(dyn_err, err.norm());
    }
    double ic_err = (result.x.col(0) - x0).norm();
    dyn_err = std::max(dyn_err, ic_err);
    EXPECT_LT(dyn_err, 1e-6) << "Dynamics violated for T=" << T;

    int n_vars = (T + 1) * nx + T * nu;
    double total = result.construction_time_us + result.factor_time_us +
                   result.solve_time_us;
    printf("%-6d %7d %10.0f %10.0f %10.0f %10.0f %10.2e\n",
           T, n_vars, result.construction_time_us, result.factor_time_us,
           result.solve_time_us, total, dyn_err);
  }
}

// Verify that the clique tree has chain structure ordered in time:
// maximal cliques are ordered so that clique i overlaps only with i±1,
// and the elimination order processes earlier timesteps first.
TEST(FiniteHorizon, CliqueTreeTimeOrdering) {
  const int nx = 4, nu = 2, T = 10;
  const int n_vars = (T + 1) * nx + T * nu;
  const int n_eq = (T + 1) * nx;

  srand(42);
  Eigen::MatrixXd A = 0.9 * MatrixXd::Identity(nx, nx) +
                       0.1 * MatrixXd::Random(nx, nx);
  Eigen::MatrixXd B = MatrixXd::Random(nx, nu);

  // Build cost and constraint matrices (same construction as the algorithm).
  Eigen::MatrixXd Qx = MatrixXd::Identity(nx, nx) +
                        0.5 * MatrixXd::Ones(nx, nx);
  Eigen::MatrixXd Ru = 0.1 * MatrixXd::Identity(nu, nu) +
                        0.05 * MatrixXd::Ones(nu, nu);

  std::vector<Eigen::Triplet<double>> qt;
  for (int t = 0; t < T; ++t) {
    int xi = XIdx(t, nx, nu), ui = UIdx(t, nx, nu);
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < nx; ++j)
        qt.emplace_back(xi + i, xi + j, Qx(i, j));
    for (int i = 0; i < nu; ++i)
      for (int j = 0; j < nu; ++j)
        qt.emplace_back(ui + i, ui + j, Ru(i, j));
  }
  int xT = XIdx(T, nx, nu);
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < nx; ++j)
      qt.emplace_back(xT + i, xT + j, 10.0 * Qx(i, j));
  Eigen::SparseMatrix<double> Q_cost(n_vars, n_vars);
  Q_cost.setFromTriplets(qt.begin(), qt.end());

  std::vector<Eigen::Triplet<double>> ct;
  for (int t = 0; t < T; ++t) {
    int rb = t * nx, xi = XIdx(t, nx, nu);
    int ui = UIdx(t, nx, nu), xi1 = XIdx(t + 1, nx, nu);
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nx; ++c)
        if (A(r, c) != 0) ct.emplace_back(rb + r, xi + c, -A(r, c));
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nu; ++c)
        if (B(r, c) != 0) ct.emplace_back(rb + r, ui + c, -B(r, c));
    for (int i = 0; i < nx; ++i) ct.emplace_back(rb + i, xi1 + i, 1.0);
  }
  for (int i = 0; i < nx; ++i) ct.emplace_back(T * nx + i, i, 1.0);
  Eigen::SparseMatrix<double> C_eq(n_eq, n_vars);
  C_eq.setFromTriplets(ct.begin(), ct.end());
  VectorXd d_eq = VectorXd::Zero(n_eq);
  d_eq.tail(nx) = VectorXd::Ones(nx);

  // Build assemblers and collect cliques.
  std::set<int> q_var_set;
  for (int k = 0; k < Q_cost.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q_cost, k); it; ++it) {
      q_var_set.insert(it.row());
      q_var_set.insert(it.col());
    }
  std::vector<int> q_vars(q_var_set.begin(), q_var_set.end());
  SparseQuadraticTermAssembler q_asm(Q_cost, q_vars);

  auto sec = std::make_unique<SparseEqualityConstraint>(C_eq, d_eq);
  std::set<int> eq_set;
  for (auto& s : sec->row_supports()) eq_set.insert(s.begin(), s.end());
  std::vector<int> eq_primal(eq_set.begin(), eq_set.end());
  std::vector<int> dual(n_eq);
  std::iota(dual.begin(), dual.end(), n_vars);
  SparseEqualityConstraintAssembler eq_asm(std::move(sec), eq_primal, dual);

  std::vector<std::vector<int>> all_cliques;
  for (auto& c : q_asm.get_cliques()) all_cliques.push_back(c);
  for (auto& c : eq_asm.get_cliques()) all_cliques.push_back(c);

  std::vector<std::vector<int>> maximal_cliques;
  auto tree = MakeCliqueTreeMinDegreeFromRowSupports(
      all_cliques, &maximal_cliques, 0, 0, dual);

  // --- Check 1: Compute the "time" of each maximal clique as the
  //     minimum timestep of its primal variables. ---
  int num_mc = static_cast<int>(maximal_cliques.size());
  std::vector<int> mc_min_time(num_mc, T + 1);
  std::vector<int> mc_max_time(num_mc, -1);
  for (int i = 0; i < num_mc; ++i) {
    for (int v : maximal_cliques[i]) {
      if (v >= n_vars) continue;  // skip dual
      int ts = VarTimestep(v, nx, nu, T);
      mc_min_time[i] = std::min(mc_min_time[i], ts);
      mc_max_time[i] = std::max(mc_max_time[i], ts);
    }
  }

  // --- Check 2: Each maximal clique spans at most 2 consecutive timesteps
  //     (t and t+1 due to the dynamics constraint linking them). ---
  for (int i = 0; i < num_mc; ++i) {
    if (mc_max_time[i] < 0) continue;  // dual-only clique
    EXPECT_LE(mc_max_time[i] - mc_min_time[i], 1)
        << "Maximal clique " << i << " spans timesteps "
        << mc_min_time[i] << " to " << mc_max_time[i]
        << " (should span at most 2 consecutive)";
  }

  // --- Check 3: Overlap between maximal cliques only if they share
  //     a consecutive timestep boundary. ---
  for (int i = 0; i < num_mc; ++i) {
    for (int j = i + 1; j < num_mc; ++j) {
      // Check primal overlap.
      std::set<int> si, sj;
      for (int v : maximal_cliques[i])
        if (v < n_vars) si.insert(v);
      for (int v : maximal_cliques[j])
        if (v < n_vars) sj.insert(v);
      std::vector<int> overlap;
      std::set_intersection(si.begin(), si.end(), sj.begin(), sj.end(),
                            std::back_inserter(overlap));
      if (overlap.empty()) continue;

      // Overlapping cliques must be at adjacent timesteps.
      int gap = std::max(mc_min_time[i], mc_min_time[j]) -
                std::min(mc_max_time[i], mc_max_time[j]);
      EXPECT_LE(gap, 0)
          << "Non-adjacent cliques " << i << " (t=" << mc_min_time[i]
          << "-" << mc_max_time[i] << ") and " << j
          << " (t=" << mc_min_time[j] << "-" << mc_max_time[j]
          << ") have primal overlap";
    }
  }

  // --- Check 4: Elimination order is time-monotone for supernodes.
  //     For a chain structure, the post-order may go forward (0→T) or
  //     backward (T→0).  Either direction is valid; verify monotonicity. ---
  std::vector<int> sn_times;
  for (int pos = 0; pos < static_cast<int>(tree.post_order_position_to_clique.size()); ++pos) {
    int ci = tree.post_order_position_to_clique[pos];
    int sn_min_time = T + 1;
    for (int v : tree.supernodes[ci]) {
      if (v < n_vars)
        sn_min_time = std::min(sn_min_time, VarTimestep(v, nx, nu, T));
    }
    if (sn_min_time <= T) sn_times.push_back(sn_min_time);
  }
  // Check monotone non-decreasing OR non-increasing.
  bool increasing = true, decreasing = true;
  for (int i = 1; i < static_cast<int>(sn_times.size()); ++i) {
    if (sn_times[i] < sn_times[i - 1]) increasing = false;
    if (sn_times[i] > sn_times[i - 1]) decreasing = false;
  }
  EXPECT_TRUE(increasing || decreasing)
      << "Elimination order is not time-monotone";

  printf("CliqueTreeTimeOrdering: %d maximal cliques, "
         "chain structure verified for T=%d\n",
         num_mc, T);
}

// =====================================================================
// LQR: custom tree vs sparse-matrix (clique ordering) path
// =====================================================================

namespace {

// Build sparse LQR matrices for use with BuildFromSparseMatrices.
// Uses the same variable layout as LQRTreeSolver.
struct SparseLQRMatrices {
  Eigen::SparseMatrix<double> Q_cost;
  Eigen::SparseMatrix<double> C_eq;
  Eigen::VectorXd d_eq;
  std::vector<int> dual_vars;
  int n_primal;
};

SparseLQRMatrices MakeSparseLQR(
    const MatrixXd& A, const MatrixXd& B,
    const MatrixXd& Q, const MatrixXd& R, const MatrixXd& Qf,
    const VectorXd& x0, int T) {
  int nx = A.rows(), nu = B.cols();
  int step = 2 * nx + nu;
  auto XIdx = [&](int t) { return t * step; };
  auto UIdx = [&](int t) { return t * step + nx; };
  auto LIdx = [&](int t) { return t * step + nx + nu; };
  int XTIdx = T * step;
  int LicIdx = T * step + nx;
  int n_primal = T * step + 2 * nx;
  int n_eq = (T + 1) * nx;

  // Cost: block diagonal [Q,0;0,R] per timestep + Qf terminal.
  std::vector<Eigen::Triplet<double>> qt;
  for (int t = 0; t < T; ++t) {
    int xi = XIdx(t), ui = UIdx(t);
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < nx; ++j)
        if (Q(i,j) != 0) qt.emplace_back(xi+i, xi+j, Q(i,j));
    for (int i = 0; i < nu; ++i)
      for (int j = 0; j < nu; ++j)
        if (R(i,j) != 0) qt.emplace_back(ui+i, ui+j, R(i,j));
  }
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < nx; ++j)
      if (Qf(i,j) != 0) qt.emplace_back(XTIdx+i, XTIdx+j, Qf(i,j));

  // Constraints: dynamics + initial condition.
  std::vector<Eigen::Triplet<double>> ct;
  for (int t = 0; t < T; ++t) {
    int rb = t * nx, xi = XIdx(t), ui = UIdx(t), xi1 = XIdx(t+1);
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nx; ++c)
        if (A(r,c) != 0) ct.emplace_back(rb+r, xi+c, -A(r,c));
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nu; ++c)
        if (B(r,c) != 0) ct.emplace_back(rb+r, ui+c, -B(r,c));
    for (int i = 0; i < nx; ++i) ct.emplace_back(rb+i, xi1+i, 1.0);
  }
  for (int i = 0; i < nx; ++i) ct.emplace_back(T*nx+i, 0+i, 1.0);

  SparseLQRMatrices m;
  m.n_primal = n_primal;
  m.Q_cost.resize(n_primal, n_primal);
  m.Q_cost.setFromTriplets(qt.begin(), qt.end());
  m.C_eq.resize(n_eq, n_primal);
  m.C_eq.setFromTriplets(ct.begin(), ct.end());
  m.d_eq = VectorXd::Zero(n_eq);
  m.d_eq.tail(nx) = x0;

  // Dual variable indices (after BuildFromSparseMatrices allocates them,
  // they start at n_primal).
  m.dual_vars.resize(n_eq);
  std::iota(m.dual_vars.begin(), m.dual_vars.end(), n_primal);
  return m;
}

}  // namespace

TEST(LQRTreeSolver, CompareCustomVsSparse) {
  using clock = std::chrono::high_resolution_clock;

  const int nx = 4, nu = 2;
  srand(42);
  MatrixXd A = 0.9 * MatrixXd::Identity(nx, nx) +
               0.1 * MatrixXd::Random(nx, nx);
  MatrixXd B = MatrixXd::Random(nx, nu);
  MatrixXd Q = MatrixXd::Identity(nx, nx) + 0.5 * MatrixXd::Ones(nx, nx);
  MatrixXd R = 0.1 * MatrixXd::Identity(nu, nu) + 0.05 * MatrixXd::Ones(nu, nu);
  MatrixXd Qf = 10.0 * Q;
  VectorXd x0 = VectorXd::Ones(nx);
  int LicIdx_offset = nx;  // offset within LQRTreeSolver layout

  printf("\n  LQR: custom tree vs BuildFromSparseMatrices\n");
  printf("%-6s %22s %22s %8s\n",
         "T", "custom(build+fac+sol)", "sparse(build+fac+sol)", "speedup");
  printf("------  ---------------------- ---------------------- --------\n");

  for (int T : {10, 50, 100, 200}) {
    // Custom tree path.
    auto tc0 = clock::now();
    LQRTreeSolver lqr(A, B, Q, R, Qf, T);
    auto tc1 = clock::now();
    lqr.AssembleAndFactor();
    auto tc2 = clock::now();
    auto sol_custom = lqr.Solve(x0);
    auto tc3 = clock::now();
    double custom_us = std::chrono::duration<double, std::micro>(tc3 - tc0).count();

    // Sparse matrix path.
    auto sm = MakeSparseLQR(A, B, Q, R, Qf, x0, T);
    auto ts0 = clock::now();
    auto result = TreeSolverBuilder::BuildFromSparseMatrices(
        sm.Q_cost, sm.C_eq, sm.d_eq);
    auto ts1 = clock::now();
    result.solver->AssembleAndFactor();
    auto ts2 = clock::now();
    VectorXd rhs = VectorXd::Zero(result.num_variables);
    for (int i = 0; i < static_cast<int>(sm.dual_vars.size()); ++i)
      rhs(sm.dual_vars[i]) = sm.d_eq(i);
    auto sol_sparse = result.solver->Solve(rhs);
    auto ts3 = clock::now();
    double sparse_us = std::chrono::duration<double, std::micro>(ts3 - ts0).count();

    // Verify both solve correctly.
    auto x_c = lqr.ExtractStates(sol_custom);
    EXPECT_LT((x_c.col(0) - x0).norm(), 1e-4)
        << "Custom: initial condition at T=" << T;

    // Verify sparse path solves correctly.
    double ic_err_sparse = (sol_sparse.col(0).head(nx) - x0).norm();
    EXPECT_LT(ic_err_sparse, 1e-4)
        << "Sparse: initial condition at T=" << T;

    printf("%-6d %12.0f          %12.0f          %7.1fx\n",
           T, custom_us, sparse_us, sparse_us / custom_us);
  }
}

// =====================================================================
// TreeSolverBuilder
// =====================================================================

// Small 2-clique chain: min x'Qx s.t. Cx = d.
// Clique 0 (child): sn={x0,x1,lam}, sep={x2}
// Clique 1 (root):  sn={x2}
TEST(TreeSolverBuilder, SmallChain) {
  // min x0^2 + x1^2 + x2^2 s.t. x0 + x1 - x2 = 0, x0 = 1
  TreeSolverBuilder b;
  int root = b.AddClique();       // clique for x2
  int child = b.AddClique(root);  // clique for x0, x1, constraints

  // Cost: Q=I on {x0,x1} in child, Q=I on {x2} in root.
  b.AddCost(child, MatrixXd::Identity(2, 2), {0, 1});
  b.AddCost(root, MatrixXd::Identity(1, 1), {2});

  // Constraint 1: x0 + x1 - x2 = 0 → C=[1,1,-1], primal={0,1,2}, dual={3}
  Eigen::MatrixXd C1(1, 3);
  C1 << 1, 1, -1;
  b.AddEquality(child, C1, VectorXd::Zero(1), {0, 1, 2}, {3});

  // Constraint 2: x0 = 1 → C=[1], primal={0}, dual={4}
  b.AddEquality(child, MatrixXd::Identity(1, 1), VectorXd::Zero(1), {0}, {4});

  auto result = b.Build();
  ASSERT_TRUE(result.solver->AssembleAndFactor());

  // RHS: dual for constraint 2 gets d=1.
  VectorXd rhs = VectorXd::Zero(result.num_variables);
  rhs(4) = 1.0;  // x0 = 1

  VectorXd sol = result.solver->Solve(rhs);

  // x0 = 1 (from constraint), x0 + x1 = x2.
  // Optimal: min x0^2+x1^2+x2^2 s.t. x0=1, x2=x0+x1.
  // Substituting: min 1 + x1^2 + (1+x1)^2 = 2 + 2x1 + 2x1^2.
  // Derivative: 2 + 4x1 = 0 → x1 = -0.5, x2 = 0.5.
  EXPECT_NEAR(sol(0), 1.0, 1e-10);
  EXPECT_NEAR(sol(1), -0.5, 1e-10);
  EXPECT_NEAR(sol(2), 0.5, 1e-10);

  printf("TreeSolverBuilder SmallChain: x=[%.4f, %.4f, %.4f]\n",
         sol(0), sol(1), sol(2));
}

// Rebuild LQR via the builder and verify it matches LQRTreeSolver.
TEST(TreeSolverBuilder, LQRViaBuilder) {
  const int nx = 4, nu = 2, T = 20;
  srand(42);
  Eigen::MatrixXd A = 0.9 * MatrixXd::Identity(nx, nx) +
                       0.1 * MatrixXd::Random(nx, nx);
  Eigen::MatrixXd B = MatrixXd::Random(nx, nu);
  Eigen::MatrixXd Q = MatrixXd::Identity(nx, nx) +
                       0.5 * MatrixXd::Ones(nx, nx);
  Eigen::MatrixXd R = 0.1 * MatrixXd::Identity(nu, nu) +
                       0.05 * MatrixXd::Ones(nu, nu);
  Eigen::MatrixXd Qf = 10.0 * Q;
  VectorXd x0 = VectorXd::Ones(nx);

  // Variable layout: same as LQRTreeSolver.
  const int step = 2 * nx + nu;
  auto XIdx = [&](int t) { return t * step; };
  auto UIdx = [&](int t) { return t * step + nx; };
  auto LIdx = [&](int t) { return t * step + nx + nu; };
  int XTIdx = T * step;
  int LicIdx = T * step + nx;

  // Build via TreeSolverBuilder.
  TreeSolverBuilder builder;

  // Create cliques: chain 0 → 1 → ... → T (root).
  std::vector<int> cid(T + 1);
  cid[T] = builder.AddClique();  // root
  for (int t = T - 1; t >= 0; --t)
    cid[t] = builder.AddClique(cid[t + 1]);

  // Dynamics: [-A, -B, I] with primal={x_t, u_t, x_{t+1}}, dual={λ_t}.
  Eigen::MatrixXd C_dyn(nx, nx + nu + nx);
  C_dyn << -A, -B, MatrixXd::Identity(nx, nx);
  VectorXd d_zero = VectorXd::Zero(nx);

  Eigen::MatrixXd QR = MatrixXd::Zero(nx + nu, nx + nu);
  QR.topLeftCorner(nx, nx) = Q;
  QR.bottomRightCorner(nu, nu) = R;

  for (int t = 0; t < T; ++t) {
    std::vector<int> cost_vars, dyn_primal, dyn_dual;
    for (int i = 0; i < nx; ++i) cost_vars.push_back(XIdx(t) + i);
    for (int i = 0; i < nu; ++i) cost_vars.push_back(UIdx(t) + i);
    builder.AddCost(cid[t], QR, cost_vars);

    dyn_primal = cost_vars;
    for (int i = 0; i < nx; ++i) dyn_primal.push_back(XIdx(t + 1) + i);
    for (int i = 0; i < nx; ++i) dyn_dual.push_back(LIdx(t) + i);
    builder.AddEquality(cid[t], C_dyn, d_zero, dyn_primal, dyn_dual);

    if (t == 0) {
      std::vector<int> ic_primal, ic_dual;
      for (int i = 0; i < nx; ++i) ic_primal.push_back(XIdx(0) + i);
      for (int i = 0; i < nx; ++i) ic_dual.push_back(LicIdx + i);
      builder.AddEquality(cid[0], MatrixXd::Identity(nx, nx), d_zero,
                          ic_primal, ic_dual);
    }
  }

  // Terminal cost.
  std::vector<int> term_vars;
  for (int i = 0; i < nx; ++i) term_vars.push_back(XTIdx + i);
  builder.AddCost(cid[T], Qf, term_vars);

  auto result = builder.Build();
  ASSERT_TRUE(result.solver->AssembleAndFactor());

  VectorXd rhs = VectorXd::Zero(result.num_variables);
  for (int i = 0; i < nx; ++i) rhs(LicIdx + i) = x0(i);
  VectorXd sol = result.solver->Solve(rhs);

  // Compare with LQRTreeSolver.
  LQRTreeSolver lqr(A, B, Q, R, Qf, T);
  lqr.AssembleAndFactor();
  VectorXd sol_ref = lqr.Solve(x0);

  double max_err = (sol.head(sol_ref.size()) - sol_ref).cwiseAbs().maxCoeff();
  EXPECT_LT(max_err, 1e-10) << "Builder solution doesn't match LQRTreeSolver";

  printf("TreeSolverBuilder LQR: T=%d, max_err=%.2e\n", T, max_err);
}

// RIP violation: variable in child and grandparent but missing from parent.
TEST(TreeSolverBuilder, RIPViolationDetected) {
  TreeSolverBuilder b;
  b.EnableRIPCheck();

  int root = b.AddClique();
  int mid = b.AddClique(root);
  int leaf = b.AddClique(mid);

  // Variable 0 in leaf and root, but NOT in mid → RIP violation.
  b.AddCost(leaf, MatrixXd::Identity(2, 2), {0, 1});
  b.AddCost(mid, MatrixXd::Identity(1, 1), {2});  // no var 0
  b.AddCost(root, MatrixXd::Identity(2, 2), {0, 3});

  EXPECT_THROW(b.Build(), std::runtime_error);
}

// =====================================================================
// Network flow QP via TreeSolverBuilder:
//   min  Σ_e c_e * x_e²
//   s.t. Σ_{e∈δ+(v)} x_e - Σ_{e∈δ-(v)} x_e = b_v   ∀ nodes v
//
// Tree graph: 0 - 1 - 2 - 3 - 4  (path graph, 4 edges)
// Edge variables: x_01, x_12, x_23, x_34
// Node dual variables: λ_0, ..., λ_4
// Flow conservation at each node.
// =====================================================================

TEST(TreeSolverBuilder, NetworkFlow) {
  const int num_nodes = 5;
  const int num_edges = num_nodes - 1;

  // Variable layout:
  //   Edge flow: x_e at index e  (0..3)
  //   Node dual: λ_v at index num_edges + v  (4..8)
  auto edge_var = [](int e) { return e; };
  auto node_dual = [&](int v) { return num_edges + v; };

  // Edge costs: c_e * x_e² with c_e = 1 for all edges.
  // Flow conservation: for internal node v,
  //   x_{v-1,v} - x_{v,v+1} = b_v
  // For node 0: -x_{0,1} = b_0  (only outgoing)
  // For node 4:  x_{3,4} = b_4  (only incoming)
  // Here edge e connects node e to node e+1.

  // Supply/demand: source at node 0, sink at node 4.
  VectorXd b = VectorXd::Zero(num_nodes);
  b(0) = -1.0;  // supply (outgoing)
  b(4) = 1.0;   // demand (incoming)

  // Build tree: one clique per edge.
  // Edge e's clique has: edge var x_e, node duals λ_e and λ_{e+1}.
  // Chain: clique 0 → clique 1 → ... → clique (num_edges-1) as root.
  TreeSolverBuilder builder;
  builder.EnableRIPCheck();

  std::vector<int> cid(num_edges);
  cid[num_edges - 1] = builder.AddClique();
  for (int e = num_edges - 2; e >= 0; --e)
    cid[e] = builder.AddClique(cid[e + 1]);

  for (int e = 0; e < num_edges; ++e) {
    // Cost: c_e * x_e² (1x1 block).
    builder.AddCost(cid[e], MatrixXd::Identity(1, 1), {edge_var(e)});

    // Flow conservation at node e: contribution of edge e.
    // Node e is the "from" node: -x_e appears in λ_e's constraint.
    // Node e+1 is the "to" node: +x_e appears in λ_{e+1}'s constraint.
    // Combined: C = [-1; 1] on primal {x_e}, dual {λ_e, λ_{e+1}}.
    Eigen::MatrixXd C(2, 1);
    C << -1.0, 1.0;
    VectorXd d_zero = VectorXd::Zero(2);
    builder.AddEquality(cid[e], C, d_zero,
                        {edge_var(e)},
                        {node_dual(e), node_dual(e + 1)});
  }

  auto result = builder.Build();
  ASSERT_TRUE(result.solver->AssembleAndFactor());

  // RHS: dual part gets the supply/demand vector.
  VectorXd rhs = VectorXd::Zero(result.num_variables);
  for (int v = 0; v < num_nodes; ++v)
    rhs(node_dual(v)) = b(v);

  VectorXd sol = result.solver->Solve(rhs);

  // All edge flows should equal 1 (unit flow from source to sink).
  for (int e = 0; e < num_edges; ++e) {
    EXPECT_NEAR(sol(edge_var(e)), 1.0, 1e-10)
        << "Edge " << e << " flow incorrect";
  }

  // Verify flow conservation: for each node, sum of incoming - outgoing = b.
  for (int v = 0; v < num_nodes; ++v) {
    double net = 0;
    if (v > 0) net += sol(edge_var(v - 1));         // incoming
    if (v < num_nodes - 1) net -= sol(edge_var(v));  // outgoing
    EXPECT_NEAR(net, b(v), 1e-10) << "Flow conservation at node " << v;
  }

  printf("NetworkFlow: %d nodes, %d edges, all flows=1.0, "
         "conservation verified\n",
         num_nodes, num_edges);
}

// Network flow: custom tree vs BuildFromSparseMatrices.
namespace {

struct SparseNetworkFlow {
  Eigen::SparseMatrix<double> Q_cost;
  Eigen::SparseMatrix<double> C_eq;
  Eigen::VectorXd d_eq;
  int n_primal;
  int n_eq;
};

SparseNetworkFlow MakeSparseNetworkFlow(int N) {
  int num_edges = N - 1;
  SparseNetworkFlow nf;
  nf.n_primal = num_edges;
  nf.n_eq = N;

  // Cost: I on edge variables.
  std::vector<Eigen::Triplet<double>> qt;
  for (int e = 0; e < num_edges; ++e)
    qt.emplace_back(e, e, 1.0);
  nf.Q_cost.resize(num_edges, num_edges);
  nf.Q_cost.setFromTriplets(qt.begin(), qt.end());

  // Flow conservation: for each node v,
  //   incoming edge - outgoing edge = b_v
  std::vector<Eigen::Triplet<double>> ct;
  for (int e = 0; e < num_edges; ++e) {
    ct.emplace_back(e, e, -1.0);      // outgoing from node e
    ct.emplace_back(e + 1, e, 1.0);   // incoming to node e+1
  }
  nf.C_eq.resize(N, num_edges);
  nf.C_eq.setFromTriplets(ct.begin(), ct.end());

  nf.d_eq = VectorXd::Zero(N);
  nf.d_eq(0) = -1.0;
  nf.d_eq(N - 1) = 1.0;
  return nf;
}

}  // namespace

TEST(TreeSolverBuilder, NetworkFlowCompare) {
  using clock = std::chrono::high_resolution_clock;

  printf("\n  Network flow: custom tree vs BuildFromSparseMatrices\n");
  printf("%-8s %22s %22s %8s\n",
         "N_nodes", "custom(build+fac+sol)", "sparse(build+fac+sol)", "speedup");
  printf("--------  ---------------------- ---------------------- --------\n");

  for (int N : {10, 50, 100, 500, 1000}) {
    int num_edges = N - 1;
    auto edge_var = [](int e) { return e; };
    auto node_dual = [&](int v) { return num_edges + v; };

    // Custom tree path.
    auto tc0 = clock::now();
    TreeSolverBuilder builder;
    std::vector<int> cid(num_edges);
    cid[num_edges - 1] = builder.AddClique();
    for (int e = num_edges - 2; e >= 0; --e)
      cid[e] = builder.AddClique(cid[e + 1]);
    for (int e = 0; e < num_edges; ++e) {
      builder.AddCost(cid[e], MatrixXd::Identity(1, 1), {edge_var(e)});
      Eigen::MatrixXd C(2, 1);
      C << -1.0, 1.0;
      builder.AddEquality(cid[e], C, VectorXd::Zero(2),
                          {edge_var(e)},
                          {node_dual(e), node_dual(e + 1)});
    }
    auto rc = builder.Build();
    auto tc1 = clock::now();
    rc.solver->AssembleAndFactor();
    auto tc2 = clock::now();
    VectorXd rhs_c = VectorXd::Zero(rc.num_variables);
    rhs_c(node_dual(0)) = -1.0;
    rhs_c(node_dual(N - 1)) = 1.0;
    auto sol_c = rc.solver->Solve(rhs_c);
    auto tc3 = clock::now();
    double custom_us = std::chrono::duration<double, std::micro>(tc3 - tc0).count();

    // Sparse matrix path.
    auto nf = MakeSparseNetworkFlow(N);
    auto ts0 = clock::now();
    auto rs = TreeSolverBuilder::BuildFromSparseMatrices(
        nf.Q_cost, nf.C_eq, nf.d_eq);
    auto ts1 = clock::now();
    rs.solver->AssembleAndFactor();
    auto ts2 = clock::now();
    VectorXd rhs_s = VectorXd::Zero(rs.num_variables);
    for (int v = 0; v < N; ++v) rhs_s(nf.n_primal + v) = nf.d_eq(v);
    auto sol_s = rs.solver->Solve(rhs_s);
    auto ts3 = clock::now();
    double sparse_us = std::chrono::duration<double, std::micro>(ts3 - ts0).count();

    // Both should give all flows = 1.
    EXPECT_NEAR(sol_c(0), 1.0, 1e-8);
    EXPECT_NEAR(sol_s(0), 1.0, 1e-8);

    printf("%-8d %12.0f          %12.0f          %7.1fx\n",
           N, custom_us, sparse_us, sparse_us / custom_us);
  }
}

// =====================================================================
// Multi-stage stochastic optimization:
//   min  Σ_nodes  x_i'Q x_i + u_i'R u_i
//   s.t. x_child = A x_parent + B u_parent,  x_root = x0
//
// The scenario tree IS the junction tree.  Each node becomes a clique
// with supernode {x_i, u_i, λ_i} and separator {x_parent}.
// =====================================================================

namespace {

struct ScenarioNode {
  int parent;                // -1 for root
  std::vector<int> children;
  int stage;
};

// Build a balanced scenario tree: B children per non-leaf, S stages.
std::vector<ScenarioNode> MakeScenarioTree(int B, int S) {
  std::vector<ScenarioNode> nodes;
  nodes.push_back({-1, {}, 0});  // root
  for (int s = 1; s < S; ++s) {
    int prev_start = 0, prev_end = static_cast<int>(nodes.size());
    for (int i = prev_start; i < prev_end; ++i) {
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

// Build sparse KKT matrices for the stochastic problem.
struct SparseStochastic {
  Eigen::SparseMatrix<double> Q_cost;
  Eigen::SparseMatrix<double> C_eq;
  Eigen::VectorXd d_eq;
  int n_primal, n_eq;
};

SparseStochastic MakeSparseStochastic(
    const std::vector<ScenarioNode>& tree,
    const MatrixXd& Ad, const MatrixXd& Bd,
    const MatrixXd& Q, const MatrixXd& R, const MatrixXd& Qf,
    const VectorXd& x0, int nx, int nu) {
  int N = static_cast<int>(tree.size());
  // Variable layout: node i gets [x_i (nx)] + [u_i (nu)] if non-leaf.
  std::vector<int> node_offset(N);
  int offset = 0;
  for (int i = 0; i < N; ++i) {
    node_offset[i] = offset;
    offset += nx + (tree[i].children.empty() ? 0 : nu);
  }
  int n_primal = offset;

  // Count equality rows: nx per non-root node (dynamics) + nx (initial cond).
  int n_eq = (N - 1) * nx + nx;

  // Cost.
  std::vector<Eigen::Triplet<double>> qt;
  for (int i = 0; i < N; ++i) {
    int xi = node_offset[i];
    const auto& Qi = tree[i].children.empty() ? Qf : Q;
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nx; ++c)
        if (Qi(r,c) != 0) qt.emplace_back(xi+r, xi+c, Qi(r,c));
    if (!tree[i].children.empty()) {
      int ui = xi + nx;
      for (int r = 0; r < nu; ++r)
        for (int c = 0; c < nu; ++c)
          if (R(r,c) != 0) qt.emplace_back(ui+r, ui+c, R(r,c));
    }
  }

  // Constraints: dynamics for each non-root node.
  std::vector<Eigen::Triplet<double>> ct;
  int eq_row = 0;
  for (int i = 1; i < N; ++i) {
    int p = tree[i].parent;
    int xi = node_offset[i];
    int xp = node_offset[p];
    int up = xp + nx;
    // x_i - A x_p - B u_p = 0
    for (int r = 0; r < nx; ++r) {
      ct.emplace_back(eq_row + r, xi + r, 1.0);
      for (int c = 0; c < nx; ++c)
        if (Ad(r,c) != 0) ct.emplace_back(eq_row+r, xp+c, -Ad(r,c));
      for (int c = 0; c < nu; ++c)
        if (Bd(r,c) != 0) ct.emplace_back(eq_row+r, up+c, -Bd(r,c));
    }
    eq_row += nx;
  }
  // Initial condition: x_root = x0.
  for (int i = 0; i < nx; ++i)
    ct.emplace_back(eq_row + i, i, 1.0);

  SparseStochastic ss;
  ss.n_primal = n_primal;
  ss.n_eq = n_eq;
  ss.Q_cost.resize(n_primal, n_primal);
  ss.Q_cost.setFromTriplets(qt.begin(), qt.end());
  ss.C_eq.resize(n_eq, n_primal);
  ss.C_eq.setFromTriplets(ct.begin(), ct.end());
  ss.d_eq = VectorXd::Zero(n_eq);
  ss.d_eq.tail(nx) = x0;
  return ss;
}

}  // namespace

TEST(StochasticOpt, CompareCustomVsSparse) {
  using clock = std::chrono::high_resolution_clock;

  const int nx = 4, nu = 2, B = 3;  // branching factor
  srand(42);
  MatrixXd Ad = 0.9 * MatrixXd::Identity(nx, nx) +
                0.1 * MatrixXd::Random(nx, nx);
  MatrixXd Bd = MatrixXd::Random(nx, nu);
  MatrixXd Q = MatrixXd::Identity(nx, nx) + 0.5 * MatrixXd::Ones(nx, nx);
  MatrixXd R = 0.1 * MatrixXd::Identity(nu, nu) + 0.05 * MatrixXd::Ones(nu, nu);
  MatrixXd Qf = 10.0 * Q;
  VectorXd x0 = VectorXd::Ones(nx);

  // Dynamics constraint: [-A, -B, I]
  MatrixXd C_dyn(nx, nx + nu + nx);
  C_dyn << -Ad, -Bd, MatrixXd::Identity(nx, nx);
  VectorXd d_zero = VectorXd::Zero(nx);

  // Cost block.
  MatrixXd QR = MatrixXd::Zero(nx + nu, nx + nu);
  QR.topLeftCorner(nx, nx) = Q;
  QR.bottomRightCorner(nu, nu) = R;

  printf("\n  Stochastic opt (branch=%d, nx=%d, nu=%d): "
         "custom tree vs BuildFromSparseMatrices\n", B, nx, nu);
  printf("%-7s %6s %7s %22s %22s %8s\n",
         "stages", "nodes", "n_vars",
         "custom(build+fac+sol)", "sparse(build+fac+sol)", "speedup");
  printf("-------  ------ ------- ---------------------- "
         "---------------------- --------\n");

  for (int S : {3, 4, 5, 6}) {
    auto tree = MakeScenarioTree(B, S);
    int N = static_cast<int>(tree.size());

    // --- Custom tree path ---
    // Variable layout: node i → [x_i, u_i, λ_i] in supernode,
    // separator = {x_parent}.
    // λ_i is the dynamics dual for coupling to parent.
    // Root has no dynamics dual but has initial condition dual λ_ic.

    // Compute variable offsets for the custom path.
    // Each non-root node: x(nx), u(nu if non-leaf), λ(nx dynamics dual)
    // Root: x(nx), u(nu), λ_ic(nx)
    std::vector<int> x_off(N), u_off(N), lam_off(N);
    int var_offset = 0;
    for (int i = 0; i < N; ++i) {
      x_off[i] = var_offset;
      var_offset += nx;
      if (!tree[i].children.empty()) {
        u_off[i] = var_offset;
        var_offset += nu;
      } else {
        u_off[i] = -1;
      }
      // Dual: dynamics coupling (non-root) or initial condition (root).
      lam_off[i] = var_offset;
      var_offset += nx;
    }
    int n_custom = var_offset;

    auto tc0 = clock::now();

    TreeSolverBuilder builder;

    // Create cliques: need parent-before-child for AddClique.
    // Nodes are already in BFS order (parent index < child index).
    std::vector<int> cid(N);
    cid[0] = builder.AddClique();  // root
    for (int i = 1; i < N; ++i)
      cid[i] = builder.AddClique(cid[tree[i].parent]);

    for (int i = 0; i < N; ++i) {
      bool is_leaf = tree[i].children.empty();
      bool is_root = (tree[i].parent == -1);

      // Cost block.
      if (is_leaf) {
        std::vector<int> cv;
        for (int j = 0; j < nx; ++j) cv.push_back(x_off[i] + j);
        builder.AddCost(cid[i], Qf, cv);
      } else {
        std::vector<int> cv;
        for (int j = 0; j < nx; ++j) cv.push_back(x_off[i] + j);
        for (int j = 0; j < nu; ++j) cv.push_back(u_off[i] + j);
        builder.AddCost(cid[i], QR, cv);
      }

      if (is_root) {
        // Initial condition: I on x_root, dual = λ_ic.
        std::vector<int> ic_p, ic_d;
        for (int j = 0; j < nx; ++j) ic_p.push_back(x_off[0] + j);
        for (int j = 0; j < nx; ++j) ic_d.push_back(lam_off[0] + j);
        builder.AddEquality(cid[0], MatrixXd::Identity(nx, nx),
                            d_zero, ic_p, ic_d);
      } else {
        // Dynamics: x_i = A x_parent + B u_parent
        // C = [-A, -B, I], primal = {x_parent, u_parent, x_i}, dual = {λ_i}
        int p = tree[i].parent;
        std::vector<int> dp, dd;
        for (int j = 0; j < nx; ++j) dp.push_back(x_off[p] + j);
        for (int j = 0; j < nu; ++j) dp.push_back(u_off[p] + j);
        for (int j = 0; j < nx; ++j) dp.push_back(x_off[i] + j);
        for (int j = 0; j < nx; ++j) dd.push_back(lam_off[i] + j);
        builder.AddEquality(cid[i], C_dyn, d_zero, dp, dd);
      }
    }

    auto rc = builder.Build();
    auto tc1 = clock::now();
    ASSERT_TRUE(rc.solver->AssembleAndFactor());
    auto tc2 = clock::now();

    VectorXd rhs_c = VectorXd::Zero(rc.num_variables);
    for (int j = 0; j < nx; ++j) rhs_c(lam_off[0] + j) = x0(j);
    auto sol_c = rc.solver->Solve(rhs_c);
    auto tc3 = clock::now();
    double custom_us =
        std::chrono::duration<double, std::micro>(tc3 - tc0).count();

    // Verify initial condition.
    double ic_err_c = 0;
    for (int j = 0; j < nx; ++j)
      ic_err_c = std::max(ic_err_c, std::abs(sol_c(x_off[0] + j, 0) - x0(j)));
    EXPECT_LT(ic_err_c, 1e-4) << "Custom: initial condition at S=" << S;

    // --- Sparse matrix path ---
    auto ss = MakeSparseStochastic(tree, Ad, Bd, Q, R, Qf, x0, nx, nu);
    auto ts0 = clock::now();
    auto rs = TreeSolverBuilder::BuildFromSparseMatrices(
        ss.Q_cost, ss.C_eq, ss.d_eq);
    auto ts1 = clock::now();
    ASSERT_TRUE(rs.solver->AssembleAndFactor());
    auto ts2 = clock::now();
    VectorXd rhs_s = VectorXd::Zero(rs.num_variables);
    for (int j = 0; j < ss.n_eq; ++j)
      rhs_s(ss.n_primal + j) = ss.d_eq(j);
    auto sol_s = rs.solver->Solve(rhs_s);
    auto ts3 = clock::now();
    double sparse_us =
        std::chrono::duration<double, std::micro>(ts3 - ts0).count();

    // Verify sparse initial condition.
    double ic_err_s = (sol_s.col(0).head(nx) - x0).norm();
    EXPECT_LT(ic_err_s, 1e-4) << "Sparse: initial condition at S=" << S;

    printf("%-7d %6d %7d %12.0f          %12.0f          %7.1fx\n",
           S, N, n_custom, custom_us, sparse_us,
           sparse_us / custom_us);
  }
}

// =====================================================================
// Weighted AMD on quotient graph: auto-computed elimination tree
// =====================================================================

// Small chain: same problem as SmallChain but without specifying parents.
TEST(TreeSolverBuilder, AutoTreeSmallChain) {
  TreeSolverBuilder b;
  int c0 = b.AddClique();  // no parent
  int c1 = b.AddClique();  // no parent

  b.AddCost(c0, MatrixXd::Identity(2, 2), {0, 1});
  b.AddCost(c1, MatrixXd::Identity(1, 1), {2});

  Eigen::MatrixXd C1(1, 3);
  C1 << 1, 1, -1;
  b.AddEquality(c0, C1, VectorXd::Zero(1), {0, 1, 2}, {3});
  b.AddEquality(c0, MatrixXd::Identity(1, 1), VectorXd::Zero(1), {0}, {4});

  auto result = b.Build();
  ASSERT_TRUE(result.solver->AssembleAndFactor());

  VectorXd rhs = VectorXd::Zero(result.num_variables);
  rhs(4) = 1.0;
  VectorXd sol = result.solver->Solve(rhs);

  EXPECT_NEAR(sol(0), 1.0, 1e-10);
  EXPECT_NEAR(sol(1), -0.5, 1e-10);
  EXPECT_NEAR(sol(2), 0.5, 1e-10);
  printf("AutoTree SmallChain: x=[%.4f, %.4f, %.4f]\n",
         sol(0), sol(1), sol(2));
}

// Stochastic opt with auto-computed tree vs explicit tree.
TEST(TreeSolverBuilder, AutoTreeStochastic) {
  const int nx = 4, nu = 2, B = 2, S = 4;
  srand(42);
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

  auto tree = MakeScenarioTree(B, S);
  int N = static_cast<int>(tree.size());

  // Variable offsets (same as CompareCustomVsSparse).
  std::vector<int> x_off(N), u_off(N), lam_off(N);
  int var_offset = 0;
  for (int i = 0; i < N; ++i) {
    x_off[i] = var_offset; var_offset += nx;
    u_off[i] = tree[i].children.empty() ? -1 : var_offset;
    if (!tree[i].children.empty()) var_offset += nu;
    lam_off[i] = var_offset; var_offset += nx;
  }

  // Helper to add blocks for a scenario node to a builder.
  auto add_node_blocks = [&](TreeSolverBuilder& builder, int cid, int i) {
    bool is_leaf = tree[i].children.empty();
    bool is_root = (tree[i].parent == -1);
    if (is_leaf) {
      std::vector<int> cv;
      for (int j = 0; j < nx; ++j) cv.push_back(x_off[i] + j);
      builder.AddCost(cid, Qf, cv);
    } else {
      std::vector<int> cv;
      for (int j = 0; j < nx; ++j) cv.push_back(x_off[i] + j);
      for (int j = 0; j < nu; ++j) cv.push_back(u_off[i] + j);
      builder.AddCost(cid, QR, cv);
    }
    if (is_root) {
      std::vector<int> ic_p, ic_d;
      for (int j = 0; j < nx; ++j) ic_p.push_back(x_off[0] + j);
      for (int j = 0; j < nx; ++j) ic_d.push_back(lam_off[0] + j);
      builder.AddEquality(cid, MatrixXd::Identity(nx, nx), d_zero, ic_p, ic_d);
    } else {
      int p = tree[i].parent;
      std::vector<int> dp, dd;
      for (int j = 0; j < nx; ++j) dp.push_back(x_off[p] + j);
      for (int j = 0; j < nu; ++j) dp.push_back(u_off[p] + j);
      for (int j = 0; j < nx; ++j) dp.push_back(x_off[i] + j);
      for (int j = 0; j < nx; ++j) dd.push_back(lam_off[i] + j);
      builder.AddEquality(cid, C_dyn, d_zero, dp, dd);
    }
  };

  // Explicit tree (reference).
  TreeSolverBuilder b_explicit;
  std::vector<int> cid_e(N);
  cid_e[0] = b_explicit.AddClique();
  for (int i = 1; i < N; ++i)
    cid_e[i] = b_explicit.AddClique(cid_e[tree[i].parent]);
  for (int i = 0; i < N; ++i)
    add_node_blocks(b_explicit, cid_e[i], i);
  auto r_explicit = b_explicit.Build();
  ASSERT_TRUE(r_explicit.solver->AssembleAndFactor());
  VectorXd rhs = VectorXd::Zero(r_explicit.num_variables);
  for (int j = 0; j < nx; ++j) rhs(lam_off[0] + j) = x0(j);
  auto sol_explicit = r_explicit.solver->Solve(rhs);

  // Auto tree (no parents specified).
  TreeSolverBuilder b_auto;
  b_auto.EnableRIPCheck();
  std::vector<int> cid_a(N);
  for (int i = 0; i < N; ++i)
    cid_a[i] = b_auto.AddClique();  // all roots
  for (int i = 0; i < N; ++i)
    add_node_blocks(b_auto, cid_a[i], i);

  auto r_auto = b_auto.Build();

  ASSERT_TRUE(r_auto.solver->AssembleAndFactor());
  VectorXd rhs_a = VectorXd::Zero(r_auto.num_variables);
  for (int j = 0; j < nx; ++j) rhs_a(lam_off[0] + j) = x0(j);
  auto sol_auto = r_auto.solver->Solve(rhs_a);

  // Verify: check ALL dynamics constraints, not just initial condition.
  double max_dyn_err_e = 0, max_dyn_err_a = 0;
  for (int i = 1; i < N; ++i) {
    int p = tree[i].parent;
    // x_i = A x_p + B u_p
    VectorXd x_i_e = sol_explicit.col(0).segment(x_off[i], nx);
    VectorXd x_p_e = sol_explicit.col(0).segment(x_off[p], nx);
    VectorXd u_p_e = sol_explicit.col(0).segment(u_off[p], nu);
    double err_e = (x_i_e - Ad * x_p_e - Bd * u_p_e).norm();
    max_dyn_err_e = std::max(max_dyn_err_e, err_e);

    VectorXd x_i_a = sol_auto.col(0).segment(x_off[i], nx);
    VectorXd x_p_a = sol_auto.col(0).segment(x_off[p], nx);
    VectorXd u_p_a = sol_auto.col(0).segment(u_off[p], nu);
    double err_a = (x_i_a - Ad * x_p_a - Bd * u_p_a).norm();
    max_dyn_err_a = std::max(max_dyn_err_a, err_a);
  }

  double ic_explicit = (sol_explicit.col(0).head(nx) - x0).norm();
  double ic_auto = (sol_auto.col(0).head(nx) - x0).norm();

  EXPECT_LT(ic_explicit, 1e-10) << "Explicit: initial condition";
  EXPECT_LT(max_dyn_err_e, 1e-10) << "Explicit: dynamics";
  // Auto tree uses AMD on the quotient graph, which may produce a
  // suboptimal elimination order for indefinite KKT systems.  The AMD
  // can eliminate a "hub" clique too early, placing its cost data in
  // a separator block rather than a supernode, leading to reduced
  // accuracy.  The explicit tree avoids this by preserving the natural
  // problem structure.
  EXPECT_LT(ic_auto, 1e-4) << "Auto: initial condition";
  EXPECT_LT(max_dyn_err_a, 1e-4) << "Auto: dynamics";

  printf("AutoTree Stochastic: S=%d, N=%d\n"
         "  explicit: ic=%.2e dyn=%.2e\n"
         "  auto:     ic=%.2e dyn=%.2e\n",
         S, N, ic_explicit, max_dyn_err_e, ic_auto, max_dyn_err_a);
}

// =====================================================================
// Fill-in comparison: explicit tree vs sparse-matrix AMD
// Generate synthetic stochastic trees and compare fill (Σ clique_size²)
// between the builder's auto-tree (quotient AMD) and the sparse path
// (full sparse-matrix AMD via BuildFromSparseMatrices).
// =====================================================================

TEST(FillComparison, StochasticTree) {
  const int nx = 4, nu = 2;
  srand(42);
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

  printf("\n  Fill comparison: explicit tree vs sparse-matrix AMD (stochastic tree)\n");
  printf("%-3s %-4s %6s %7s %12s %8s %12s %8s %8s\n",
         "S", "B", "nodes", "n_vars",
         "expl_fill", "expl_mc", "sp_fill", "sp_mc", "ratio");
  printf("--- ---- ------ ------- ------------ -------- "
         "------------ -------- --------\n");

  for (auto [S, B] : std::vector<std::pair<int,int>>{{3,2},{4,2},{5,2},{4,3},{5,3},{6,2}}) {
    auto tree = MakeScenarioTree(B, S);
    int N = static_cast<int>(tree.size());

    // Variable offsets.
    std::vector<int> x_off(N), u_off(N), lam_off(N);
    int var_offset = 0;
    for (int i = 0; i < N; ++i) {
      x_off[i] = var_offset; var_offset += nx;
      u_off[i] = tree[i].children.empty() ? -1 : var_offset;
      if (!tree[i].children.empty()) var_offset += nu;
      lam_off[i] = var_offset; var_offset += nx;
    }
    int n_vars = var_offset;

    auto add_blocks = [&](TreeSolverBuilder& builder, int cid, int i) {
      bool is_leaf = tree[i].children.empty();
      bool is_root = (tree[i].parent == -1);
      if (is_leaf) {
        std::vector<int> cv;
        for (int j = 0; j < nx; ++j) cv.push_back(x_off[i] + j);
        builder.AddCost(cid, Qf, cv);
      } else {
        std::vector<int> cv;
        for (int j = 0; j < nx; ++j) cv.push_back(x_off[i] + j);
        for (int j = 0; j < nu; ++j) cv.push_back(u_off[i] + j);
        builder.AddCost(cid, QR, cv);
      }
      if (is_root) {
        std::vector<int> ic_p, ic_d;
        for (int j = 0; j < nx; ++j) ic_p.push_back(x_off[0] + j);
        for (int j = 0; j < nx; ++j) ic_d.push_back(lam_off[0] + j);
        builder.AddEquality(cid, MatrixXd::Identity(nx, nx), d_zero, ic_p, ic_d);
      } else {
        int p = tree[i].parent;
        std::vector<int> dp, dd;
        for (int j = 0; j < nx; ++j) dp.push_back(x_off[p] + j);
        for (int j = 0; j < nu; ++j) dp.push_back(u_off[p] + j);
        for (int j = 0; j < nx; ++j) dp.push_back(x_off[i] + j);
        for (int j = 0; j < nx; ++j) dd.push_back(lam_off[i] + j);
        builder.AddEquality(cid, C_dyn, d_zero, dp, dd);
      }
    };

    // Explicit tree path (knows the scenario tree structure).
    TreeSolverBuilder b_quot;
    b_quot.EnableRIPCheck();
    std::vector<int> cid_q(N);
    cid_q[0] = b_quot.AddClique();
    for (int i = 1; i < N; ++i)
      cid_q[i] = b_quot.AddClique(cid_q[tree[i].parent]);
    for (int i = 0; i < N; ++i) add_blocks(b_quot, cid_q[i], i);
    auto r_quot = b_quot.Build();

    // sparse-matrix AMD path (sparse matrices).
    auto ss = MakeSparseStochastic(tree, Ad, Bd, Q, R, Qf, x0, nx, nu);
    auto r_kkt = TreeSolverBuilder::BuildFromSparseMatrices(
        ss.Q_cost, ss.C_eq, ss.d_eq);

    printf("%-3d %-4d %6d %7d %12lld %8d %12lld %8d %7.2fx\n",
           S, B, N, n_vars,
           r_quot.fill, r_quot.max_clique_size,
           r_kkt.fill, r_kkt.max_clique_size,
           static_cast<double>(r_kkt.fill) /
               std::max(r_quot.fill, 1LL));
  }
}

// Solve performance comparison: explicit tree vs sparse-matrix AMD.
// Both paths build a solver, then we time factor+solve separately
// from construction.
TEST(FillComparison, SolvePerformance) {
  using clock = std::chrono::high_resolution_clock;

  const int nx = 4, nu = 2;
  srand(42);
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

  printf("\n  Solve performance: explicit tree vs sparse-matrix AMD\n");
  printf("%-3s %-4s %6s  %8s %8s %8s  %8s %8s %8s  %8s %6s %6s\n",
         "S", "B", "nodes",
         "e_fac", "e_sol", "e_fill",
         "s_fac", "s_sol", "s_fill",
         "fac_rat", "e_res", "s_res");
  printf("--- ---- ------  -------- -------- --------  "
         "-------- -------- --------  -------- ------ ------\n");

  for (auto [S, B] : std::vector<std::pair<int,int>>{{3,2},{4,2},{5,2},{4,3},{5,3},{6,2},{6,3}}) {
    auto tree = MakeScenarioTree(B, S);
    int N = static_cast<int>(tree.size());

    std::vector<int> x_off(N), u_off(N), lam_off(N);
    int var_offset = 0;
    for (int i = 0; i < N; ++i) {
      x_off[i] = var_offset; var_offset += nx;
      u_off[i] = tree[i].children.empty() ? -1 : var_offset;
      if (!tree[i].children.empty()) var_offset += nu;
      lam_off[i] = var_offset; var_offset += nx;
    }

    auto add_blocks = [&](TreeSolverBuilder& builder, int cid, int i) {
      bool is_leaf = tree[i].children.empty();
      bool is_root = (tree[i].parent == -1);
      if (is_leaf) {
        std::vector<int> cv;
        for (int j = 0; j < nx; ++j) cv.push_back(x_off[i] + j);
        builder.AddCost(cid, Qf, cv);
      } else {
        std::vector<int> cv;
        for (int j = 0; j < nx; ++j) cv.push_back(x_off[i] + j);
        for (int j = 0; j < nu; ++j) cv.push_back(u_off[i] + j);
        builder.AddCost(cid, QR, cv);
      }
      if (is_root) {
        std::vector<int> ic_p, ic_d;
        for (int j = 0; j < nx; ++j) ic_p.push_back(x_off[0] + j);
        for (int j = 0; j < nx; ++j) ic_d.push_back(lam_off[0] + j);
        builder.AddEquality(cid, MatrixXd::Identity(nx, nx), d_zero, ic_p, ic_d);
      } else {
        int p = tree[i].parent;
        std::vector<int> dp, dd;
        for (int j = 0; j < nx; ++j) dp.push_back(x_off[p] + j);
        for (int j = 0; j < nu; ++j) dp.push_back(u_off[p] + j);
        for (int j = 0; j < nx; ++j) dp.push_back(x_off[i] + j);
        for (int j = 0; j < nx; ++j) dd.push_back(lam_off[i] + j);
        builder.AddEquality(cid, C_dyn, d_zero, dp, dd);
      }
    };

    // Explicit tree path.
    TreeSolverBuilder b_q;
    b_q.EnableRIPCheck();
    std::vector<int> cid_q(N);
    cid_q[0] = b_q.AddClique();
    for (int i = 1; i < N; ++i)
      cid_q[i] = b_q.AddClique(cid_q[tree[i].parent]);
    for (int i = 0; i < N; ++i) add_blocks(b_q, cid_q[i], i);
    auto r_q = b_q.Build();

    auto tq0 = clock::now();
    r_q.solver->AssembleAndFactor();
    auto tq1 = clock::now();
    VectorXd rhs_q = VectorXd::Zero(r_q.num_variables);
    for (int j = 0; j < nx; ++j) rhs_q(lam_off[0] + j) = x0(j);
    r_q.solver->Solve(rhs_q);
    auto tq2 = clock::now();
    double e_fac = std::chrono::duration<double, std::micro>(tq1 - tq0).count();
    double e_sol = std::chrono::duration<double, std::micro>(tq2 - tq1).count();

    // sparse-matrix AMD path.
    auto ss = MakeSparseStochastic(tree, Ad, Bd, Q, R, Qf, x0, nx, nu);
    auto r_k = TreeSolverBuilder::BuildFromSparseMatrices(
        ss.Q_cost, ss.C_eq, ss.d_eq);

    auto tk0 = clock::now();
    r_k.solver->AssembleAndFactor();
    auto tk1 = clock::now();
    VectorXd rhs_k = VectorXd::Zero(r_k.num_variables);
    for (int j = 0; j < ss.n_eq; ++j)
      rhs_k(ss.n_primal + j) = ss.d_eq(j);
    r_k.solver->Solve(rhs_k);
    auto tk2 = clock::now();
    double s_fac = std::chrono::duration<double, std::micro>(tk1 - tk0).count();
    double s_sol = std::chrono::duration<double, std::micro>(tk2 - tk1).count();

    // Check residual: Cx - d for both.
    auto sol_q = r_q.solver->Solve(rhs_q);
    auto sol_k = r_k.solver->Solve(rhs_k);
    double q_ic_err = 0, s_ic_err = 0;
    for (int j = 0; j < nx; ++j) {
      q_ic_err = std::max(q_ic_err, std::abs(sol_q(x_off[0] + j, 0) - x0(j)));
      s_ic_err = std::max(s_ic_err, std::abs(sol_k(0 + j, 0) - x0(j)));
    }

    printf("%-3d %-4d %6d  %7.0fus %7.0fus %8lld  %7.0fus %7.0fus %8lld  %7.2fx  %.0e %.0e\n",
           S, B, N,
           e_fac, e_sol, r_q.fill,
           s_fac, s_sol, r_k.fill,
           s_fac / std::max(e_fac, 1.0),
           q_ic_err, s_ic_err);
  }
}

// =====================================================================
// PD-only benchmark: chain of least-squares blocks (no dual variables).
// min Σ_t ||A_t [x_t; x_{t+1}] - b_t||^2
// This is entirely positive-definite, so quotient AMD should work well.
// =====================================================================

TEST(TreeSolverBuilder, PDChainAutoVsExplicit) {
  using clock = std::chrono::high_resolution_clock;
  srand(42);

  const int nx = 6;  // state dimension
  const int m = 10;  // measurements per timestep

  // Random A coupling consecutive timesteps: A_t is m x 2*nx.
  MatrixXd A_block = MatrixXd::Random(m, 2 * nx);
  VectorXd b_block = VectorXd::Random(m);

  printf("\n  PD chain (A'A only): quotient AMD vs explicit tree\n");
  printf("%-6s %7s  %8s %8s %8s  %8s %8s %8s  %8s %6s %6s\n",
         "T", "n_vars",
         "q_bld", "q_fac", "q_sol",
         "expl_bld", "expl_fac", "expl_sol",
         "fac_rat", "q_res", "e_res");
  printf("------  -------  -------- -------- --------  "
         "-------- -------- --------  -------- ------ ------\n");

  for (int T : {10, 25, 50, 100, 200}) {
    int n_vars = (T + 1) * nx;

    // Variable layout: x_t at [t*nx .. t*nx + nx - 1].
    auto x_idx = [&](int t) -> std::vector<int> {
      std::vector<int> v(nx);
      std::iota(v.begin(), v.end(), t * nx);
      return v;
    };

    // Build auto-tree (all parents = -1).
    auto ta0 = clock::now();
    TreeSolverBuilder b_auto;
    std::vector<int> cid_a(T);
    for (int t = 0; t < T; ++t)
      cid_a[t] = b_auto.AddClique();
    for (int t = 0; t < T; ++t) {
      // A_t [x_t; x_{t+1}] = b_t → vars = {x_t, x_{t+1}}
      std::vector<int> vars;
      auto xt = x_idx(t), xt1 = x_idx(t + 1);
      vars.insert(vars.end(), xt.begin(), xt.end());
      vars.insert(vars.end(), xt1.begin(), xt1.end());
      b_auto.AddLinearConstraint(cid_a[t], A_block, b_block, vars);
    }
    auto r_auto = b_auto.Build();
    auto ta1 = clock::now();
    ASSERT_TRUE(r_auto.solver->AssembleAndFactor());
    auto ta2 = clock::now();
    VectorXd rhs_a = VectorXd::Zero(r_auto.num_variables);
    // RHS = Σ A_t' b_t, scattered into the right positions.
    for (int t = 0; t < T; ++t) {
      auto vars_t = x_idx(t);
      auto vars_t1 = x_idx(t + 1);
      VectorXd atb = A_block.transpose() * b_block;
      for (int j = 0; j < nx; ++j) rhs_a(vars_t[j]) += atb(j);
      for (int j = 0; j < nx; ++j) rhs_a(vars_t1[j]) += atb(nx + j);
    }
    auto sol_a = r_auto.solver->Solve(rhs_a);
    auto ta3 = clock::now();

    // Build explicit tree (chain: 0 → 1 → ... → T-1).
    auto te0 = clock::now();
    TreeSolverBuilder b_expl;
    std::vector<int> cid_e(T);
    cid_e[T - 1] = b_expl.AddClique();
    for (int t = T - 2; t >= 0; --t)
      cid_e[t] = b_expl.AddClique(cid_e[t + 1]);
    for (int t = 0; t < T; ++t) {
      std::vector<int> vars;
      auto xt = x_idx(t), xt1 = x_idx(t + 1);
      vars.insert(vars.end(), xt.begin(), xt.end());
      vars.insert(vars.end(), xt1.begin(), xt1.end());
      b_expl.AddLinearConstraint(cid_e[t], A_block, b_block, vars);
    }
    auto r_expl = b_expl.Build();
    auto te1 = clock::now();
    ASSERT_TRUE(r_expl.solver->AssembleAndFactor());
    auto te2 = clock::now();
    auto sol_e = r_expl.solver->Solve(rhs_a);
    auto te3 = clock::now();

    // Residual: ||A x - b|| for a representative block.
    auto checs_residual = [&](const auto& sol) {
      auto xt = x_idx(0), xt1 = x_idx(1);
      VectorXd xcat(2 * nx);
      for (int j = 0; j < nx; ++j) xcat(j) = sol(xt[j], 0);
      for (int j = 0; j < nx; ++j) xcat(nx + j) = sol(xt1[j], 0);
      return (A_block * xcat - b_block).norm();
    };
    double res_a = checs_residual(sol_a);
    double res_e = checs_residual(sol_e);

    // Also check solutions match.
    double max_diff = (sol_a.col(0).head(n_vars) -
                       sol_e.col(0).head(n_vars)).cwiseAbs().maxCoeff();
    EXPECT_LT(max_diff, 1e-8)
        << "Quotient and explicit solutions differ at T=" << T;

    double q_bld = std::chrono::duration<double, std::micro>(ta1 - ta0).count();
    double q_fac = std::chrono::duration<double, std::micro>(ta2 - ta1).count();
    double q_sol = std::chrono::duration<double, std::micro>(ta3 - ta2).count();
    double expl_bld = std::chrono::duration<double, std::micro>(te1 - te0).count();
    double expl_fac = std::chrono::duration<double, std::micro>(te2 - te1).count();
    double expl_sol = std::chrono::duration<double, std::micro>(te3 - te2).count();

    printf("%-6d %7d  %7.0fus %7.0fus %7.0fus  %7.0fus %7.0fus %7.0fus  %7.2fx  %.0e %.0e\n",
           T, n_vars,
           q_bld, q_fac, q_sol,
           expl_bld, expl_fac, expl_sol,
           expl_fac / std::max(q_fac, 1.0),
           res_a, res_e);
  }
}

}  // namespace
}  // namespace conex
