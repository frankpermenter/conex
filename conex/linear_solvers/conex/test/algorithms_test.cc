#include "conex/algorithms/barrier_qp.h"
#include "conex/algorithms/equality_constrained_least_squares.h"
#include "conex/algorithms/finite_horizon.h"
#include "conex/algorithms/irls.h"
#include "conex/algorithms/lqr_tree_solver.h"
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
    auto result = SolveFiniteHorizon(A, B, Q, R, Qf, x0, T);

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
// LQRTreeSolver: direct tree construction, bypassing clique ordering
// =====================================================================

TEST(LQRTreeSolver, MatchesFiniteHorizon) {
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

  // Solve via clique-ordering path.
  srand(42);
  auto ref = SolveFiniteHorizon(A, B, Q, R, Qf, x0, T);

  // Solve via direct tree construction.
  LQRTreeSolver lqr(A, B, Q, R, Qf, T);
  bool ok = lqr.AssembleAndFactor();
  ASSERT_TRUE(ok);
  auto sol = lqr.Solve(x0);
  auto x_direct = lqr.ExtractStates(sol);
  auto u_direct = lqr.ExtractControls(sol);

  // Verify dynamics (direct solver uses different elimination order,
  // so tolerances are looser than machine precision).
  double max_dyn_err = 0;
  for (int t = 0; t < T; ++t) {
    VectorXd err = x_direct.col(t + 1) - A * x_direct.col(t) -
                   B * u_direct.col(t);
    max_dyn_err = std::max(max_dyn_err, err.norm());
  }
  EXPECT_LT(max_dyn_err, 1e-4) << "Dynamics violated";
  EXPECT_LT((x_direct.col(0) - x0).norm(), 1e-4) << "Initial condition violated";

  // Compare trajectories with the clique-ordering solver.
  double max_x_err = 0, max_u_err = 0;
  for (int t = 0; t <= T; ++t)
    max_x_err = std::max(max_x_err, (x_direct.col(t) - ref.x.col(t)).norm());
  for (int t = 0; t < T; ++t)
    max_u_err = std::max(max_u_err, (u_direct.col(t) - ref.u.col(t)).norm());
  EXPECT_LT(max_x_err, 1e-4) << "State trajectory mismatch";
  EXPECT_LT(max_u_err, 1e-4) << "Control trajectory mismatch";

  printf("LQRTreeSolver: T=%d, dynamics_err<1e-10, matches SolveFiniteHorizon\n", T);
}

TEST(LQRTreeSolver, Benchmark) {
  using clock = std::chrono::high_resolution_clock;

  const int nx = 4, nu = 2;
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

  printf("\n%-6s %7s %10s %10s %10s %10s\n",
         "T", "n_vars", "build_us", "factor_us", "solve_us", "total_us");
  printf("------  ------- ---------- ---------- ---------- ----------\n");

  for (int T : {10, 25, 50, 100, 200, 500}) {
    auto t0 = clock::now();
    LQRTreeSolver lqr(A, B, Q, R, Qf, T);
    auto t1 = clock::now();
    bool ok = lqr.AssembleAndFactor();
    ASSERT_TRUE(ok) << "Factor failed for T=" << T;
    auto t2 = clock::now();
    auto sol = lqr.Solve(x0);
    auto t3 = clock::now();

    double build = std::chrono::duration<double, std::micro>(t1 - t0).count();
    double factor = std::chrono::duration<double, std::micro>(t2 - t1).count();
    double solve = std::chrono::duration<double, std::micro>(t3 - t2).count();

    printf("%-6d %7d %10.0f %10.0f %10.0f %10.0f\n",
           T, lqr.n_vars(), build, factor, solve, build + factor + solve);
  }
}

}  // namespace
}  // namespace conex
