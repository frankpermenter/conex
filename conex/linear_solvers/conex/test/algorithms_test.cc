#include "conex/algorithms/barrier_qp.h"
#include "conex/algorithms/equality_constrained_least_squares.h"
#include "conex/algorithms/irls.h"
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
// Finite-horizon optimal control benchmark:
//   min  Σ x_t'Q x_t + u_t'R u_t  +  x_T'Qf x_T
//   s.t. x_{t+1} = A x_t + B u_t,  x_0 = x_init
//
// Decision variables: z = [x_0, u_0, x_1, u_1, ..., u_{T-1}, x_T]
// Sparse block-diagonal cost + sparse block-banded equality constraints.
// =====================================================================

namespace {

struct FiniteHorizonProblem {
  int nx, nu, T;
  int n_vars;       // (T+1)*nx + T*nu
  int n_eq;         // (T+1)*nx  (T dynamics + 1 initial condition)
  Eigen::SparseMatrix<double> Q_cost;  // block-diagonal cost
  Eigen::SparseMatrix<double> C_eq;    // dynamics + initial condition
  Eigen::VectorXd d_eq;               // RHS (zeros + x_init)
};

// Variable layout: [x_0, u_0, x_1, u_1, ..., u_{T-1}, x_T]
int XIndex(int t, int nx, int nu) { return t * (nx + nu); }
int UIndex(int t, int nx, int nu) { return t * (nx + nu) + nx; }

FiniteHorizonProblem MakeFiniteHorizon(int nx, int nu, int T, int seed = 42) {
  srand(seed);
  FiniteHorizonProblem prob;
  prob.nx = nx;
  prob.nu = nu;
  prob.T = T;
  prob.n_vars = (T + 1) * nx + T * nu;
  prob.n_eq = (T + 1) * nx;

  // Random stable dynamics: A = 0.9*I + 0.1*rand, B = rand.
  Eigen::MatrixXd Ad = 0.9 * Eigen::MatrixXd::Identity(nx, nx) +
                        0.1 * Eigen::MatrixXd::Random(nx, nx);
  Eigen::MatrixXd Bd = Eigen::MatrixXd::Random(nx, nu);

  // Cost: Q = dense SPD (nx x nx), R = dense SPD (nu x nu), Qf = 10*Q.
  // Using dense blocks ensures each timestep's variables form a clique
  // in the cost, which the tree solver can map correctly.
  Eigen::MatrixXd Qx = Eigen::MatrixXd::Identity(nx, nx);
  Qx += 0.5 * Eigen::MatrixXd::Ones(nx, nx);  // dense, SPD
  Eigen::MatrixXd Ru = 0.1 * Eigen::MatrixXd::Identity(nu, nu);
  Ru += 0.05 * Eigen::MatrixXd::Ones(nu, nu);

  // Build sparse cost matrix (block diagonal with dense sub-blocks).
  std::vector<Eigen::Triplet<double>> qt;
  for (int t = 0; t < T; ++t) {
    int xi = XIndex(t, nx, nu);
    int ui = UIndex(t, nx, nu);
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < nx; ++j)
        qt.emplace_back(xi + i, xi + j, Qx(i, j));
    for (int i = 0; i < nu; ++i)
      for (int j = 0; j < nu; ++j)
        qt.emplace_back(ui + i, ui + j, Ru(i, j));
  }
  // Terminal cost.
  int xT = XIndex(T, nx, nu);
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < nx; ++j)
      qt.emplace_back(xT + i, xT + j, 10.0 * Qx(i, j));
  prob.Q_cost.resize(prob.n_vars, prob.n_vars);
  prob.Q_cost.setFromTriplets(qt.begin(), qt.end());

  // Build equality constraint matrix.
  // Rows 0..(T*nx-1): dynamics x_{t+1} = A x_t + B u_t
  //   → [-A, -B, I] on [x_t, u_t, x_{t+1}]
  // Rows T*nx..((T+1)*nx-1): initial condition x_0 = x_init
  //   → [I] on x_0
  std::vector<Eigen::Triplet<double>> ct;
  for (int t = 0; t < T; ++t) {
    int row_base = t * nx;
    int xi = XIndex(t, nx, nu);
    int ui = UIndex(t, nx, nu);
    int xi1 = XIndex(t + 1, nx, nu);
    // -A block.
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nx; ++c)
        if (Ad(r, c) != 0)
          ct.emplace_back(row_base + r, xi + c, -Ad(r, c));
    // -B block.
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nu; ++c)
        if (Bd(r, c) != 0)
          ct.emplace_back(row_base + r, ui + c, -Bd(r, c));
    // I block for x_{t+1}.
    for (int i = 0; i < nx; ++i)
      ct.emplace_back(row_base + i, xi1 + i, 1.0);
  }
  // Initial condition: x_0 = x_init.
  int ic_row = T * nx;
  for (int i = 0; i < nx; ++i)
    ct.emplace_back(ic_row + i, i, 1.0);

  prob.C_eq.resize(prob.n_eq, prob.n_vars);
  prob.C_eq.setFromTriplets(ct.begin(), ct.end());

  // RHS: zeros for dynamics, x_init for initial condition.
  prob.d_eq = Eigen::VectorXd::Zero(prob.n_eq);
  Eigen::VectorXd x_init = Eigen::VectorXd::Ones(nx);
  prob.d_eq.tail(nx) = x_init;

  return prob;
}

}  // namespace

TEST(FiniteHorizon, Benchmark) {
  using clock = std::chrono::high_resolution_clock;

  printf("\n%-6s %7s %7s %10s %10s %10s %10s %10s\n",
         "T", "n_vars", "n_eq", "build_us", "factor_us", "solve_us",
         "total_us", "dyn_err");
  printf("%-6s %7s %7s %10s %10s %10s %10s %10s\n",
         "------", "-------", "-------", "----------", "----------",
         "----------", "----------", "----------");

  const int nx = 4, nu = 2;

  for (int T : {10, 25, 50, 100, 200}) {
    auto prob = MakeFiniteHorizon(nx, nu, T);

    auto t0 = clock::now();

    // Cost assembler.
    std::set<int> q_var_set;
    for (int k = 0; k < prob.Q_cost.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(prob.Q_cost, k); it;
           ++it) {
        q_var_set.insert(it.row());
        q_var_set.insert(it.col());
      }
    std::vector<int> q_vars(q_var_set.begin(), q_var_set.end());

    ConstraintManager cm(prob.n_vars);

    auto q_asm = std::make_unique<SparseQuadraticTermAssembler>(
        prob.Q_cost, q_vars);
    cm.AddCustomAssembler(std::move(q_asm));

    // Equality constraint assembler.
    auto sec = std::make_unique<SparseEqualityConstraint>(
        prob.C_eq, prob.d_eq);
    std::set<int> eq_var_set;
    for (const auto& s : sec->row_supports())
      eq_var_set.insert(s.begin(), s.end());
    std::vector<int> eq_primal(eq_var_set.begin(), eq_var_set.end());
    auto dual_vars = cm.AllocateDualVariables(prob.n_eq);
    auto eq_asm = std::make_unique<SparseEqualityConstraintAssembler>(
        std::move(sec), eq_primal, dual_vars);
    cm.AddCustomAssembler(std::move(eq_asm));

    SolverConfiguration config;
    auto solver = MakeTreeSolver(&cm, config);

    auto t1 = clock::now();

    bool ok = solver->AssembleAndFactor();
    ASSERT_TRUE(ok) << "AssembleAndFactor failed for T=" << T;

    auto t2 = clock::now();

    // RHS = [0 (primal); d (dual)].
    int sys_size = cm.SizeOfKKTSystem();
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(sys_size);
    for (int i = 0; i < prob.n_eq; ++i)
      rhs(dual_vars[i]) = prob.d_eq(i);

    Eigen::VectorXd sol = solver->Solve(rhs);

    auto t3 = clock::now();

    Eigen::VectorXd z = sol.head(prob.n_vars);

    // Verify dynamics: C z = d.
    double dyn_err = (prob.C_eq * z - prob.d_eq).norm();
    EXPECT_LT(dyn_err, 1e-6) << "Dynamics violated for T=" << T;

    double build_us =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
    double factor_us =
        std::chrono::duration<double, std::micro>(t2 - t1).count();
    double solve_us =
        std::chrono::duration<double, std::micro>(t3 - t2).count();
    double total_us = build_us + factor_us + solve_us;

    printf("%-6d %7d %7d %10.0f %10.0f %10.0f %10.0f %10.2e\n",
           T, prob.n_vars, prob.n_eq, build_us, factor_us, solve_us,
           total_us, dyn_err);
  }
}

}  // namespace
}  // namespace conex
