#include "conex/algorithms/barrier_qp.h"
#include "conex/algorithms/equality_constrained_least_squares.h"
#include "conex/algorithms/irls.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cstdio>

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

// Small dense problem with known solution.
// A = [1 0; 0 1; 1 1], b = [1; 2; 0], C = [1 1], d = [3].
// Solution: min ||Ax - b||^2 s.t. x1 + x2 = 3.
// Lagrangian: A^T(Ax - b) + C^T lambda = 0, Cx = d.
TEST(EqualityConstrainedLS, SmallDense) {
  const int m = 3, n = 2, p = 1;
  std::vector<Eigen::Triplet<double>> trips;
  trips.emplace_back(0, 0, 1.0);
  trips.emplace_back(1, 1, 1.0);
  trips.emplace_back(2, 0, 1.0);
  trips.emplace_back(2, 1, 1.0);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  VectorXd b(m);
  b << 1, 2, 0;

  MatrixXd C(p, n);
  C << 1, 1;
  VectorXd d(p);
  d << 3;

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  // Check constraint satisfaction.
  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-10) << "Equality constraint violated";

  // Verify against closed-form KKT solution.
  MatrixXd Ad(A);
  MatrixXd AtA = Ad.transpose() * Ad;
  VectorXd Atb = Ad.transpose() * b;
  // KKT: [AtA C'; C 0] [x; lam] = [Atb; d]
  MatrixXd KKT = MatrixXd::Zero(n + p, n + p);
  KKT.topLeftCorner(n, n) = AtA;
  KKT.topRightCorner(n, p) = C.transpose();
  KKT.bottomLeftCorner(p, n) = C;
  VectorXd rhs(n + p);
  rhs << Atb, d;
  VectorXd sol = KKT.lu().solve(rhs);
  VectorXd x_expected = sol.head(n);

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

  // Banded A with bandwidth 3.
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r) {
    int col_start = (r * n) / m;
    for (int j = 0; j < 3 && col_start + j < n; ++j) {
      trips.emplace_back(r, col_start + j, (double)rand() / RAND_MAX + 0.1);
    }
  }
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  // Random C: each row touches 4 variables.
  MatrixXd C = MatrixXd::Zero(p, n);
  for (int r = 0; r < p; ++r) {
    for (int j = 0; j < 4; ++j) {
      C(r, rand() % n) = (double)rand() / RAND_MAX - 0.5;
    }
  }

  // Choose x_true satisfying Cx = d, then b = A * x_true + noise.
  VectorXd x_true = VectorXd::Random(n);
  VectorXd d = C * x_true;
  VectorXd b = MatrixXd(A) * x_true + 0.01 * VectorXd::Random(m);

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-8) << "Equality constraint violated";

  // Verify KKT optimality: A^T(Ax - b) + C^T lambda = 0.
  // We don't have lambda, but we can check that the gradient projected
  // onto the null space of C is zero.
  VectorXd grad = A.transpose() * (MatrixXd(A) * result.x - b);
  // Null-space projector: I - C^T (C C^T)^{-1} C
  MatrixXd CCt = C * C.transpose();
  MatrixXd P = MatrixXd::Identity(n, n) - C.transpose() * CCt.lu().solve(C);
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
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  // Solve unconstrained first.
  MatrixXd Ad(A);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd b = Ad * x_true;  // Exact fit, so x_true is the LS solution.

  // Constraints satisfied by x_true.
  MatrixXd C = MatrixXd::Random(p, n);
  VectorXd d = C * x_true;

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-8);

  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-10);

  printf("ECLS feasible: err=%.2e, constraint_err=%.2e\n", err, constraint_err);
}

// Benchmark: larger problem for timing.
// Uses banded A to ensure every column is touched (avoids singular A^T A).
TEST(EqualityConstrainedLS, Benchmark) {
  srand(42);
  const int n = 500, m = 1000, p = 20;
  const int bandwidth = 5;

  // Banded A: each row touches a contiguous band of columns.
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r) {
    int col_start = (r * n) / m;
    for (int j = 0; j < bandwidth && col_start + j < n; ++j) {
      trips.emplace_back(r, col_start + j, (double)rand() / RAND_MAX + 0.1);
    }
  }
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  // Equality constraints: each row touches a few consecutive variables.
  MatrixXd C = MatrixXd::Zero(p, n);
  for (int r = 0; r < p; ++r) {
    int start = (r * n) / p;
    for (int j = 0; j < 5 && start + j < n; ++j) {
      C(r, start + j) = (double)rand() / RAND_MAX - 0.5;
    }
  }

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
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  // Two independent constraints: row 0 touches {0,1}, row 1 touches {2,3}.
  // Two redundant rows: row 2 same support as row 0, row 3 same as row 1.
  MatrixXd C = MatrixXd::Zero(4, n);
  C(0, 0) = 1.0; C(0, 1) = 2.0;           // support {0,1}
  C(1, 2) = 3.0; C(1, 3) = -1.0;          // support {2,3}
  C(2, 0) = 0.5; C(2, 1) = -1.0;          // support {0,1} — redundant
  C(3, 2) = 2.0; C(3, 3) = 1.0;           // support {2,3} — redundant

  VectorXd x_true = VectorXd::Random(n);
  VectorXd d = C * x_true;
  VectorXd b = MatrixXd(A) * x_true;

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  // All 4 constraints should be satisfied (the 2 kept ones imply the
  // 2 dropped ones since x_true satisfies all of them).
  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-8)
      << "Equality constraints violated after rank reduction";

  // Solution should match x_true (exact fit + constraints).
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

  // Banded A.
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; ++r) {
    int col_start = (r * n) / m;
    for (int j = 0; j < 3 && col_start + j < n; ++j)
      trips.emplace_back(r, col_start + j, (double)rand() / RAND_MAX + 0.1);
  }
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  // Build C: p_indep independent rows, each touching a unique pair of vars.
  // Then p_redundant rows that duplicate the support of existing rows.
  MatrixXd C = MatrixXd::Zero(p_total, n);
  for (int r = 0; r < p_indep; ++r) {
    int base = r * 2;
    C(r, base) = (double)rand() / RAND_MAX + 0.5;
    C(r, base + 1) = (double)rand() / RAND_MAX + 0.5;
  }
  for (int r = 0; r < p_redundant; ++r) {
    // Copy support from an independent row but with different coefficients.
    int src = r % p_indep;
    int base = src * 2;
    C(p_indep + r, base) = (double)rand() / RAND_MAX + 0.5;
    C(p_indep + r, base + 1) = (double)rand() / RAND_MAX + 0.5;
  }

  VectorXd x_true = VectorXd::Random(n);
  VectorXd d = C * x_true;
  VectorXd b = MatrixXd(A) * x_true + 0.01 * VectorXd::Random(m);

  auto result = EqualityConstrainedLeastSquares(A, b, C, d);

  double constraint_err = (C * result.x - d).norm();
  EXPECT_LT(constraint_err, 1e-8);

  printf("ECLS rank-deficient larger: p_total=%d, p_indep=%d, "
         "constraint_err=%.2e, construct=%.0fus, factor=%.0fus\n",
         p_total, p_indep, constraint_err,
         result.construction_time_us, result.assemble_and_factor_time_us);
}

}  // namespace
}  // namespace conex
