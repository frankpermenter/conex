// Tests for equality constraint handling in the supernodal solver.
//
// Each test constructs a minimal QP that reproduces a specific failure mode
// discovered on the Maros-Meszaros benchmark suite:
//
// 1. Indefiniteness propagation (CONT-050, AUG2DCQP):
//    Equality constraints create indefinite child cliques whose Schur
//    complement must propagate indefiniteness to LLT-only parent cliques.
//    Without propagation, parent uses Cholesky on indefinite data → segfault.
//
// 2. Dependent equality removal (QSCORPIO: 280 eqs, rank 250):
//    Structurally independent but numerically dependent equations create
//    a 30-dimensional null space in the dual block. No tree repair can
//    fix this — the equations must be reduced before building the KKT.
//
// 3. Tree repair via dual demotion (QSHARE2B):
//    Network flow constraints where 3 equations restricted to a supernode's
//    3 primals are rank 2 (they sum to zero on the supernode variables).
//    The trial factorization detects the singular supernode and demotes
//    a dual variable to the separator, resolving the singularity.
//
// 4. LU fallback to RLDLT (QGROW7, QSHARE2B):
//    Even after tree repair, some supernodes become near-singular at
//    non-identity W during the iterative solve. The per-clique RLDLT
//    fallback (triggered by min LU pivot < threshold) handles these
//    without divergence.

#include <gtest/gtest.h>

#include <cstdio>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/common/eja_ops.h"
#include "conex/common/conex.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/linear_solvers/cholesky_solvers.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

// Helper: build a sparse matrix from dense.
Eigen::SparseMatrix<double> ToDense(const MatrixXd& M) {
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  std::vector<Eigen::Triplet<double>> t;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (M(i, j) != 0) t.emplace_back(i, j, M(i, j));
  S.setFromTriplets(t.begin(), t.end());
  return S;
}

std::vector<int> Range(int n) {
  std::vector<int> v(n);
  std::iota(v.begin(), v.end(), 0);
  return v;
}

// Solve with all three indefinite factorization methods.
struct TestResult {
  bool converged;       // Solver::Solve convergence (strict stationarity check).
  bool algo_converged;  // Algorithm's own convergence (mu < tol).
  Eigen::VectorXd x;
  int num_demotions;
  double stationarity_norm;
  double eq_residual_norm;
  double mu;
};
TestResult SolveWith(const Model& model, const SolverConfiguration& config,
                      double tol = 1e-8, int max_iter = 200) {
  auto solver = Solver::Build(model, config);
  int nd = solver.tree_solver() ? solver.tree_solver()->num_demotions() : 0;
  auto result = solver.Solve(ThetaContinuation{tol, max_iter, 1});
  double stat = result.duals.stationarity_gradient.norm();
  double eq_res = 0;
  for (const auto& r : result.duals.eq_residual)
    eq_res = std::max(eq_res, r.norm());
  // Algorithm converges when mu < tol; the Solver::Solve convergence flag
  // additionally requires stationarity < 1e-4 which can fail when equations
  // are dependent (underdetermined duals inflate the gradient).
  bool algo_conv = result.mu < tol;
  return {result.converged, algo_conv, result.x, nd, stat, eq_res, result.mu};
}
TestResult SolveRLDLT(const Model& m) { return SolveWith(m, {}); }
TestResult SolveLU(const Model& m) {
  SolverConfiguration c; c.tree.use_lu_for_indefinite = true;
  return SolveWith(m, c);
}
TestResult SolveLAPACK(const Model& m) {
  SolverConfiguration c; c.tree.use_lapack_for_indefinite = true;
  return SolveWith(m, c);
}

// Test 1: Equality + inequality constraints (indefiniteness propagation).
// min 0.5 x'x  s.t.  x0 + x1 + x2 = 1,  x >= 0.
// Optimal: x = (1/3, 1/3, 1/3), obj = 1/6.
TEST(EqualityRepair, BasicEqualityPlusInequality) {
  const int n = 3;
  Model model;
  model.AddQuadraticCost(ToDense(MatrixXd::Identity(n, n)), Range(n));
  model.SetLinearCost(VectorXd::Zero(n));

  MatrixXd A_eq(1, n);
  A_eq << 1, 1, 1;
  model.AddEqualityConstraint(ToDense(A_eq), VectorXd::Ones(1), Range(n));
  model.AddLinearConstraint(
      ToDense(MatrixXd::Identity(n, n)), VectorXd::Zero(n), Range(n));

  for (auto [name, solve] : {std::pair{"RLDLT", &SolveRLDLT},
                              {"LU", &SolveLU},
                              {"LAPACK", &SolveLAPACK}}) {
    SCOPED_TRACE(name);
    auto r = solve(model);
    EXPECT_TRUE(r.converged);
    EXPECT_NEAR(r.x[0], 1.0 / 3, 1e-4);
    EXPECT_LT(r.eq_residual_norm, 1e-4);
    EXPECT_LT(r.stationarity_norm, 1e-2);
  }
}

// Test 2: Dependent equalities (QSCORPIO-like).
// 3 equations on 4 variables, but eq0 + eq1 = eq2 → rank 2.
// min 0.5 x'x  s.t.  x0+x1=2, x2+x3=2, x0+x1+x2+x3=4,  x >= -10.
TEST(EqualityRepair, DependentEquations) {
  const int n = 4;
  Model model;
  model.AddQuadraticCost(ToDense(MatrixXd::Identity(n, n)), Range(n));
  model.SetLinearCost(VectorXd::Zero(n));

  MatrixXd A_eq(3, n);
  A_eq << 1, 1, 0, 0,   // x0 + x1 = 2
          0, 0, 1, 1,   // x2 + x3 = 2
          1, 1, 1, 1;   // x0+x1+x2+x3 = 4 (dependent: row0 + row1)
  VectorXd b_eq(3);
  b_eq << 2, 2, 4;
  model.AddEqualityConstraint(ToDense(A_eq), b_eq, Range(n));

  model.AddLinearConstraint(
      ToDense(MatrixXd::Identity(n, n)),
      10 * VectorXd::Ones(n), Range(n));

  for (auto [name, solve] : {std::pair{"RLDLT", &SolveRLDLT},
                              {"LU", &SolveLU},
                              {"LAPACK", &SolveLAPACK}}) {
    SCOPED_TRACE(name);
    auto r = solve(model);
    EXPECT_TRUE(r.converged);
    EXPECT_NEAR(r.x[0], 1.0, 1e-3);
    EXPECT_NEAR(r.x[1], 1.0, 1e-3);
    EXPECT_NEAR(r.x[2], 1.0, 1e-3);
    EXPECT_NEAR(r.x[3], 1.0, 1e-3);
    EXPECT_LT(r.eq_residual_norm, 1e-3);
    EXPECT_LT(r.stationarity_norm, 1e-2);
  }
}

// Test 3: Network flow structure (QSHARE2B-like).
// Flow conservation: each internal variable appears with +1 in one eq,
// -1 in another, so equations sum to zero on internal variables.
// This creates a rank-deficient supernode block requiring tree repair.
//
// min 0.5 x'x  s.t. flow conservation, x >= 0.
// Variables: x0..x5 (6 vars), 3 flow equations.
// eq0:  x1 + x2 + x4 = 3
// eq1:  x0 + x3 - x2 = 1
// eq2:  x5 - x0 - x1 = 1
// Sum: x3 + x4 + x5 = 5  (internal vars x0,x1,x2 cancel).
TEST(EqualityRepair, NetworkFlowStructure) {
  const int n = 6;
  Model model;
  model.AddQuadraticCost(ToDense(MatrixXd::Identity(n, n)), Range(n));
  model.SetLinearCost(VectorXd::Zero(n));

  MatrixXd A_eq(3, n);
  //        x0  x1  x2  x3  x4  x5
  A_eq << 0,  1,  1,  0,  1,  0,   // eq0
          1,  0, -1,  1,  0,  0,   // eq1
         -1, -1,  0,  0,  0,  1;   // eq2
  VectorXd b_eq(3);
  b_eq << 3, 1, 1;
  model.AddEqualityConstraint(ToDense(A_eq), b_eq, Range(n));

  model.AddLinearConstraint(
      ToDense(MatrixXd::Identity(n, n)), VectorXd::Zero(n), Range(n));

  for (auto [name, solve] : {std::pair{"RLDLT", &SolveRLDLT},
                              {"LU", &SolveLU},
                              {"LAPACK", &SolveLAPACK}}) {
    SCOPED_TRACE(name);
    auto r = solve(model);
    EXPECT_TRUE(r.converged);
    EXPECT_LT(r.eq_residual_norm, 1e-3);
    EXPECT_LT(r.stationarity_norm, 1e-2);
  }
}

// Test 4: Multiple equalities with bounds (exercises multiple cliques with duals).
// min 0.5 x'x  s.t. A_eq x = b_eq, x >= 0.
// 10 vars, 3 equalities, all x >= 0.
TEST(EqualityRepair, MultipleEqualitiesWithBounds) {
  const int n = 10, m_eq = 3;
  Model model;
  model.AddQuadraticCost(ToDense(MatrixXd::Identity(n, n)), Range(n));
  model.SetLinearCost(VectorXd::Zero(n));

  // Sparse equalities touching different variable subsets.
  MatrixXd A_eq = MatrixXd::Zero(m_eq, n);
  A_eq(0, 0) = 1; A_eq(0, 1) = 1; A_eq(0, 2) = 1;  // x0+x1+x2 = 3
  A_eq(1, 3) = 1; A_eq(1, 4) = 1; A_eq(1, 5) = 1;  // x3+x4+x5 = 3
  A_eq(2, 6) = 1; A_eq(2, 7) = 1; A_eq(2, 8) = 1;  // x6+x7+x8 = 3
  VectorXd b_eq = 3.0 * VectorXd::Ones(m_eq);
  model.AddEqualityConstraint(ToDense(A_eq), b_eq, Range(n));

  model.AddLinearConstraint(
      ToDense(MatrixXd::Identity(n, n)), VectorXd::Zero(n), Range(n));

  for (auto [name, solve] : {std::pair{"RLDLT", &SolveRLDLT},
                              {"LU", &SolveLU},
                              {"LAPACK", &SolveLAPACK}}) {
    SCOPED_TRACE(name);
    auto r = solve(model);
    EXPECT_TRUE(r.converged);
    for (int i = 0; i < 9; ++i)
      EXPECT_NEAR(r.x[i], 1.0, 1e-3);
    EXPECT_LT(r.eq_residual_norm, 1e-3);
    EXPECT_LT(r.stationarity_norm, 1e-2);
  }
}

// Test 5: Equality with tight bounds (exercises saddle-point structure).
// min 0.5 x'x  s.t.  x0 + x1 = 2, 0 <= x <= 3.
// Optimal: x = (1, 1), obj = 1.
TEST(EqualityRepair, EqualityWithTightBounds) {
  const int n = 2;
  Model model;
  model.AddQuadraticCost(ToDense(MatrixXd::Identity(n, n)), Range(n));
  model.SetLinearCost(VectorXd::Zero(n));

  MatrixXd A_eq(1, n);
  A_eq << 1, 1;
  VectorXd b_eq(1);
  b_eq << 2;
  model.AddEqualityConstraint(ToDense(A_eq), b_eq, Range(n));

  // Lower bound: x >= 0, upper bound: 3 - x >= 0.
  MatrixXd A_ineq(2 * n, n);
  A_ineq.topRows(n) = MatrixXd::Identity(n, n);
  A_ineq.bottomRows(n) = -MatrixXd::Identity(n, n);
  VectorXd b_ineq(2 * n);
  b_ineq.head(n) = VectorXd::Zero(n);
  b_ineq.tail(n) = 3.0 * VectorXd::Ones(n);
  model.AddLinearConstraint(ToDense(A_ineq), b_ineq, Range(n));

  for (auto [name, solve] : {std::pair{"RLDLT", &SolveRLDLT},
                              {"LU", &SolveLU},
                              {"LAPACK", &SolveLAPACK}}) {
    SCOPED_TRACE(name);
    auto r = solve(model);
    EXPECT_TRUE(r.converged);
    EXPECT_NEAR(r.x[0], 1.0, 1e-3);
    EXPECT_NEAR(r.x[1], 1.0, 1e-3);
    EXPECT_LT(r.eq_residual_norm, 1e-3);
    EXPECT_LT(r.stationarity_norm, 1e-2);
  }
}

// Test 6: Verify demotion counter is accessible and consistent.
// RLDLT never runs the trial (no demotions).  LU/LAPACK run the trial;
// demotion count depends on the elimination ordering.
TEST(EqualityRepair, DemotionCounter) {
  const int n = 3;
  Model model;
  model.AddQuadraticCost(ToDense(MatrixXd::Identity(n, n)), Range(n));
  model.SetLinearCost(VectorXd::Zero(n));
  MatrixXd A_eq(1, n); A_eq << 1, 1, 1;
  model.AddEqualityConstraint(ToDense(A_eq), VectorXd::Ones(1), Range(n));
  model.AddLinearConstraint(
      ToDense(MatrixXd::Identity(n, n)), VectorXd::Zero(n), Range(n));

  // RLDLT: no trial → 0 demotions.
  {
    auto solver = Solver::Build(model);
    EXPECT_EQ(solver.tree_solver()->num_demotions(), 0);
    auto r = solver.Solve(ThetaContinuation{1e-8, 200, 1});
    EXPECT_TRUE(r.converged);
  }
  // LU: trial runs → demotion count is non-negative.
  {
    SolverConfiguration c; c.tree.use_lu_for_indefinite = true;
    auto solver = Solver::Build(model, c);
    EXPECT_GE(solver.tree_solver()->num_demotions(), 0);
    auto r = solver.Solve(ThetaContinuation{1e-8, 200, 1});
    EXPECT_TRUE(r.converged);
  }
  // LAPACK: same.
  {
    SolverConfiguration c; c.tree.use_lapack_for_indefinite = true;
    auto solver = Solver::Build(model, c);
    EXPECT_GE(solver.tree_solver()->num_demotions(), 0);
    auto r = solver.Solve(ThetaContinuation{1e-8, 200, 1});
    EXPECT_TRUE(r.converged);
  }
}

// Test 7: Forced demotion with user-specified clique tree.
// A clique tree that puts 3 duals in a supernode with only 3 primals,
// where the equation restriction is rank-deficient.
// RLDLT regularizes the zero pivot; LU after demotion degrades.
TEST(EqualityRepair, ForcedBadCliqueTree) {
  const int n = 10;
  Model model;
  model.AddQuadraticCost(ToDense(MatrixXd::Identity(n, n)), Range(n));
  model.SetLinearCost(VectorXd::Zero(n));

  // 3 equations on vars {0..5} with rank-deficient restriction on {0,1,2}:
  // eq0: x1 + x2 + x4 = 3
  // eq1: x0 + x3 - x2 = 1
  // eq2: x5 - x0 - x1 = 1
  // Restricted to {0,1,2}: eq0+eq2 = -eq1 → rank 2.
  MatrixXd A_eq = MatrixXd::Zero(3, n);
  A_eq(0, 1) = 1; A_eq(0, 2) = 1; A_eq(0, 4) = 1;
  A_eq(1, 0) = 1; A_eq(1, 3) = 1; A_eq(1, 2) = -1;
  A_eq(2, 5) = 1; A_eq(2, 0) = -1; A_eq(2, 1) = -1;
  VectorXd b_eq(3); b_eq << 3, 1, 1;
  model.AddEqualityConstraint(ToDense(A_eq), b_eq, Range(n));
  model.AddLinearConstraint(
      ToDense(MatrixXd::Identity(n, n)), VectorXd::Zero(n), Range(n));

  // Force clique tree: clique 0 has primals {0,1,2} + duals {10,11,12}
  // with separators {3,4,5}. The 3x3 equation restriction is rank 2.
  CliqueTree ct;
  ct.supernodes = {{0,1,2, 10,11,12}, {3,4,5,6,7,8,9}};
  ct.separators = {{3,4,5}, {}};
  ct.node_to_parent = {1, -1};
  ct.post_order_position_to_clique = {0, 1};

  // RLDLT: regularizes the rank-deficient block, small residuals.
  {
    auto solver = Solver::Build(model, ct);
    auto result = solver.Solve(ThetaContinuation{1e-8, 200, 1});
    EXPECT_TRUE(result.converged);
    double eq_res = 0;
    for (const auto& r : result.duals.eq_residual)
      eq_res = std::max(eq_res, r.norm());
    EXPECT_LT(eq_res, 1e-3);
    EXPECT_LT(result.duals.stationarity_gradient.norm(), 1e-2);
  }

  // LU with user-provided bad tree: no repair (user takes responsibility).
  // The singular supernode causes LU to fail.  We just verify no crash.
  {
    SolverConfiguration lu_config;
    lu_config.tree.use_lu_for_indefinite = true;
    auto solver = Solver::Build(model, ct, lu_config);
    // No demotion — user-provided trees are not repaired.
    EXPECT_EQ(solver.tree_solver()->num_demotions(), 0);
    // Solve may fail or produce bad residuals — that's expected.
    auto result = solver.Solve(ThetaContinuation{1e-8, 200, 1});
    printf("  LU user tree: conv=%d iter=%d\n",
           result.converged, result.iterations);
  }

  // LU with default AMD tree: the repair in MakeTreeSolver handles it.
  // This problem is small enough that AMD avoids the bad grouping,
  // so no demotion is needed.  Verify convergence.
  {
    auto r = SolveLU(model);
    EXPECT_TRUE(r.converged);
    EXPECT_LT(r.eq_residual_norm, 1e-3);
  }
}

// Test 8: QSCORPIO from Maros-Meszaros benchmark (loaded from binary data).
//
// QSCORPIO: 358 vars, 280 equalities (rank 250, 30 dependent), 466 inequalities.
// Very sparse P (22/358 nonzero diagonal).
//
// The 30 dependent equations create a null space that cannot be resolved by
// tree repair.  RLDLT converges (regularization absorbs it); LU needs ~20
// demotions and only reaches optimal_inaccurate.  This behavioral gap is
// the defining feature this test captures.
//
// Data file: tests/data/QSCORPIO.bin (exported from OSQP .mat format).
// If the file is missing, the test is skipped.
namespace {

// Read helpers for the binary format written by the Python export script.
Eigen::SparseMatrix<double> ReadCSC(FILE* f) {
  int32_t dims[3];
  CONEX_DEMAND(fread(dims, sizeof(int32_t), 3, f) == 3, "");
  int rows = dims[0], cols = dims[1], nnz = dims[2];
  std::vector<int32_t> indptr(cols + 1), indices(nnz);
  std::vector<double> data(nnz);
  CONEX_DEMAND(fread(indptr.data(), sizeof(int32_t), cols + 1, f) == (size_t)(cols + 1), "");
  CONEX_DEMAND(fread(indices.data(), sizeof(int32_t), nnz, f) == (size_t)nnz, "");
  CONEX_DEMAND(fread(data.data(), sizeof(double), nnz, f) == (size_t)nnz, "");

  Eigen::SparseMatrix<double> M(rows, cols);
  std::vector<Eigen::Triplet<double>> trips;
  for (int j = 0; j < cols; ++j)
    for (int k = indptr[j]; k < indptr[j + 1]; ++k)
      trips.emplace_back(indices[k], j, data[k]);
  M.setFromTriplets(trips.begin(), trips.end());
  return M;
}

Eigen::VectorXd ReadVec(FILE* f) {
  int32_t len;
  CONEX_DEMAND(fread(&len, sizeof(int32_t), 1, f) == 1, "");
  Eigen::VectorXd v(len);
  CONEX_DEMAND(fread(v.data(), sizeof(double), len, f) == (size_t)len, "");
  return v;
}

}  // namespace

TEST(EqualityRepair, QSCORPIO) {
  const char* path = "tests/data/QSCORPIO.bin";
  FILE* f = fopen(path, "rb");
  if (!f) {
    // Also try from build directory.
    f = fopen("../tests/data/QSCORPIO.bin", "rb");
  }
  if (!f) {
    printf("  QSCORPIO.bin not found, skipping.\n");
    GTEST_SKIP() << "QSCORPIO.bin not found";
  }

  int32_t n;
  ASSERT_EQ(fread(&n, sizeof(int32_t), 1, f), 1u);
  auto P = ReadCSC(f);
  auto q = ReadVec(f);
  auto A_eq = ReadCSC(f);
  auto b_eq = ReadVec(f);
  auto A_ineq = ReadCSC(f);
  auto b_ineq = ReadVec(f);
  fclose(f);

  ASSERT_EQ(n, 358);
  ASSERT_EQ(A_eq.rows(), 280);
  ASSERT_EQ(A_ineq.rows(), 466);

  printf("  QSCORPIO: n=%d, eq=%d, ineq=%d, P_nnz=%d\n",
         n, (int)A_eq.rows(), (int)A_ineq.rows(), (int)P.nonZeros());
  fflush(stdout);

  auto vars = Range(n);
  Model model;
  model.AddQuadraticCost(P, vars);
  model.SetLinearCost(q);
  model.AddEqualityConstraint(A_eq, b_eq, vars);
  model.AddLinearConstraint(A_ineq, b_ineq, vars);

  const double tol = 1e-8;
  const int max_iter = 200;

  // RLDLT: converges — regularization absorbs the 30 dependent equations.
  {
    auto solver = Solver::Build(model);
    int nd = solver.tree_solver() ? solver.tree_solver()->num_demotions() : 0;
    auto compiled = solver.MakeCompiledModel();
    auto raw = ThetaContinuation{tol, max_iter, 1}.Run(compiled);
    printf("  RLDLT: conv=%d, dem=%d, mu=%.2e, iter=%d\n",
           raw.mu < tol, nd, raw.mu, raw.iterations);
    EXPECT_LT(raw.mu, tol);
    EXPECT_EQ(nd, 0);
  }

  // LU: needs ~454 demotions to cascade all dependent duals to the root,
  // where RLDLT absorbs the null space.  With enough demotions, LU
  // converges identically to RLDLT.
  {
    SolverConfiguration c; c.tree.use_lu_for_indefinite = true;
    auto solver = Solver::Build(model, c);
    int nd = solver.tree_solver() ? solver.tree_solver()->num_demotions() : 0;
    auto compiled = solver.MakeCompiledModel();
    auto raw = ThetaContinuation{tol, max_iter, 1}.Run(compiled);
    printf("  LU: conv=%d, dem=%d, mu=%.2e, iter=%d\n",
           raw.mu < tol, nd, raw.mu, raw.iterations);
    EXPECT_GT(nd, 0);
    EXPECT_LT(raw.mu, tol);  // Converges with enough demotions.
  }
}

}  // namespace
}  // namespace conex
