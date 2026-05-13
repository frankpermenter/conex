#include <gtest/gtest.h>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/conex.h"
#include "conex/common/extended_embedding.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/kkt_solver_dense.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

TEST(TreeStructure, DiagonalQPWithOneEquality) {
  // min (1/2) x'Qx  s.t.  x >= 0,  e'x = 1
  // Q = I (diagonal), one equality constraint touching all variables.
  // The tree should have structure from the equality constraint.
  const int n = 10;

  // Linear constraint: Ix + 0 >= 0  (i.e. x >= 0).
  Eigen::SparseMatrix<double> A(n, n);
  A.setIdentity();
  VectorXd b = VectorXd::Zero(n);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // Quadratic cost: Q = I.
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setIdentity();

  // Equality: e'x = 1 (1 row, all n columns).
  Eigen::SparseMatrix<double> C(1, n);
  for (int i = 0; i < n; i++) C.insert(0, i) = 1.0;
  VectorXd d(1);
  d << 1.0;

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.AddQuadraticCost(Q, vars);
  model.AddEqualityConstraint(C, d, vars);

  // Default config.
  {
    auto solver = Solver::Build(model);
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(solver.kkt());
    ASSERT_NE(ts, nullptr);
    int ns = ts->num_subsystems();
    printf("  default: %d cliques, %d vars\n", ns, ts->number_of_variables());
    for (int k = 0; k < ns; k++)
      printf("    clique %d: size=%d\n", k, ts->clique_size(k));
  }

  // No merging + LU: should produce star tree.
  {
    SolverConfiguration cfg;
    cfg.tree.max_merge_supernode_size = 0;
    cfg.tree.use_lu_for_indefinite = true;
    auto solver = Solver::Build(model, cfg);
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(solver.kkt());
    ASSERT_NE(ts, nullptr);
    int ns = ts->num_subsystems();
    printf("  max_merge=0 + LU: %d cliques, %d vars\n", ns, ts->number_of_variables());
    for (int k = 0; k < ns; k++)
      printf("    clique %d: size=%d\n", k, ts->clique_size(k));
    EXPECT_GT(ns, 1) << "Star tree should have multiple cliques";

    auto result = solver.Solve(GeodesicLP{1e-10, 20, 0, true});
    printf("  obj=%.6e  x[0]=%.4f  sum(x)=%.4f\n",
           result.objective, result.x[0], result.x.sum());
    EXPECT_NEAR(result.x.sum(), 1.0, 1e-6);
  }
}

TEST(TreeStructure, DiagonalQPWithTwoEqualities) {
  // min (1/2) x'Qx  s.t.  x >= 0,  C1*x = d1,  C2*x = d2
  // C1 touches x[0..4], C2 touches x[5..9]. Disjoint supports.
  // The tree should split into 2+ cliques.
  const int n = 10;

  Eigen::SparseMatrix<double> A(n, n);
  A.setIdentity();
  VectorXd b = VectorXd::Zero(n);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Eigen::SparseMatrix<double> Q(n, n);
  Q.setIdentity();

  // Equality 1: x0 + x1 + x2 + x3 + x4 = 1.
  Eigen::SparseMatrix<double> C1(1, n);
  for (int i = 0; i < 5; i++) C1.insert(0, i) = 1.0;
  VectorXd d1(1); d1 << 1.0;

  // Equality 2: x5 + x6 + x7 + x8 + x9 = 1.
  Eigen::SparseMatrix<double> C2(1, n);
  for (int i = 5; i < 10; i++) C2.insert(0, i) = 1.0;
  VectorXd d2(1); d2 << 1.0;

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.AddQuadraticCost(Q, vars);
  model.AddEqualityConstraint(C1, d1, vars);
  model.AddEqualityConstraint(C2, d2, vars);

  auto solver = Solver::Build(model);
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(solver.kkt());
  ASSERT_NE(ts, nullptr);

  int ns = ts->num_subsystems();
  int nv = ts->number_of_variables();
  printf("  n=%d: %d cliques, %d variables (n + 2 duals = %d)\n",
         n, ns, nv, n + 2);
  for (int k = 0; k < ns; k++) {
    printf("    clique %d: size=%d\n", k, ts->clique_size(k));
  }

  auto result = solver.Solve(ThetaContinuation{1e-10});
  printf("  obj=%.6e  sum(x[0:5])=%.4f  sum(x[5:10])=%.4f\n",
         result.objective,
         Eigen::Map<const Eigen::VectorXd>(result.x.data(), result.x.size()).head(5).sum(), result.x.tail(5).sum());
  EXPECT_NEAR(Eigen::Map<const Eigen::VectorXd>(result.x.data(), result.x.size()).head(5).sum(), 1.0, 1e-6);
  EXPECT_NEAR(result.x.tail(5).sum(), 1.0, 1e-6);
}

TEST(TreeStructure, DiagonalQPWithChainEqualities) {
  // min (1/2) x'Qx  s.t.  x >= 0,  x[i] + x[i+1] = 1 for i=0,2,4,...
  // Each equality touches 2 adjacent variables. Chain structure.
  const int n = 10;

  Eigen::SparseMatrix<double> A(n, n);
  A.setIdentity();
  VectorXd b = VectorXd::Zero(n);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Eigen::SparseMatrix<double> Q(n, n);
  Q.setIdentity();

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.AddQuadraticCost(Q, vars);

  // Add 5 equality constraints: x0+x1=1, x2+x3=1, ..., x8+x9=1.
  for (int i = 0; i < n; i += 2) {
    Eigen::SparseMatrix<double> C(1, n);
    C.insert(0, i) = 1.0;
    C.insert(0, i + 1) = 1.0;
    VectorXd d(1); d << 1.0;
    model.AddEqualityConstraint(C, d, vars);
  }

  auto solver = Solver::Build(model);
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(solver.kkt());
  ASSERT_NE(ts, nullptr);

  int ns = ts->num_subsystems();
  int nv = ts->number_of_variables();
  printf("  n=%d: %d cliques, %d variables (n + 5 duals = %d)\n",
         n, ns, nv, n + 5);
  for (int k = 0; k < ns; k++) {
    printf("    clique %d: size=%d\n", k, ts->clique_size(k));
  }

  auto result = solver.Solve(ThetaContinuation{1e-10});
  printf("  obj=%.6e\n", result.objective);
  for (int i = 0; i < n; i += 2) {
    double pair_sum = result.x[i] + result.x(i + 1);
    printf("    x[%d]+x[%d]=%.4f\n", i, i + 1, pair_sum);
    EXPECT_NEAR(pair_sum, 1.0, 1e-6);
  }
}

// Verify AccumulateCtranspose and EqualityAffineTermRHS produce the
// same result on the star tree (split equality) as on the dense solver
// (single clique, no split).
TEST(TreeStructure, AccumulateCtransposeStarTree) {
  const int n = 6;

  Eigen::SparseMatrix<double> A(n, n);
  A.setIdentity();
  VectorXd b = VectorXd::Zero(n);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Eigen::SparseMatrix<double> Q(n, n);
  Q.setIdentity();

  // One dense equality: c'x = 1 with random c.
  srand(42);
  VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);
  Eigen::SparseMatrix<double> C(1, n);
  for (int i = 0; i < n; i++) C.insert(0, i) = c(i);
  VectorXd d(1);
  d << 1.0;

  Model model;
  model.AddLinearConstraint(A, b, vars);
  model.AddQuadraticCost(Q, vars);
  model.AddEqualityConstraint(C, d, vars);

  // Dense solver (1 clique, reference).
  auto solver_dense = Solver::BuildDense(model);
  auto* ts_dense = dynamic_cast<SymmetricLinearSystemTreeSolver*>(
      solver_dense.kkt());
  ASSERT_NE(ts_dense, nullptr);
  printf("  dense: %d cliques\n", ts_dense->num_subsystems());

  // Star tree solver (max_merge=0 + LU).
  SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = 0;
  cfg.tree.use_lu_for_indefinite = true;
  auto solver_star = Solver::Build(model, cfg);
  auto* ts_star = dynamic_cast<SymmetricLinearSystemTreeSolver*>(
      solver_star.kkt());
  ASSERT_NE(ts_star, nullptr);
  printf("  star: %d cliques\n", ts_star->num_subsystems());
  EXPECT_GT(ts_star->num_subsystems(), 1);

  int nv_dense = ts_dense->number_of_variables();
  int nv_star = ts_star->number_of_variables();
  ASSERT_EQ(nv_dense, nv_star) << "Variable count must match";
  printf("  nv=%d (n=%d + 1 dual)\n", nv_dense, n);

  // Factor both at W=I.
  ASSERT_TRUE(ts_dense->AssembleAndFactor());
  ASSERT_TRUE(ts_star->AssembleAndFactor());

  // --- Test 1: EqualityAffineTermRHS ---
  // Should produce the same vector d at the dual position.
  auto d_rhs_dense = ts_dense->EqualityAffineTermRHS();
  auto d_rhs_star = ts_star->EqualityAffineTermRHS();
  VectorXd d_vec_dense(nv_dense), d_vec_star(nv_star);
  d_rhs_dense.supernodes->GatherInto(d_vec_dense);
  d_rhs_star.supernodes->GatherInto(d_vec_star);
  double d_diff = (d_vec_dense - d_vec_star).norm();
  printf("  EqualityAffineTermRHS diff: %.2e\n", d_diff);
  EXPECT_LT(d_diff, 1e-14);

  // --- Test 2: AccumulateCtranspose ---
  // Given a random input [x; nu], both should produce the same [C'nu; Cx].
  VectorXd input = VectorXd::Random(nv_dense);

  auto in_dense = ts_dense->MakeSolverRHS();
  in_dense = MakeBlockVariable(*ts_dense, input);
  auto out_dense = ts_dense->MakeSolverRHS();
  out_dense.SetZero();
  ts_dense->AccumulateCtranspose(in_dense, out_dense);
  ts_dense->GatherSeparators(out_dense);
  VectorXd ct_dense(nv_dense);
  out_dense.supernodes->GatherInto(ct_dense);

  auto in_star = ts_star->MakeSolverRHS();
  in_star = MakeBlockVariable(*ts_star, input);
  auto out_star = ts_star->MakeSolverRHS();
  out_star.SetZero();
  ts_star->AccumulateCtranspose(in_star, out_star);
  ts_star->GatherSeparators(out_star);
  VectorXd ct_star(nv_star);
  out_star.supernodes->GatherInto(ct_star);

  double ct_diff = (ct_dense - ct_star).norm();
  printf("  AccumulateCtranspose diff: %.2e  (dense=%.2e, star=%.2e)\n",
         ct_diff, ct_dense.norm(), ct_star.norm());
  EXPECT_LT(ct_diff, 1e-14);

  // --- Test 3: Solve ---
  // Solve the same RHS with both and compare.
  VectorXd rhs_vec = VectorXd::Random(nv_dense);

  auto rhs_dense = ts_dense->MakeSolverRHS();
  rhs_dense = MakeBlockVariable(*ts_dense, rhs_vec);
  ts_dense->SolveSolverRHS(rhs_dense);
  VectorXd x_dense(nv_dense);
  rhs_dense.supernodes->GatherInto(x_dense);

  auto rhs_star = ts_star->MakeSolverRHS();
  rhs_star = MakeBlockVariable(*ts_star, rhs_vec);
  ts_star->SolveSolverRHS(rhs_star);
  VectorXd x_star(nv_star);
  rhs_star.supernodes->GatherInto(x_star);

  double solve_diff = (x_dense - x_star).norm();
  double solve_rel = solve_diff / std::max(x_dense.norm(), 1e-15);
  printf("  Solve diff: %.2e  rel=%.2e\n", solve_diff, solve_rel);
  EXPECT_LT(solve_rel, 1e-10);
}

TEST(TreeStructure, EmbeddingCustomTree) {
  // Verify the custom CliqueTree from BuildExtendedEmbedding produces
  // correct solves by comparing against the dense solver.
  srand(42);
  const int n = 3, m = 3;
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
  printf("  CliqueTree: %d cliques, RIP=%s\n",
         (int)emb_tree.supernodes.size(),
         emb_tree.CheckRunningIntersectionProperty() ? "ok" : "FAIL");
  ASSERT_TRUE(emb_tree.CheckRunningIntersectionProperty());

  // Dense solver (reference).
  auto solver_dense = Solver::BuildDense(emb_model);
  ASSERT_TRUE(solver_dense.kkt()->AssembleAndFactor());

  // Custom tree solver.
  auto solver_tree = Solver::Build(emb_model, emb_tree);
  ASSERT_TRUE(solver_tree.kkt()->AssembleAndFactor());

  int nv = solver_dense.kkt()->number_of_variables();
  printf("  nv=%d\n", nv);
  ASSERT_EQ(nv, solver_tree.kkt()->number_of_variables());

  // Compare solve on random RHS.
  VectorXd rhs_vec = VectorXd::Random(nv);

  auto rhs_d = solver_dense.kkt()->MakeSolverRHS();
  rhs_d = MakeBlockVariable(*solver_dense.kkt(), rhs_vec);
  solver_dense.kkt()->SolveSolverRHS(rhs_d);
  VectorXd x_dense(nv);
  rhs_d.supernodes->GatherInto(x_dense);

  auto rhs_t = solver_tree.kkt()->MakeSolverRHS();
  rhs_t = MakeBlockVariable(*solver_tree.kkt(), rhs_vec);
  solver_tree.kkt()->SolveSolverRHS(rhs_t);
  VectorXd x_tree(nv);
  rhs_t.supernodes->GatherInto(x_tree);

  double diff = (x_dense - x_tree).norm();
  double rel = diff / std::max(x_dense.norm(), 1e-15);
  printf("  Solve diff: %.2e  rel=%.2e\n", diff, rel);
  EXPECT_LT(rel, 1e-6);
}

}  // namespace
}  // namespace conex
