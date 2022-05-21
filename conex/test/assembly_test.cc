#include <any>
#include <chrono>

#include "conex/block_triangular_operations.h"
#include "conex/clique_ordering.h"
#include "conex/constraint_manager.h"
#include "conex/debug_macros.h"
#include "conex/equality_constraint.h"
#include "conex/kkt_solver.h"
#include "conex/kkt_solver_factory.h"
#include "conex/supernodal_assembler.h"
#include "conex/supernodal_solver.h"
#include "gtest/gtest.h"

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void BuildLQRProblem(int N, ConstraintManager* prg) {
  auto& prog = *prg;
  Eigen::MatrixXd Qi = Eigen::MatrixXd::Identity(3, 3) * 2;

  MatrixXd A0(2, 3);
  MatrixXd Ai(2, 5);
  MatrixXd bi(2, 1);
  bi << 1, 2;

  // clang-format off
  A0 << 1, 1, 0,
        1, 0, 1;

  Ai << 1, 1, 1, 1, 0,
        1, 1, 1, 0, 1;
  // clang-format on

  int max_var = (N + 1) * (2 + 1);

  prog.SetNumberOfVariables(max_var);
  prog.AddEqualityConstraint(EqualityConstraints{A0, bi}, vector{0, 1, 2});

  int o = 0;
  for (int i = 0; i < N; i++) {
    vector vars{1 + o, 2 + o, 3 + o, 4 + o, 5 + o};
    o += 3;
    prog.AddEqualityConstraint(EqualityConstraints{Ai * (i + 2), bi * (i + 2)},
                               vars);
  }
  prog.AddQuadraticCost(Qi, vector{0, 1, 2});

  o = 3;
  for (int i = 0; i < N; i++) {
    vector vars{o, 1 + o, 2 + o};
    o += 3;
    prog.AddQuadraticCost(Qi, vars);
  }
  prog.InitializeWorkspace();
}

GTEST_TEST(LDLT, TestAssembly) {
  using Eigen::MatrixXd;
  constexpr int m = 6;
  constexpr int n = 9;
  Eigen::MatrixXd A(m, n);
  // clang-format off
  //   0  1  2  3  4  5  6  7  8
  A << 1, 1, 0, 0, 0, 0, 0, 0, 0,
       1, 0, 1, 0, 0, 0, 0, 0, 0,
       0, 2, 2, 2, 2, 0, 0, 0, 0,
       0, 2, 2, 2, 0, 2, 0, 0, 0,
       0, 0, 0, 0, 3, 3, 3, 3, 0,
       0, 0, 0, 0, 3, 3, 3, 0, 3;

  Eigen::VectorXd b(n+m);
  b.setZero();
  b.bottomRows(m) << 1, 2, 
                     2, 4,
                     3, 6;
  // clang-format on

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n) * 2;
  Eigen::MatrixXd Qi = Eigen::MatrixXd::Identity(3, 3) * 2;

  int N = 2;

  ConstraintManager prog;
  BuildLQRProblem(N, &prog);
  std::unique_ptr<KKTSolverBase> solver_ptr =
      KKTSolverFactory().create_unique(&prog, SolverConfiguration());

  auto& solver = *solver_ptr;
  solver.Assemble();

  EXPECT_EQ(n + m, prog.SizeOfKKTSystem());
  MatrixXd T(n + m, n + m);
  T.setZero();
  T.block(0, 0, n, n) = Q;
  T.block(0, n, n, m) = A.transpose();
  T.block(n, 0, m, n) = A;

  using Eigen::VectorXd;
  Eigen::LDLT<MatrixXd> ldlt;
  VectorXd yref;
  ldlt.compute(T);

  solver.Assemble();

  MatrixXd error = (solver.KKTMatrix() - T);

  EXPECT_EQ(error.norm(), 0);

  solver.Factor();
  for (int i = 0; i < 3; i++) {
    yref = ldlt.solve(b);
    VectorXd y = solver.Solve(b);
    EXPECT_NEAR((y - yref).norm(), 0, 1e-9);
    b = y;
  }
}

GTEST_TEST(LDLT, Benchmark2) {
  using Eigen::MatrixXd;

  int N = 40;

  ConstraintManager prog;
  BuildLQRProblem(N, &prog);

  std::unique_ptr<KKTSolverBase> solver_ptr =
      KKTSolverFactory().create_unique(&prog, SolverConfiguration());
  auto& solver = *solver_ptr;

  solver.Assemble();
  Eigen::MatrixXd T = solver.KKTMatrix().selfadjointView<Eigen::Lower>();

  Eigen::VectorXd b(prog.SizeOfKKTSystem());
  b.setConstant(1);
  solver.Factor();
  for (int i = 0; i < 3; i++) {
    Eigen::VectorXd y = solver.Solve(b);
    EXPECT_NEAR((T * y - b).norm(), 0, 1e-9);
  }
}

GTEST_TEST(Assemble, VariablesSpecifiedOutOfOrder) {
  MatrixXd Q = MatrixXd::Identity(3, 3);

  ConstraintManager prog;
  prog.SetNumberOfVariables(4);
  // clang-format off
  Q << 1, 0, 0,
       0, 0, 0,
       0, 0, 3;
  // clang-format on

  prog.AddQuadraticCost(Q, vector{1, 0, 3});
  // clang-format off
  Q << 1, 0, 0,
       0, 0, 0,
       0, 0, 2;
  // clang-format on
  prog.AddQuadraticCost(Q, vector{1, 0, 2});

  prog.InitializeWorkspace();
  std::unique_ptr<KKTSolverBase> solver_ptr =
      KKTSolverFactory().create_unique(&prog, SolverConfiguration());
  auto& solver = *solver_ptr;

  solver.Assemble();
  auto M = solver.KKTMatrix();
  Eigen::VectorXd expected(4);
  expected << 0, 2, 2, 3;
  EXPECT_EQ((Eigen::MatrixXd(expected.asDiagonal()) - M).norm(), 0);
}
#if 0

void ModifyCliquesForEqualities(ConstraintManager& prog) {
  std::vector<std::vector<int>> cliques;
  std::vector<std::vector<int>> dual_vars =
      prog.equality_constraint_multipliers();
  for (auto& e : prog.clique_assemblers()) {
    if (e->is_positive_definite()) {
      cliques.push_back(e->variables());
      int j = 0;
      for (auto& f : prog.clique_assemblers()) {
        std::vector<int> intersection;
        IntersectionOfSorted(e->variables(), f->variables(), &intersection);
        if (intersection.size() > 0) {
          for (auto lambda : dual_vars.at(j)) {
            cliques.back().push_back(-lambda);
          }
          std::sort(cliques.back().begin(), cliques.back().end());
        }
        j++;
      }
    }
  }
  vector<std::vector<int>> supernodes(cliques.size());
  vector<std::vector<int>> separators(cliques.size());
  std::vector<int> tree(cliques.size());
  std::vector<int> order(cliques.size());
  PickCliqueOrder(cliques, 1, &order, &tree, &supernodes, &separators);
  DUMP(cliques);
  DUMP(supernodes);
  DUMP(separators);
  DUMP(tree);
}

GTEST_TEST(EqualityConstraints, TestElimination) {
  int N = 5;
  ConstraintManager prog;
  BuildLQRProblem(N, &prog);

  // ModifyCliquesForEqualities(prog);

  MatrixXd Q(2, 2);
  ConstraintManager prog2(3);
  prog2.AddQuadraticCost(Q, {0, 2});
  prog2.AddQuadraticCost(Q, {1, 2});
  Eigen::MatrixXd A(1, 1);
  A << 1;
  Eigen::MatrixXd b(1, 1);
  b << 0;
  prog2.AddEqualityConstraint(EqualityConstraints(A, b), {2});
  ModifyCliquesForEqualities(prog2);
}

//
//
GTEST_TEST(Simple, TestElimination) {
  MatrixXd Q(2, 2);
  Q.setConstant(3);
  ConstraintManager prog(6);
  prog.AddQuadraticCost(Q, {0, 1});
  prog.AddQuadraticCost(Q, {2, 3});
  prog.AddQuadraticCost(Q, {4, 5});
  Eigen::MatrixXd A(2, 4);
  A << 1, 1, 1, 2, 2, 3, 4, 5;
  Eigen::MatrixXd b(2, 1);
  b << 1, 1;
  prog.AddEqualityConstraint(EqualityConstraints(A, b), {0, 1, 2, 3});
  prog.AddEqualityConstraint(EqualityConstraints(A, b), {2, 3, 4, 5});
  prog.InitializeWorkspace();

  auto solver = KKTSolverFactory().create_unique(&prog, SolverConfiguration());
  solver->Assemble();
  DUMP(solver->KKTMatrix());
  ModifyCliquesForEqualities(prog);

  // Want clique tree: (0, 1,

  // l1 x2
  // l1 x1
  // x1 x2
  //
  // * *
  // * *
  //
  // * *
}
#endif

}  // namespace conex
