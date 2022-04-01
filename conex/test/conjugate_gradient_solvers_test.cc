#include "conex/conjugate_gradient_solvers.h"
#include "gtest/gtest.h"

#include "conex/constraint_manager.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

struct BlockSparseMatrix {
  std::vector<std::vector<int>> non_zero_columns;
  std::vector<std::vector<double>> entries;
};

BlockSparseMatrix MakeBlockSparseMatrix(const Eigen::MatrixXd& B) {
  BlockSparseMatrix matrix;
  matrix.non_zero_columns.resize(B.rows());
  matrix.entries.resize(B.rows());
  for (int i = 0; i < B.rows(); i++) {
    for (int j = 0; j < B.cols(); j++) {
      if (B(i, j) != 0) {
        matrix.non_zero_columns.at(i).push_back(j);
        matrix.entries.at(i).push_back(B(i, j));
      }
    }
  }
  return matrix;
}

GTEST_TEST(ConjugateGradient, TrivalExample) {
  int num_vars = 4;
  int num_eq = 2;
  ConstraintManager prog;
  prog.SetNumberOfVariables(num_vars);
  MatrixXd Q1(2, 2);
  MatrixXd Q2(2, 2);
  // clang-format off
  Q1 << 5, 2, 
        2, 1;
  Q2 << 5, 2, 
        2, 1;
  // clang-format on
  prog.AddQuadraticCost(Q1, {0, 1});
  prog.AddQuadraticCost(Q2, {2, 3});
  prog.InitializeWorkspace();

  MatrixXd B(num_eq, num_vars);
  // clang-format off
  B << 1, 0, 0, 1, 
       0, 1, 1, 0;
  // clang-format on

  BlockSparseMatrix B_sparse = MakeBlockSparseMatrix(B);
  ConstrainedLeastSquaresConjugateGradientSolver solver(
      prog.cliques, prog.CliqueAssemblers(), B_sparse.non_zero_columns,
      B_sparse.entries);

  VectorXd f(num_vars);
  f.setLinSpaced(4, -10, 10);
  VectorXd g(num_eq);
  g.setLinSpaced(num_eq, -2, 2);
  solver.Assemble();
  MatrixXd M = solver.KKTMatrix();
  MatrixXd M_ref(M.rows(), M.cols());
  M_ref.setZero();
  M_ref.block(0, 0, 2, 2) = Q1;
  M_ref.block(2, 2, 2, 2) = Q2;
  M_ref.bottomLeftCorner(num_eq, num_vars) = B;
  M_ref.topRightCorner(num_vars, num_eq) = B.transpose();
  EXPECT_NEAR((M - M_ref).norm(), 0, 1e-14);

  solver.Factor();
  VectorXd y, z;
  for (int i = 0; i < 2; i++) {
    solver.Solve(f, g, &y, &z, i == 0 /*true: llt, false: cg*/);
    VectorXd calc = (M.leftCols(num_vars) * y + M.rightCols(num_eq) * z);
    EXPECT_NEAR((calc.topRows(f.rows()) - f).norm(), 0, 1e-9);
    EXPECT_NEAR((calc.bottomRows(g.rows()) - g).norm(), 0, 1e-9);
  }
}

}  // namespace conex
