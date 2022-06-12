#include <iostream>
#include <memory>

#include "conex/cone_program.h"
#include "conex/constraint.h"
#include "conex/dense_lmi_constraint.h"
#include "conex/quadratic_cone_constraint.h"
#include "conex/soc_constraint.h"
#include "conex/test/default_solver_config.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using DenseMatrix = Eigen::MatrixXd;

int DoMain() {
  int n = 3;
  SolverConfiguration config = DefaultTestConfiguration();
  config.inv_sqrt_mu_max = 10000;

  std::vector<Eigen::MatrixXd> A;
  // 1 x1 x3 x3
  // x1 1
  // x2   1
  // x3     1
  DenseMatrix Wsqrt = Eigen::MatrixXd::Random(n, n);

  Eigen::MatrixXd C(n + 1, n + 1);
  C.setIdentity();
  for (int i = 1; i < n + 1; i++) {
    Eigen::MatrixXd Ai(n + 1, n + 1);
    Ai.setZero();
    Ai.bottomLeftCorner(n, 1) = Wsqrt.col(i - 1);
    Ai.topRightCorner(1, n) = Wsqrt.col(i - 1).transpose();
    A.push_back(Ai);
  }
  DenseLMIConstraint lmi_constraint{n + 1, A, C};

  Eigen::MatrixXd b(n, 1);

  DenseMatrix As(n + 1, n);
  As.setZero();
  As.bottomRightCorner(n, n) = Wsqrt;
  DenseMatrix Cs(n + 1, 1);
  Cs.setConstant(0.000);
  Cs(0) = 1;
  SOCConstraint soc_constraint_with_squareroot(As, Cs);

  QuadraticConstraint quad_constraint_with_squareroot(As, Cs);

  DenseMatrix Q = Wsqrt.transpose() * Wsqrt;
  DenseMatrix Aq(n + 1, n);
  Aq.setZero();
  Aq.bottomRightCorner(n, n).setIdentity();
  QuadraticConstraint quad_constraint(Q, Aq, Cs);

  for (int i = -2; i < 2; i++) {
    b.setConstant(i);
    b += Eigen::VectorXd::Random(n, 1) * .02;

    Program prog1(n);
    prog1.AddConstraint(soc_constraint_with_squareroot);

    DenseMatrix y1(n, 1);
    config.enable_line_search = 0;
    Solve(b, prog1, config, y1.data());

    DenseMatrix y1_line_search(n, 1);
    config.enable_line_search = 1;
    Solve(b, prog1, config, y1_line_search.data());
    EXPECT_NEAR((y1 - y1_line_search).norm(), 0, 1e-6);

    Program prog_quad(n);
    prog_quad.AddConstraint(quad_constraint);

    DenseMatrix y_quad(n, 1);
    config.enable_line_search = 0;
    Solve(b, prog_quad, config, y_quad.data());
    EXPECT_NEAR((y1 - y_quad).norm(), 0, 8e-6);

    DenseMatrix y_quad_line_search(n, 1);
    config.enable_line_search = 1;
    Solve(b, prog_quad, config, y_quad_line_search.data());
    EXPECT_NEAR((y_quad_line_search - y1_line_search).norm(), 0, 8e-6);

    Program prog_lmi(n);
    config.enable_line_search = 0;
    prog_lmi.AddConstraint(lmi_constraint);
    DenseMatrix y_lmi(n, 1);
    Solve(b, prog_lmi, config, y_lmi.data());

    EXPECT_NEAR((y1 - y_lmi).norm(), 0, 1e-4);

    Program prog4(n);
    prog4.AddConstraint(quad_constraint_with_squareroot);
    DenseMatrix y4(n, 1);
    Solve(b, prog4, config, y4.data());
    EXPECT_NEAR((y1 - y4).norm(), 0, 8e-6);
  }

  return 0;
}

GTEST_TEST(Constraints, SOCP) {
  srand(1);
  for (int i = 0; i < 10; i++) {
    DoMain();
  }
}

}  // namespace conex
