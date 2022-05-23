#include "conex/cone_program.h"
#include "conex/equality_constraint.h"
#include "conex/linear_constraint.h"
#include "conex/test/default_solver_config.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;
GTEST_TEST(EqualityConstraints, Basic) {
  int num_vars = 3;
  int num_equalities = 1;
  int num_inequalities = 4;

  MatrixXd A = MatrixXd::Random(num_inequalities, num_vars);
  MatrixXd C(num_inequalities, 1);
  MatrixXd b(num_vars, 1);

  MatrixXd optimal_slack(num_inequalities, 1);
  MatrixXd optimal_dual(num_inequalities, 1);
  MatrixXd optimal_y(num_vars, 1);

  optimal_slack.setConstant(1);
  optimal_dual.setConstant(1);
  optimal_slack.topRows(num_inequalities * .5).setZero();
  optimal_dual.bottomRows(num_inequalities - num_inequalities * .5).setZero();
  optimal_y = Eigen::MatrixXd::Random(num_vars, 1);

  C = optimal_slack + A * optimal_y;

  LinearConstraint linear_inequality{A, C};

  MatrixXd eq = MatrixXd::Random(num_equalities, num_vars);
  MatrixXd eq_affine(num_equalities, 1);
  eq = Eigen::MatrixXd::Random(num_equalities, num_vars);
  eq_affine = eq * optimal_y;

  Program prog(num_vars);
  prog.AddConstraint(EqualityConstraints{eq, eq_affine});
  prog.AddConstraint(linear_inequality);

  VectorXd linear_cost(num_vars);
  linear_cost = A.transpose() * optimal_dual;

  VectorXd solution(num_vars);
  Solve(linear_cost, prog, DefaultTestConfiguration(), solution.data());

  EXPECT_NEAR((eq * solution - eq_affine).norm(), 0,
              DefaultEqualityConstraintTolerance());
  EXPECT_NEAR((solution - optimal_y).norm(), 0,
              DefaultEqualityConstraintTolerance());
}

void DoManySeparate(bool separate) {
  int num_vars = 10;
  int num_inequalities = num_vars + 10;
  int num_equalities = num_vars - 2;

  MatrixXd A = MatrixXd::Random(num_inequalities, num_vars);
  MatrixXd C(num_inequalities, 1);
  VectorXd feasible_slack(num_inequalities);
  VectorXd feasible_dual(num_inequalities);
  VectorXd feasible_y(num_vars);

  feasible_slack.setConstant(1);
  feasible_dual.setConstant(1);

  feasible_y = Eigen::MatrixXd::Random(num_vars, 1);

  C = feasible_slack + A * feasible_y;

  LinearConstraint linear_inequality{A, C};

  Program prog(num_vars);
  prog.AddConstraint(linear_inequality);

  MatrixXd eq = MatrixXd::Zero(num_equalities, num_vars);
  Eigen::MatrixXd Bi(1, 3);
  Bi << 1, 2, 3;
  MatrixXd eq_affine(num_equalities, 1);
  for (int i = 0; i < num_equalities; i++) {
    std::vector<int> vars{0, i + 1, num_vars - 1};
    for (size_t j = 0; j < vars.size(); j++) {
      eq(i, vars.at(j)) = Bi(0, j);
    }
    eq_affine(i) = eq.row(i) * feasible_y;
    if (separate) {
      prog.AddConstraint(EqualityConstraints{Bi, eq_affine.row(i)}, vars);
    }
  }

  if (!separate) {
    prog.AddConstraint(EqualityConstraints{eq, eq_affine});
  }

  VectorXd linear_cost(num_vars);
  linear_cost = A.transpose() * feasible_dual;

  VectorXd solution(num_vars);
  SolverConfiguration config = DefaultTestConfiguration();
  config.final_centering_steps = 10;
  config.initial_centering_steps_coldstart = 0;
  config.max_iterations = 40;
  config.divergence_upper_bound = .5;
  Solve(linear_cost, prog, config, solution.data());

  EXPECT_TRUE((C - A * solution).minCoeff() > -1e8);
  EXPECT_NEAR((eq * solution - eq_affine).norm(), 0, 5e-7);
  EXPECT_GE(linear_cost.dot(solution) + 1e-4, linear_cost.dot(feasible_y));
}
GTEST_TEST(EqualityConstraints, ManyConstraints) {
  for (int i = 0; i < 10; i++) {
    srand(i);
    DoManySeparate(false /*do not split constraints*/);
  }
}

GTEST_TEST(EqualityConstraints, ManySeparateConstraints) {
  for (int i = 0; i < 10; i++) {
    srand(i);
    DoManySeparate(true /* split constraints*/);
  }
}

}  // namespace conex
