#include "conex/cone_program.h"
#include "conex/constraint.h"
#include "conex/dense_lmi_constraint.h"
#include "conex/linear_constraint.h"
#include "conex/test/test_util.h"
#include <Eigen/Dense>

using DenseMatrix = Eigen::MatrixXd;

namespace conex {
void TestSDP(int i) {
  SolverConfiguration config;
  int n = 300;
  int m = 50;
  auto constraints2 = GetRandomDenseMatrices(n, m);

  DenseMatrix affine2 = Eigen::MatrixXd::Identity(n, n);
  DenseLMIConstraint LMI{n, constraints2, affine2};

  Program prog(m);
  DenseMatrix y(m, 1);
  prog.AddConstraint(LMI);

  auto b = GetFeasibleObjective(&prog);
  Solve(b, prog, config, y.data());
}
}  // namespace conex

int main() {
  for (int i = 0; i < 1; i++) {
    conex::TestSDP(i);
  }
  return 0;
}
