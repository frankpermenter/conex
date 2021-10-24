#include "logspace_interior_point_method.h"
#include <iostream>
#include "conex/debug_macros.h"

namespace {
using Eigen::MatrixXd;
using Eigen::VectorXd;

void BasicTest() {
  int m = 10;
  int n = 9;
  ProblemData data(m, n);
  data.A = MatrixXd::Random(m, n);
  data.W = MatrixXd::Random(n, n);
  // Refinement works
#if 0
  data.W = 0*data.W * data.W.transpose();
#else
  data.W = data.W * data.W.transpose();
#endif

  VectorXd ones(m);
  ones.setConstant(1);

  VectorXd x0 = VectorXd::Random(m);
  x0 = x0.cwiseProduct(x0);
  data.c = data.A.transpose() * x0;

  data.b = x0;

  SolverOptions options;
  options.enable_dual_correction = 1;
  options.enable_rescaling = 1;
  options.dinf_limit = 1.1 * sqrt(2);
  options.maximum_iterations = 18;
  LogspaceIPM(data, options);
}

void DoSimpleFeasibilityTest(const Eigen::VectorXd& b) {
  int m = 2;
  int n = 2;
  ProblemData data(m, n);
  // x + y >=  -k1;
  // x + y <=  k2;
  data.A << 1, 1, -1, -1,

      data.c << -1, 2;
  data.W << 0, 0, 0, 0;
  data.b = b;

  SolverOptions options;
  options.enable_dual_correction = 0;
  options.enable_rescaling = 0;
  options.dinf_limit = 1;
  options.maximum_iterations = 10;
  LogspaceIPM(data, options);
}

void DoFailLLT() {
  int m = 2;
  int n = 2;
  ProblemData data(m, n);
  // x + y >=  -k1;
  // x + y <=  k2;
  data.A << 1, 1, -1, -1,

      data.c << -1, 2;
  data.W << 0, 0, 0, 0;
  data.b << -1, 1;

  SolverOptions options;
  options.enable_dual_correction = 0;
  options.enable_rescaling = 0;
  options.dinf_limit = 0.99;
  options.maximum_iterations = 10;
  LogspaceIPM(data, options);
}

void DoUnboundedTest() {
  int m = 2;
  int n = 2;
  ProblemData data(m, n);
  // x + y >=  -k1;
  // x + y <=  k2;
  data.A << 1, 0, 0, 1,

      data.c << -1, -1;
  data.W << 0, 0, 0, 0;
  data.b << -1, -1;

  SolverOptions options;
  options.enable_dual_correction = 0;
  options.enable_rescaling = 0;
  options.dinf_limit = 0.99;
  options.maximum_iterations = 10;
  LogspaceIPM(data, options);
}

void SimpleFeasibilityTests() {
  Eigen::VectorXd b(2);

  // Infeas //
  DUMP("INFEAS");
  b << -2, 1;
  DoSimpleFeasibilityTest(b);

  DUMP("NO SLATER");
  // No Slater//
  b << -1, 1;
  DoSimpleFeasibilityTest(b);

  DUMP("STRICTLY FEAS");
  // Feeas //
  b << 2, -1;
  DoSimpleFeasibilityTest(b);
}

void WarmstartTests() {
  int num_ineqs = 1;
  int n = 2;
  ProblemData data(num_ineqs, n);
  // data.A = MatrixXd::Random(num_ineqs, n);
  //
  //
  // data.b = -data.A * MatrixXd::Random(n, 1);

  data.A << 1, 0;
  data.b << 0;
  // data.c = VectorXd::Random(n);
  data.c << 5, 0;
  data.W << MatrixXd::Identity(n, n);

  // Ax + b >== 0

  SolverOptions options;
  options.dinf_limit = .9;
  options.enable_rescaling = 1;
  DUMP("COLD");
  auto x0 = LogspaceIPM(data, options);
  DUMP(x0.lambda);
  DUMP(x0.expv);

  DUMP("WARM");
  data.c << -1, 0;
  LogspaceIPM(data, options, x0);
  return;

  DUMP("WARM");
  options.dinf_limit = .9;
  LogspaceIPM(data, options, VectorXd::Random(n));
  DUMP("WARM");
  LogspaceIPM(data, options, VectorXd::Random(n));

  for (int i = 0; i < 10; i++) {
    VectorXd x = data.W.inverse() * -data.c;
    DUMP(data.A * x + data.b);
    LogspaceIPM(data, options, x);
  }
}

// A x + b

void RandomTests() {
  int m = 7;
  int num_vars = 5;
  ProblemData data(m, num_vars);
  data.A = MatrixXd::Random(m, num_vars);
  data.c = VectorXd::Random(num_vars);
  data.W << 0.1 * MatrixXd::Identity(num_vars, num_vars);

  VectorXd e = MatrixXd::Random(m, 1);
  e.setConstant(1);
  data.b = e - data.A * MatrixXd::Random(num_vars, 1);
  // Ax + b >== 0

  SolverOptions options;
  options.dinf_limit = 1.00;
  options.enable_rescaling = 0;
  options.maximum_iterations = 20;
  DUMP("COLD");
  auto x0 = LogspaceIPM(data, options);
}

void RescalingTests() {
  int m = 3;
  int n = 5;
  ProblemData data(m, n);
  data.A = MatrixXd::Random(m, n);
  data.W << MatrixXd::Identity(n, n);

  VectorXd x0 = MatrixXd::Random(n, 1);

  VectorXd e = MatrixXd::Random(m, 1);
  e.setConstant(1);
  double noise = 0.1;
  VectorXd ep = e + noise * MatrixXd::Random(m, 1);
  ep = ep.array().abs();

  VectorXd eslack = e + noise * MatrixXd::Random(m, 1);
  eslack = eslack.array().abs();

  // Ax + b == e;
  data.b = eslack - data.A * x0;

  // A' e = Wx + c
  data.c = data.A.transpose() * ep - data.W * x0;

  SolverOptions options;
  DUMP("STARTING");
  options.dinf_limit = 1;
  options.maximum_iterations = 8;

  DUMP("RESCALE");
  options.enable_rescaling = 1;
  auto sol = LogspaceIPM(data, options);

  DUMP(data.A * sol.x + data.b);

  DUMP("NO RESCALE");
  options.enable_rescaling = 0;
  sol = LogspaceIPM(data, options);
  DUMP(data.A * sol.x + data.b);
}

struct Data {
  ProblemData data;
  Variable var;
  double sqrtmu;
};

Data DataFromActiveSet(int n, int num_ineqs, int size_of_active_set) {
  ProblemData data(num_ineqs, n);

  VectorXd optimal_slack(num_ineqs);
  optimal_slack.setZero();
  VectorXd optimal_lambda(num_ineqs);
  optimal_lambda.setZero();
  optimal_lambda.head(size_of_active_set) =
      VectorXd::Random(size_of_active_set).array().abs();
  optimal_slack.tail(num_ineqs - size_of_active_set) =
      VectorXd::Random(num_ineqs - size_of_active_set).array().abs();

  double sqrtmu = 5e-7;
  VectorXd optimal_expv(num_ineqs);
  optimal_expv.tail(num_ineqs - size_of_active_set) =
      2 * sqrtmu *
      optimal_slack.tail(num_ineqs - size_of_active_set).cwiseInverse();

  optimal_expv.head(size_of_active_set) =
      optimal_lambda.head(size_of_active_set) / (2 * sqrtmu);

  data.A = MatrixXd::Random(num_ineqs, n);
  data.b = optimal_slack;
  data.c = data.A.transpose() * optimal_lambda;
  data.W << MatrixXd::Zero(n, n);

  Data output;
  output.data = data;
  output.var.expv = optimal_expv;
  output.var.sqrtmu = sqrtmu;
  output.var.lambda = optimal_lambda;
  output.var.slack = optimal_slack;
  return output;
}

double Divergence(const VectorXd& x, const VectorXd& y) {
  DUMP(x.cwiseProduct(y.cwiseInverse()));
  DUMP("HEHEH");
  return x.dot(y.cwiseInverse()) + y.dot(x.cwiseInverse()) - 2 * y.rows();
}

void ConstantActiveSet() {
  int num_ineqs = 5;
  int n = 3;
  int size_of_active_set = num_ineqs - n;
  auto stuff = DataFromActiveSet(n, num_ineqs, size_of_active_set);
  auto data = stuff.data;
  DUMP(data.A);
  DUMP(data.b);
  DUMP(data.c);

  // Two measures:
  // <z, s - s^*> + <x*, s - x*>
  //
  // <z, invw> + <invz, w>  - 2n
  //
  // <z, s - s^*> + <invz,

  SolverOptions options;
  options.dinf_limit = 1;
  options.enable_rescaling = 0;
  options.maximum_iterations = 12;
  DUMP("COLD");
  auto x0 = LogspaceIPM(data, options, stuff.var);
  // auto x0 = LogspaceIPM(data, options);
  DUMP(x0.expv);
  DUMP(stuff.var.lambda.cwiseQuotient(2 * x0.expv));

  DUMP(stuff.var.lambda - 2 * (.5 * stuff.var.sqrtmu) * x0.expv);
  DUMP(stuff.var.slack - 2 * (.5 * stuff.var.sqrtmu) * x0.expv.cwiseInverse());

  DUMP(stuff.var.slack.cwiseQuotient(2 * x0.expv.cwiseInverse()));
  // DUMP(x0.expv.array().log() -stuff.var.expv.array().log());

  // Shows that the divergence of the initial point
  // to the central path is n/2, which equals the lower
  // bound |d|^2_2/(1 + 1).
  // DUMP(Divergence(x0.expv, stuff.var.expv));
  // DUMP( x0.expv.rows()/(2.0));
  // This occurs because (2 + .5) * n - 2*n = .5 * n.
  // Goal:
  // Pick (v, mu) such that
  //
  //   1) divergence lower-bound is nearly tight
  //   Occurs if \hat v(mu) is close to optimal, since
  //
  //   Divergence( 2v  ,  v^* ) = 2 * n + 2 * n = 4*n - 2
  //

  // DUMP("WARM");
  // data = DataFromActiveSet(n, num_ineqs, size_of_active_set);
  // LogspaceIPM(data, options, x0);
  // data = DataFromActiveSet(n, num_ineqs, size_of_active_set);
  // LogspaceIPM(data, options, x0);
  // data = DataFromActiveSet(n, num_ineqs, size_of_active_set);
  // LogspaceIPM(data, options, x0);
  // data = DataFromActiveSet(n, num_ineqs, size_of_active_set);
  // LogspaceIPM(data, options, x0);
  return;

  DUMP("WARM");
  options.dinf_limit = .9;
  LogspaceIPM(data, options, VectorXd::Random(n));
  DUMP("WARM");
  LogspaceIPM(data, options, VectorXd::Random(n));

  for (int i = 0; i < 10; i++) {
    VectorXd x = data.W.inverse() * -data.c;
    DUMP(data.A * x + data.b);
    LogspaceIPM(data, options, x);
  }
}

void RandomLPTests() {
  int num_constraints = 5;
  VectorXd l0 = MatrixXd::Random(num_constraints, 1);
  l0 = l0.array().abs();

  VectorXd s0 = MatrixXd::Random(num_constraints, 1);
  s0 = s0.array().abs();

  int num_vars = 5;
  ProblemData data(num_constraints, num_vars);

  data.A = MatrixXd::Random(num_constraints, num_vars);
  data.c = data.A.transpose() * l0;
  data.b = s0;
  data.W.setZero();
  // Ax + b >== 0

  SolverOptions options;
  options.dinf_limit = 1.00;
  options.enable_rescaling = 0;
  options.maximum_iterations = 10;
  DUMP("COLD");
  auto x0 = LogspaceIPM(data, options);

  int num_eq = 4;
  data.B = MatrixXd::Random(num_eq, num_vars);
  data.d = data.B * x0.x;
  auto x1 = LogspaceIPM(data, options);
  DUMP(x1.x - x0.x);
}

}  // namespace

int main() {
  RandomLPTests();
  // SimpleFeasibilityTests();
  //  DoUnboundedTest();
  //  DoFailLLT();
  // WarmstartTests();
  //  ConstantActiveSet();
  // ConstantActiveSet();
  //  ConstantActiveSet();
  // for (int i = 0; i < 1; i++) {
  //  RandomTests();
  //  RescalingTests();
  //}
}
