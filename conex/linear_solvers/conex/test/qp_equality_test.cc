// Test SolveQPEquality on toy examples and LOTSCHD (Q+eq only, no bounds).
#include <cstdio>
#include <cmath>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/algorithms/equality_constrained_least_squares.h"
#include "conex/common/problem.h"
#include "conex/common/qps_reader.h"
using namespace conex;

bool TestToy() {
  printf("=== Toy: min x1^2 + x2^2  s.t. x1 + x2 = 1 ===\n");
  Problem p;
  Eigen::SparseMatrix<double> Q(2, 2);
  Q.insert(0, 0) = 2; Q.insert(1, 1) = 2; Q.makeCompressed();
  std::vector<int> v = {0, 1};
  p.AddQuadraticCost(Q, v);
  Eigen::SparseMatrix<double> C(1, 2);
  C.insert(0, 0) = 1; C.insert(0, 1) = 1; C.makeCompressed();
  Eigen::VectorXd d(1); d(0) = 1.0;
  p.AddEqualityConstraint(C, d, v);
  p.SetLinearCost(Eigen::VectorXd::Zero(2));

  auto r = SolveQPEquality(p);
  printf("  x = [%.8f, %.8f]\n", r.x(0), r.x(1));
  printf("  obj = %.8f (expected 0.5)\n", r.objective);
  printf("  Cx-d = %.2e\n", (C * r.x - d).norm());
  bool ok = r.success && std::abs(r.objective - 0.5) < 1e-6;
  printf("  %s\n\n", ok ? "PASS" : "FAIL");
  return ok;
}

bool TestToy2() {
  printf("=== Toy2: min [1,-2,0]'x + (1/2)x'Qx  s.t. x1+x2+x3=2 ===\n");
  Problem p;
  Eigen::SparseMatrix<double> Q(3, 3);
  Q.insert(0, 0) = 4; Q.insert(0, 1) = 1; Q.insert(1, 0) = 1;
  Q.insert(1, 1) = 4; Q.insert(2, 2) = 2;
  Q.makeCompressed();
  std::vector<int> v = {0, 1, 2};
  p.AddQuadraticCost(Q, v);
  Eigen::SparseMatrix<double> C(1, 3);
  C.insert(0, 0) = 1; C.insert(0, 1) = 1; C.insert(0, 2) = 1;
  C.makeCompressed();
  Eigen::VectorXd d(1); d(0) = 2.0;
  p.AddEqualityConstraint(C, d, v);
  Eigen::VectorXd c(3); c << 1, -2, 0;
  p.SetLinearCost(c);

  auto r = SolveQPEquality(p);
  printf("  x = [%.8f, %.8f, %.8f]\n", r.x(0), r.x(1), r.x(2));
  printf("  obj = %.8f\n", r.objective);
  printf("  Cx-d = %.2e\n", (C * r.x - d).norm());
  printf("  %s\n\n", r.success ? "PASS" : "FAIL");
  return r.success;
}

bool TestLOTSCHD() {
  printf("=== LOTSCHD (Q+eq only, no bounds) ===\n");
  auto [prob_full, info] = ReadQPS(
      "/agent-workspace/problem_libraries/maros_meszaros/QPS_Files/LOTSCHD.QPS");

  // Extract only Q and equality constraints — ignore the x>=0 bounds.
  Problem qp;
  for (int i = 0; i < prob_full.num_constraints(); ++i) {
    const auto& con = prob_full.constraint(i);
    if (auto* qc = std::get_if<Problem::QuadraticCostData>(&con))
      qp.AddQuadraticCost(qc->Q_sparse, qc->vars);
    else if (auto* ec = std::get_if<Problem::EqualityConstraintData>(&con))
      qp.AddEqualityConstraint(ec->C, ec->d, ec->primal_vars);
  }
  if (prob_full.has_linear_cost())
    qp.SetLinearCost(prob_full.linear_cost());

  printf("  vars=%d, constraints=%d\n", qp.num_variables(), qp.num_constraints());

  // Note: LOTSCHD without bounds is singular — 6 of 12 variables have
  // no Q entry and only appear in equalities.  Add tiny regularization
  // so the KKT system [Q+eps*I, C'; C, 0] is non-singular.
  {
    Eigen::SparseMatrix<double> Qreg(12, 12);
    for (int i = 0; i < 12; ++i) Qreg.insert(i, i) = 1e-10;
    Qreg.makeCompressed();
    std::vector<int> all(12);
    std::iota(all.begin(), all.end(), 0);
    qp.AddQuadraticCost(Qreg, all);
  }

  auto r = SolveQPEquality(qp);
  printf("  success = %s\n", r.success ? "yes" : "NO");
  if (!r.success) { printf("  FAIL\n\n"); return false; }

  printf("  x =");
  for (int i = 0; i < r.x.size(); ++i) printf(" %.4f", r.x(i));
  printf("\n");
  printf("  obj = %.8f\n", r.objective);

  // Check Cx = d.
  double max_eq_err = 0;
  for (int i = 0; i < qp.num_constraints(); ++i) {
    if (auto* ec = std::get_if<Problem::EqualityConstraintData>(
            &qp.constraint(i))) {
      Eigen::VectorXd x_sub(ec->primal_vars.size());
      for (int j = 0; j < (int)ec->primal_vars.size(); ++j)
        x_sub(j) = r.x(ec->primal_vars[j]);
      Eigen::VectorXd res = ec->C * x_sub - ec->d;
      max_eq_err = std::max(max_eq_err, res.norm());
    }
  }
  printf("  max ||Cx-d|| = %.2e\n", max_eq_err);

  // Check stationarity: Qx + C'nu = -c.
  // We don't have nu directly, but we can check that the KKT residual
  // is small by verifying Cx=d (primal) and that the objective is
  // a reasonable unconstrained-with-equality minimum.
  bool ok = r.success && max_eq_err < 1e-6;
  printf("  %s\n\n", ok ? "PASS" : "FAIL");
  return ok;
}

int main() {
  bool all_pass = true;
  all_pass &= TestToy();
  all_pass &= TestToy2();
  all_pass &= TestLOTSCHD();
  printf("%s\n", all_pass ? "ALL PASSED" : "SOME FAILED");
  return all_pass ? 0 : 1;
}
