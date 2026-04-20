#include <cstdio>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/tree_solver/kkt_tree_solver.h"
using namespace conex;

int main() {
  // QP+eq: min [1,-2,0]'x + (1/2)x'Qx  s.t.  x >= 0, x0+x1+x2 = 2
  // Q = [[4,1,0],[1,4,0],[0,0,2]]
  Model p;
  Eigen::SparseMatrix<double> A(3, 3); A.setIdentity();
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);
  std::vector<int> v = {0, 1, 2};
  p.AddLinearConstraint(A, b, v);

  Eigen::SparseMatrix<double> C(1, 3);
  C.insert(0, 0) = 1; C.insert(0, 1) = 1; C.insert(0, 2) = 1;
  C.makeCompressed();
  Eigen::VectorXd d(1); d(0) = 2.0;
  p.AddEqualityConstraint(C, d, v);

  Eigen::SparseMatrix<double> Q(3, 3);
  Q.insert(0, 0) = 4; Q.insert(0, 1) = 1; Q.insert(1, 0) = 1;
  Q.insert(1, 1) = 4; Q.insert(2, 2) = 2;
  Q.makeCompressed();
  p.AddQuadraticCost(Q, v);

  Eigen::VectorXd c(3); c << 1, -2, 0;
  p.SetLinearCost(c);

  auto solver = Solver::Build(p);
  auto* kkt = solver.kkt();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  kkt->SetScaling(W); kkt->AssembleAndFactor();

  auto r = SolveGeodesicThetaContinuation(*kkt, cost_rhs, W, 200, 1, 1e-8, false);

  int nv = kkt->number_of_variables();  // 3 primal + 1 dual = 4
  int np = 3;  // primal vars
  printf("nv=%d, x.size=%d\n", nv, (int)r.x.size());

  Eigen::VectorXd x = r.x.head(np);
  double nu = (r.x.size() > np) ? r.x(np) : 0.0;
  printf("x = [%.8f, %.8f, %.8f]\n", x(0), x(1), x(2));
  printf("nu = %.8f\n", nu);
  printf("Cx - d = %.8e\n", (C * x - d)(0));

  // Recover lambda from the final W.
  // s = Ax + b = x (since A=I, b=0 for this problem).
  Eigen::VectorXd s = x;
  printf("s = [%.8f, %.8f, %.8f]\n", s(0), s(1), s(2));

  // Stationarity: c + Qx = A'lambda + C'nu
  // With A=I: lambda = c + Qx - C'nu
  Eigen::VectorXd Qx = Eigen::VectorXd::Zero(np);
  for (int k2 = 0; k2 < Q.outerSize(); ++k2)
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k2); it; ++it)
      Qx(it.row()) += it.value() * x(it.col());

  Eigen::VectorXd Cnu = C.transpose() * Eigen::VectorXd::Constant(1, nu);
  Eigen::VectorXd lambda = c + Qx - Cnu;  // A=I so A'lam = lam
  printf("Qx = [%.8f, %.8f, %.8f]\n", Qx(0), Qx(1), Qx(2));
  printf("lambda = c + Qx - C'nu = [%.8f, %.8f, %.8f]\n",
         lambda(0), lambda(1), lambda(2));

  // Complementarity
  double s_dot_lam = x.dot(lambda);  // s = x for this problem
  printf("\n<s, lambda> = %.8e\n", s_dot_lam);

  // Now evaluate the two candidate gap expressions:
  double xQx = x.dot(Qx);
  double cTx = c.dot(x);
  double bTlam = b.dot(lambda);  // b=0, so this is 0
  // Actually b is the cone affine term. For x >= 0 with A=I, b=0.
  // But lambda here is the cone dual, and b'lambda = 0'lambda = 0.
  // We need b'lambda from the cone: dot(b_cone, lambda_cone).
  // Since b_cone = 0, b'lambda = 0.
  double dTnu = d(0) * nu;

  printf("\nComponents:\n");
  printf("  x'Qx   = %.8e\n", xQx);
  printf("  c'x    = %.8e\n", cTx);
  printf("  b'lam  = %.8e  (b=0 for this problem)\n", bTlam);
  printf("  d'nu   = %.8e\n", dTnu);
  printf("  -d'nu  = %.8e\n", -dTnu);

  printf("\nCandidate gaps:\n");
  double gap_plus  = xQx + cTx + bTlam + dTnu;
  double gap_minus = xQx + cTx + bTlam - dTnu;
  printf("  x'Qx + c'x + b'lam + d'nu = %.8e\n", gap_plus);
  printf("  x'Qx + c'x + b'lam - d'nu = %.8e\n", gap_minus);
  printf("  <s, lambda>                = %.8e\n", s_dot_lam);

  printf("\nWhich equals <s, lambda>?\n");
  printf("  |+d'nu - <s,lam>| = %.2e\n", std::abs(gap_plus - s_dot_lam));
  printf("  |-d'nu - <s,lam>| = %.2e\n", std::abs(gap_minus - s_dot_lam));
}
