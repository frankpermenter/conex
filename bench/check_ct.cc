// Check that AccumulateCtranspose has nontrivial contribution on passing examples.
#include <cstdio>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/kkt_solver_dense.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/linear_solvers/kkt_tree_solver.h"
using namespace conex;

void Check(const char* name, Model& prob, const Eigen::VectorXd& cost) {
  auto solver = Solver::Build(prob);
  auto* kkt = solver.kkt();
  auto* ts = solver.tree_solver();

  auto cost_rhs = kkt->MakeSolverRHS();
  if (prob.has_linear_cost())
    cost_rhs = MakeBlockVariable(*kkt, cost);

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  RowSpace b = kkt->GetAffineTerm();

  auto decomp = ComputeFullDecomposition(*kkt, cost_rhs, b, W);

  int nv = kkt->number_of_variables();

  // For each of y0, y1_0, y1_theta: compute AccumulateCtranspose contribution.
  auto check_col = [&](const Eigen::VectorXd& y_col, const char* col_name) {
    auto y_rhs = kkt->MakeSolverRHS();
    y_rhs = MakeBlockVariable(*kkt, y_col);

    // Ct contribution.
    auto ct_out = kkt->MakeSolverRHS();
    ct_out.SetZero();
    if (ts) ts->AccumulateCtranspose(y_rhs, ct_out);
    Eigen::VectorXd ct_vec(nv);
    ct_out.supernodes->GatherInto(ct_vec);
    double ct_norm = ct_vec.norm();

    // For comparison: A'W²A contribution.
    RowSpace Ay = kkt->MakeRowSpace();
    kkt->MultiplyA(y_rhs, Ay);
    RowSpace WAy = quadraticRepresentation(W, Ay);
    auto gram_out = kkt->MakeSolverRHS();
    gram_out.SetZero();
    kkt->AccumulateAtranspose(WAy, gram_out);
    Eigen::VectorXd gram_vec(nv);
    gram_out.supernodes->GatherInto(gram_vec);
    double gram_norm = gram_vec.norm();

    // Q contribution.
    auto q_out = kkt->MakeSolverRHS();
    q_out.SetZero();
    kkt->AccumulateQx(y_rhs, q_out);
    Eigen::VectorXd q_vec(nv);
    q_out.supernodes->GatherInto(q_vec);
    double q_norm = q_vec.norm();

    printf("  %s: ||Ct*y||=%.2e  ||A'W²Ay||=%.2e  ||Qy||=%.2e  ||y||=%.2e\n",
           col_name, ct_norm, gram_norm, q_norm, y_col.norm());
  };

  printf("%s (nv=%d, eq=%d):\n", name, nv,
         ts ? (int)ts->equality_sub_assemblers().size() : 0);
  check_col(decomp.y0, "y0");
  check_col(decomp.y1_0, "y1_0");
  check_col(decomp.y1_theta, "y1_theta");
  printf("\n");
}

int main() {
  // LP+eq (passing)
  {
    Model p;
    Eigen::SparseMatrix<double> A(4, 4); A.setIdentity();
    Eigen::VectorXd b = Eigen::VectorXd::Zero(4);
    std::vector<int> v = {0, 1, 2, 3};
    p.AddLinearConstraint(A, b, v);
    Eigen::SparseMatrix<double> C1(1, 2);
    C1.insert(0, 0) = 1; C1.insert(0, 1) = 1; C1.makeCompressed();
    Eigen::VectorXd d1(1); d1(0) = 1.0;
    std::vector<int> eq1 = {0, 1};
    p.AddEqualityConstraint(C1, d1, eq1);
    Eigen::VectorXd c(4); c << 1, 2, 1, 2;
    p.SetLinearCost(c);
    Check("LP+eq (toy)", p, c);
  }

  // QP+eq (passing)
  {
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
    Check("QP+eq (toy)", p, c);
  }

  // QPTEST (passing, no eq)
  {
    Model p;
    Eigen::SparseMatrix<double> A(2, 2); A.setIdentity();
    Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
    std::vector<int> v = {0, 1};
    p.AddLinearConstraint(A, b, v);
    Eigen::SparseMatrix<double> A2(1, 2);
    A2.insert(0, 0) = -1; A2.insert(0, 1) = -1; A2.makeCompressed();
    Eigen::VectorXd b2(1); b2(0) = 3.0;
    p.AddLinearConstraint(A2, b2, v);
    Eigen::SparseMatrix<double> Q(2, 2);
    Q.insert(0, 0) = 4; Q.insert(0, 1) = 1; Q.insert(1, 0) = 1; Q.insert(1, 1) = 4;
    Q.makeCompressed();
    p.AddQuadraticCost(Q, v);
    Eigen::VectorXd c(2); c << 1, -2;
    p.SetLinearCost(c);
    Check("QP (no eq)", p, c);
  }
}
