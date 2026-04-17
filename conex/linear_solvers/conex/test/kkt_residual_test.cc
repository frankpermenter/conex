// Test: read CVXQP1_S, CVXQP2_S, CVXQP3_S, GOULDQP3.
// Discard bounds, merge Q + I into one matrix, one equality constraint.
// Build Problem, solve KKT directly, demand zero residual.
#include <cstdio>
#include <cmath>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/problem.h"
#include "conex/common/solver.h"
#include "conex/common/qps_reader.h"
#include "conex/common/equality_constraint.h"
#include "conex/tree_solver/kkt_tree_solver.h"
using namespace conex;

bool TestInstance(const char* name, const char* path) {
  printf("=== %s ===\n", name);
  auto [prob_full, info] = ReadQPS(path);
  int n = info.num_variables;
  printf("  vars=%d, eq=%d, quad=%d\n", n, info.num_equality_rows,
         info.num_quadratic_entries);

  // Extract Q (merged with I) and C, d from the full problem.
  // Build a single merged Q = Q_original + I.
  std::vector<Eigen::Triplet<double>> q_trips;
  for (int i = 0; i < n; ++i) q_trips.emplace_back(i, i, 1.0);  // I
  for (const auto& c : prob_full.constraints()) {
    if (auto* qc = std::get_if<Problem::QuadraticCostData>(&c)) {
      for (int k = 0; k < qc->Q_sparse.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(qc->Q_sparse, k);
             it; ++it)
          q_trips.emplace_back(qc->vars[it.row()], qc->vars[it.col()],
                               it.value());
    }
  }
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(q_trips.begin(), q_trips.end());

  // Extract single aggregated C, d.
  std::vector<Eigen::Triplet<double>> c_trips;
  std::vector<double> d_vals;
  int eq_row = 0;
  for (const auto& c : prob_full.constraints()) {
    if (auto* ec = std::get_if<Problem::EqualityConstraintData>(&c)) {
      for (int k = 0; k < ec->C.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(ec->C, k); it; ++it)
          c_trips.emplace_back(eq_row + it.row(), ec->primal_vars[it.col()],
                               it.value());
      for (int j = 0; j < ec->d.size(); ++j)
        d_vals.push_back(ec->d(j));
      eq_row += ec->C.rows();
    }
  }
  int p = eq_row;
  Eigen::SparseMatrix<double> C(p, n);
  C.setFromTriplets(c_trips.begin(), c_trips.end());
  Eigen::VectorXd d = Eigen::Map<Eigen::VectorXd>(d_vals.data(), p);

  // Build Problem with exactly one Q and one equality.
  Problem qp;
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  qp.AddQuadraticCost(Q, vars);
  qp.AddEqualityConstraint(C, d, vars);
  if (prob_full.has_linear_cost())
    qp.SetLinearCost(prob_full.linear_cost());

  printf("  Problem: %d constraints, %d vars\n",
         qp.num_constraints(), qp.num_variables());

  // Build solver with dense KKT, factor, solve.
  auto solver = Solver::BuildDense(qp);
  auto* kkt = solver.solver();
  auto* ts = solver.tree_solver();
  bool ok = kkt->AssembleAndFactor();
  printf("  Factor: %s\n", ok ? "ok" : "FAIL");
  if (!ok) { printf("  FAIL\n\n"); return false; }

  int nv = kkt->number_of_variables();
  printf("  nv=%d, ts=%s, eq_assemblers=%d\n", nv,
         ts ? "yes" : "NULL",
         ts ? (int)ts->equality_sub_assemblers().size() : -1);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nv);
  if (qp.has_linear_cost())
    rhs.head(n) = -qp.linear_cost();
  // Set equality d at dual positions using Solver's dual_var_map.
  for (int i = 0; i < qp.num_constraints(); ++i) {
    if (auto* ec = std::get_if<Problem::EqualityConstraintData>(
            &qp.constraint(i))) {
      const auto& dv = solver.dual_variables(i);
      for (int j = 0; j < (int)dv.size(); ++j)
        rhs(dv[j]) = ec->d(j);
    }
  }

  Eigen::VectorXd sol = kkt->Solve(rhs);

  // KKT residual via the assembled matrix.
  Eigen::MatrixXd K = kkt->KKTMatrix(false);
  Eigen::VectorXd tree_res = K * sol - rhs;

  // Eigen dense LDLT as reference.
  Eigen::VectorXd eigen_sol = K.ldlt().solve(rhs);
  Eigen::VectorXd eigen_res = K * eigen_sol - rhs;

  printf("  nv=%d, ||K||=%.2e\n", nv, K.norm());
  printf("  Tree:  ||K*x-rhs||=%.2e  ||x||=%.2e\n", tree_res.norm(), sol.norm());
  printf("  Eigen: ||K*x-rhs||=%.2e  ||x||=%.2e\n", eigen_res.norm(), eigen_sol.norm());
  printf("  ||tree - eigen|| = %.2e\n", (sol - eigen_sol).norm());

  // Check Cx = d.
  Eigen::VectorXd x = sol.head(n);
  double eq_err = (C * x - d).norm();
  printf("  ||Cx - d|| = %.2e\n", eq_err);

  double rel_res = tree_res.norm() / std::max(1.0, rhs.norm());
  bool pass = rel_res < 1e-8;
  printf("  %s\n\n", pass ? "PASS" : "FAIL");
  return pass;
}

int main() {
  const char* dir = "/agent-workspace/problem_libraries/maros_meszaros/QPS_Files/";
  bool all_pass = true;
  char path[512];
  for (const char* name : {"CVXQP1_S", "CVXQP2_S", "CVXQP3_S", "GOULDQP3"}) {
    snprintf(path, sizeof(path), "%s%s.QPS", dir, name);
    all_pass &= TestInstance(name, path);
  }
  printf("%s\n", all_pass ? "ALL PASSED" : "SOME FAILED");
  return all_pass ? 0 : 1;
}
