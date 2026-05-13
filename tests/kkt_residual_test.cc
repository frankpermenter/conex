// Test: read CVXQP1_S, CVXQP2_S, CVXQP3_S, GOULDQP3.
// Discard bounds, merge Q + I into one matrix, one equality constraint.
// Build Model, solve KKT directly, demand zero residual.
#include <cstdio>
#include <cmath>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/kkt_solver_dense.h"
#include "conex/common/qps_reader.h"
#include "conex/common/equality_constraint.h"
#include "conex/linear_solvers/kkt_tree_solver.h"
using namespace conex;

bool TestInstance(const char* name, const char* path,
                  const SolverConfiguration& config = SolverConfiguration{}) {
  printf("=== %s ===\n", name);
  auto [prob_full, info] = ReadQPS(path);
  int n = info.num_variables;
  printf("  vars=%d, eq=%d, quad=%d\n", n, info.num_equality_rows,
         info.num_quadratic_entries);

  // Extract Q (merged with A'A from inequalities) and C, d from the full problem.
  // Build Q = Q_original + A_ineq' * A_ineq, which matches the Gram matrix
  // G = A'W²A at W=I.  This is more realistic than adding I.
  std::vector<Eigen::Triplet<double>> q_trips;
  // Collect inequality constraint matrices and compute A'A.
  for (const auto& c : prob_full.constraints()) {
    if (auto* lc = std::get_if<Model::LinearConstraintData>(&c)) {
      Eigen::SparseMatrix<double> AtA = (lc->A.transpose() * lc->A).pruned();
      for (int k = 0; k < AtA.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(AtA, k); it; ++it)
          q_trips.emplace_back(lc->vars[it.row()], lc->vars[it.col()],
                               it.value());
    }
  }
  for (const auto& c : prob_full.constraints()) {
    if (auto* qc = std::get_if<Model::QuadraticCostData>(&c)) {
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
    if (auto* ec = std::get_if<Model::EqualityConstraintData>(&c)) {
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

  // Build Model with exactly one Q and one equality.
  Model qp;
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  qp.AddQuadraticCost(Q, vars);
  qp.AddEqualityConstraint(C, d, vars);
  if (prob_full.has_linear_cost())
    qp.SetLinearCost(prob_full.linear_cost());

  printf("  Model: %d constraints, %d vars\n",
         qp.num_constraints(), qp.num_variables());

  // Build solver with automatic tree (AMD ordering).
  auto solver = Solver::Build(qp, config);
  auto* kkt = solver.kkt();
  auto* ts = solver.tree_solver();
  bool ok = kkt->AssembleAndFactor();
  printf("  Factor: %s\n", ok ? "ok" : "FAIL");
  if (!ok) { printf("  FAIL\n\n"); return false; }

  int nv = kkt->number_of_variables();
  printf("  nv=%d, ts=%s, eq_assemblers=%d\n", nv,
         ts ? "yes" : "NULL",
         ts ? (int)ts->equality_sub_assemblers().size() : -1);
  // Print dual variable range.
  for (int i = 0; i < qp.num_constraints(); ++i) {
    if (auto* ec = std::get_if<Model::EqualityConstraintData>(
            &qp.constraint(i))) {
      const auto& dv = solver.dual_variables(i);
      if (!dv.empty())
        printf("  dual_vars[%d]: %d..%d (count=%d)\n",
               i, dv.front(), dv.back(), (int)dv.size());
      (void)ec;
    }
  }
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nv);
  if (qp.has_linear_cost())
    rhs.head(n) = -qp.linear_cost();
  // Set equality d at dual positions using Solver's dual_var_map.
  for (int i = 0; i < qp.num_constraints(); ++i) {
    if (auto* ec = std::get_if<Model::EqualityConstraintData>(
            &qp.constraint(i))) {
      const auto& dv = solver.dual_variables(i);
      for (int j = 0; j < (int)dv.size(); ++j)
        rhs(dv[j]) = ec->d(j);
    }
  }

  Eigen::VectorXd sol = KKTSolve(*kkt, rhs);

  // Build the KKT matrix manually from Q and C.
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(nv, nv);
  K.topLeftCorner(n, n) = Eigen::MatrixXd(Q);
  for (int ci = 0; ci < qp.num_constraints(); ++ci) {
    if (auto* ec = std::get_if<Model::EqualityConstraintData>(
            &qp.constraint(ci))) {
      const auto& dv = solver.dual_variables(ci);
      for (int k2 = 0; k2 < ec->C.outerSize(); ++k2)
        for (Eigen::SparseMatrix<double>::InnerIterator it(ec->C, k2);
             it; ++it) {
          int row = dv[it.row()];
          int col = ec->primal_vars[it.col()];
          K(row, col) = it.value();
          K(col, row) = it.value();
        }
    }
  }

  // Reference: Eigen dense LDLT.
  Eigen::VectorXd eigen_sol = K.ldlt().solve(rhs);

  // Tree solver residual against the manually-constructed K.
  // (KKTMatrix() is NOT usable after AssembleAndFactor — it returns
  // the factored data, not the assembled matrix.)
  Eigen::VectorXd tree_res = K * sol - rhs;
  Eigen::VectorXd eigen_res = K * eigen_sol - rhs;

  printf("  ||K||=%.2e, subsystems=%d\n", K.norm(), ts->num_subsystems());
  printf("  Tree:  ||K*x-rhs||=%.2e  ||x||=%.2e\n", tree_res.norm(), sol.norm());
  printf("  Eigen: ||K*x-rhs||=%.2e  ||x||=%.2e\n", eigen_res.norm(), eigen_sol.norm());
  printf("  ||tree - eigen|| = %.2e\n", (sol - eigen_sol).norm());

  // Check Cx = d.
  Eigen::VectorXd x = sol.head(n);
  double eq_err = (C * x - d).norm();
  printf("  ||Cx - d|| = %.2e\n", eq_err);

  double rel_res = tree_res.norm() / std::max(1.0, rhs.norm());
  printf("  rel_res = %.2e\n", rel_res);
  bool pass = rel_res < 1e-3;
  printf("  %s\n\n", pass ? "PASS" : "FAIL");
  return pass;
}

int main(int argc, char** argv) {
  SolverConfiguration config;
  if (argc > 1 && std::string(argv[1]) == "--lu") {
    config.tree.use_lu_for_indefinite = true;
    printf("Using LU for indefinite blocks\n\n");
  }
  const char* dir = "/agent-workspace/problem_libraries/maros_meszaros/QPS_Files/";
  bool all_pass = true;
  char path[512];
  for (const char* name : {"CVXQP1_S", "CVXQP2_S", "CVXQP3_S", "GOULDQP3"}) {
    snprintf(path, sizeof(path), "%s%s.QPS", dir, name);
    all_pass &= TestInstance(name, path, config);
  }
  printf("%s\n", all_pass ? "ALL PASSED" : "SOME FAILED");
  return all_pass ? 0 : 1;
}
