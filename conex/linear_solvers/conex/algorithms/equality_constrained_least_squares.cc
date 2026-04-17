#include "conex/algorithms/equality_constrained_least_squares.h"

#include <chrono>
#include <numeric>

#include "conex/common/kkt_solver_interface.h"
#include "conex/common/solver.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {

EqualityConstrainedLeastSquaresResult EqualityConstrainedLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double>& C,
    const Eigen::VectorXd& d) {
  using clock = std::chrono::high_resolution_clock;
  EqualityConstrainedLeastSquaresResult result;
  const int n = A.cols();

  auto t0 = clock::now();

  // Build Problem: Q = A'A (quadratic cost), equality Cx = d.
  // Need a dummy cone constraint so the tree solver has a PD block.
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  // Quadratic cost: (1/2)x'(A'A)x.  The solver stores Q and the
  // objective is (1/2)x'Qx.  We pass Q = A'A.
  Eigen::SparseMatrix<double> Q = (A.transpose() * A).pruned();
  problem.AddQuadraticCost(Q, vars);
  // Linear cost: -A'b (from expanding ||Ax-b||^2 = x'A'Ax - 2b'Ax + b'b).
  Eigen::VectorXd c = -(A.transpose() * b);
  problem.SetLinearCost(c);
  // Equality constraint.
  auto c_eq = problem.AddEqualityConstraint(C, d, vars);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.solver();
  auto t1 = clock::now();

  bool ok = kkt->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");
  auto t2 = clock::now();

  // Build RHS = [-c; d] = [A'b; d].
  int nv = kkt->number_of_variables();
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nv);
  rhs.head(n) = -c;  // = A'b

  // Set equality RHS at dual positions.
  const auto& duals = solver.dual_variables(c_eq);
  auto* eq = std::get_if<Problem::EqualityConstraintData>(
      &problem.constraint(c_eq));
  if (eq) {
    for (int i = 0; i < static_cast<int>(duals.size()); ++i)
      rhs(duals[i]) = eq->d(i);
  }

  Eigen::VectorXd sol = kkt->Solve(rhs);
  auto t3 = clock::now();

  result.x = sol.head(n);
  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.assemble_and_factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();

  return result;
}

QPEqualityResult SolveQPEquality(const Problem& problem) {
  // Extract Q, c, C, d from the Problem.
  const int n = problem.num_variables();
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // Rebuild a Problem with only the QP + equality data (no cone constraints).
  // The tree solver needs at least one PD contribution to build,
  // so we add a zero-weight quadratic cost if none exists.
  Problem qp;

  // Copy quadratic costs.
  bool has_Q = false;
  for (const auto& c : problem.constraints()) {
    if (auto* qc = std::get_if<Problem::QuadraticCostData>(&c)) {
      qp.AddQuadraticCost(qc->Q_sparse, qc->vars);
      has_Q = true;
    }
  }

  // Copy equality constraints.
  std::vector<ConstraintId> eq_ids;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    if (auto* ec = std::get_if<Problem::EqualityConstraintData>(
            &problem.constraint(i))) {
      eq_ids.push_back(
          qp.AddEqualityConstraint(ec->C, ec->d, ec->primal_vars));
    }
  }

  // Linear cost.
  if (problem.has_linear_cost())
    qp.SetLinearCost(problem.linear_cost());

  auto solver = Solver::Build(qp);
  auto* kkt = solver.solver();

  bool ok = kkt->AssembleAndFactor();
  if (!ok) return {{}, 0, false};

  // Build RHS = [-c; d_1; d_2; ...].
  int nv = kkt->number_of_variables();
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nv);
  if (qp.has_linear_cost()) {
    int nc = std::min(n, (int)qp.linear_cost().size());
    rhs.head(nc) = -qp.linear_cost().head(nc);
  }

  // Set equality RHS at dual positions.
  for (auto eid : eq_ids) {
    const auto& duals = solver.dual_variables(eid);
    auto* ec = std::get_if<Problem::EqualityConstraintData>(
        &qp.constraint(eid));
    if (ec) {
      for (int i = 0; i < static_cast<int>(duals.size()); ++i)
        rhs(duals[i]) = ec->d(i);
    }
  }

  Eigen::VectorXd sol = kkt->Solve(rhs);
  Eigen::VectorXd x = sol.head(n);

  // Compute objective.
  double obj = 0;
  if (qp.has_linear_cost()) {
    int nc = std::min(n, (int)qp.linear_cost().size());
    obj += qp.linear_cost().head(nc).dot(x.head(nc));
  }
  for (const auto& c : qp.constraints()) {
    if (auto* qc = std::get_if<Problem::QuadraticCostData>(&c)) {
      for (int k = 0; k < qc->Q_sparse.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(qc->Q_sparse, k);
             it; ++it) {
          int i = qc->vars[it.row()], j = qc->vars[it.col()];
          if (i < n && j < n)
            obj += 0.5 * it.value() * x(i) * x(j);
        }
    }
  }

  return {x, obj, true};
}

}  // namespace conex
