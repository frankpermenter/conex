// Diagnostic tool for debugging QPS instances.
//
// Three tests:
//   1. Central path: replace cost with c=A'e, b=e. Verify hybrid converges.
//   2. Primal feasibility: set c=0, Q=0. Solve with ThetaCont. Check slack > 0.
//   3. Dual feasibility: set b=0, Q=0. Solve with ThetaCont. Check lambda > 0.
//
// Usage: ./debug_problem <file.qps>

#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
#include "conex/common/qps_reader.h"
#include "conex/common/solver.h"

namespace conex {
namespace {

using Eigen::VectorXd;

// Build a Model from the QPS constraint structure (A, vars, bounds)
// but with custom b and cost.  Copies linear constraints, quadratic
// costs, and equality constraints from the source.
struct ProblemParts {
  // Per linear constraint: A, vars, row count.
  struct LC { Eigen::SparseMatrix<double> A; std::vector<int> vars; int m; };
  std::vector<LC> linears;
  // Per quadratic cost.
  struct QC { Eigen::SparseMatrix<double> Q; std::vector<int> vars; };
  std::vector<QC> quadratics;
  // Per equality constraint.
  struct EC { Eigen::SparseMatrix<double> C; VectorXd d; std::vector<int> vars; };
  std::vector<EC> equalities;
  // Per SOC constraint.
  struct SC { Eigen::SparseMatrix<double> A; VectorXd b; std::vector<int> vars; };
  std::vector<SC> socs;

  int num_variables = 0;
};

ProblemParts ExtractParts(const Model& model) {
  ProblemParts parts;
  parts.num_variables = model.num_variables();
  for (int i = 0; i < model.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        parts.linears.push_back({data.A, data.vars, (int)data.A.rows()});
      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        parts.quadratics.push_back({data.Q_sparse, data.vars});
      } else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        parts.equalities.push_back({data.C, data.d, data.primal_vars});
      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        parts.socs.push_back({data.A, data.b, data.vars});
      }
    }, model.constraint(i));
  }
  return parts;
}

// =====================================================================
// Test 1: Central path (c = A'e, b = e)
// =====================================================================
bool TestCentralPath(const ProblemParts& parts) {
  printf("Test 1: Central path (c=A'e, b=e)\n");

  Model model;
  for (auto& lc : parts.linears) {
    VectorXd b_ones = VectorXd::Ones(lc.m);
    model.AddLinearConstraint(lc.A, b_ones, lc.vars);
  }
  for (auto& qc : parts.quadratics)
    model.AddQuadraticCost(qc.Q, qc.vars);
  for (auto& ec : parts.equalities)
    model.AddEqualityConstraint(ec.C, ec.d, ec.vars);
  for (auto& sc : parts.socs)
    model.AddSOCConstraint(sc.A, sc.b, sc.vars);

  // c = A'e: compute from Model data.
  printf("  Building model with %d linears...\n", (int)parts.linears.size());
  fflush(stdout);
  int n = parts.num_variables;
  VectorXd c = VectorXd::Zero(n);
  for (auto& lc : parts.linears) {
    VectorXd ones_m = VectorXd::Ones(lc.m);
    VectorXd at_ones = Eigen::MatrixXd(lc.A).transpose() * ones_m;
    for (int j = 0; j < (int)lc.vars.size(); ++j)
      c(lc.vars[j]) += at_ones(j);
  }
  model.SetLinearCost(c);

  printf("  Building solver...\n"); fflush(stdout);
  auto solver = Solver::Build(model);
  printf("  Solving...\n"); fflush(stdout);
  auto result = solver.Solve(PhaseOneHybrid());

  bool ok = std::abs(result.gap) < 1e-6 && result.d_inf <= 1.001;
  printf("  gap=%.2e  d_inf=%.4f  mu=%.2e  fac=%d  %s\n",
         result.gap, result.d_inf, result.mu, result.factorizations,
         ok ? "PASS" : "FAIL");
  return ok;
}

// =====================================================================
// Test 2: Primal feasibility (c=0, Q=0, original b)
// =====================================================================
bool TestPrimalFeasibility(const ProblemParts& parts, const Model& original) {
  printf("Test 2: Primal feasibility (c=A'e, Q=0, original A,b)\n");

  Model model;
  // Use original A and b from the QPS file.
  for (int i = 0; i < original.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData>)
        model.AddLinearConstraint(data.A, data.b, data.vars);
      else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>)
        model.AddEqualityConstraint(data.C, data.d, data.primal_vars);
      else if constexpr (std::is_same_v<T, Model::SOCConstraintData>)
        model.AddSOCConstraint(data.A, data.b, data.vars);
      // Skip QuadraticCostData — testing with Q=0.
    }, original.constraint(i));
  }
  // No quadratic cost.
  for (auto& ec : parts.equalities)
    model.AddEqualityConstraint(ec.C, ec.d, ec.vars);
  for (auto& sc : parts.socs)
    model.AddSOCConstraint(sc.A, sc.b, sc.vars);

  // c = A'e for centering.
  int n = parts.num_variables;
  VectorXd c = VectorXd::Zero(n);
  for (auto& lc : parts.linears) {
    VectorXd at_ones = Eigen::MatrixXd(lc.A).transpose() * VectorXd::Ones(lc.m);
    for (int j = 0; j < (int)lc.vars.size(); ++j)
      c(lc.vars[j]) += at_ones(j);
  }
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(ThetaContinuation());

  // Check slacks.
  double min_slack = 1e30;
  for (auto& s : result.duals.slack)
    min_slack = std::min(min_slack, s.minCoeff());

  bool ok = result.converged && min_slack > -1e-6;
  printf("  min_slack=%.2e  converged=%d  mu=%.2e  %s\n",
         min_slack, result.converged, result.mu,
         ok ? "PASS" : "FAIL");
  if (min_slack > 0)
    printf("  Strictly feasible (all slacks > 0)\n");
  else
    printf("  Not strictly feasible (min slack = %.2e)\n", min_slack);
  return ok;
}

// =====================================================================
// Test 3: Dual feasibility (b=0, Q=0, original c)
// =====================================================================
bool TestDualFeasibility(const ProblemParts& parts, const Model& original) {
  printf("Test 3: Dual feasibility (b=e, Q=0, original A, original c)\n");

  Model model;
  // Use original A but b=e (guarantees primal interior at x=0).
  for (auto& lc : parts.linears)
    model.AddLinearConstraint(lc.A, VectorXd::Ones(lc.m), lc.vars);
  // No quadratic cost.
  for (auto& ec : parts.equalities)
    model.AddEqualityConstraint(ec.C, ec.d, ec.vars);
  for (auto& sc : parts.socs)
    model.AddSOCConstraint(sc.A, sc.b, sc.vars);

  VectorXd c = original.has_linear_cost()
      ? original.linear_cost()
      : VectorXd::Zero(original.num_variables());
  model.SetLinearCost(c);

  auto solver = Solver::Build(model);
  auto result = solver.Solve(ThetaContinuation());

  // Check lambda.
  double min_lambda = 1e30;
  for (auto& l : result.duals.lambda)
    min_lambda = std::min(min_lambda, l.minCoeff());

  bool ok = result.converged && min_lambda > -1e-6;
  printf("  min_lambda=%.2e  converged=%d  mu=%.2e  dual_res=%.2e  %s\n",
         min_lambda, result.converged, result.mu,
         result.optimality.dual_residual,
         ok ? "PASS" : "FAIL");
  if (min_lambda > 0)
    printf("  Strictly dual feasible (all lambda > 0)\n");
  else
    printf("  Not strictly dual feasible (min lambda = %.2e)\n", min_lambda);
  return ok;
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  if (argc < 2) {
    printf("Usage: %s <file.qps>\n", argv[0]);
    return 1;
  }

  auto [model, info] = conex::ReadQPS(argv[1]);
  printf("QPS: %s\n", info.name.c_str());
  printf("  vars=%d  eq=%d  ineq=%d  quad=%d  bounds=%d  c0=%.2e\n\n",
         info.num_variables, info.num_equality_rows,
         info.num_inequality_rows, info.num_quadratic_entries,
         info.num_bounded_vars, info.objective_constant);

  printf("Extracting parts...\n"); fflush(stdout);
  auto parts = conex::ExtractParts(model);
  printf("Parts: %d linear, %d quad, %d eq, %d soc\n",
         (int)parts.linears.size(), (int)parts.quadratics.size(),
         (int)parts.equalities.size(), (int)parts.socs.size());
  fflush(stdout);

  int pass = 0, total = 3;
  printf("---\n");
  if (conex::TestCentralPath(parts)) pass++;
  printf("---\n");
  if (conex::TestPrimalFeasibility(parts, model)) pass++;
  printf("---\n");
  if (conex::TestDualFeasibility(parts, model)) pass++;
  printf("---\n\n");

  printf("Summary: %d/%d passed\n", pass, total);
  return (pass == total) ? 0 : 1;
}
