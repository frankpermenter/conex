#include "conex/algorithms/equality_constrained_least_squares.h"

#include <chrono>
#include <numeric>
#include <set>

#include "conex/common/conex.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/equality_constraint.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/kkt_solver_factory.h"

namespace conex {

EqualityConstrainedLeastSquaresResult EqualityConstrainedLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double>& C,
    const Eigen::VectorXd& d) {
  using clock = std::chrono::high_resolution_clock;
  EqualityConstrainedLeastSquaresResult result;
  const int num_vars = A.cols();

  auto t0 = clock::now();

  // Build SparseLinearConstraint for A (assembles A^T A).
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A, b_zero);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(num_vars);

  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(std::move(assembler));

  // Add equality constraints Cx = d.
  std::vector<int> eq_vars(num_vars);
  std::iota(eq_vars.begin(), eq_vars.end(), 0);
  Eigen::MatrixXd C_dense(C);
  EqualityConstraints eq(C_dense, d);
  cm.AddEqualityConstraint(eq, eq_vars);

  // Preprocess: drop structurally dependent columns and equality rows.
  cm.Preprocess();

  SolverConfiguration config;
  auto tree_solver = MakeTreeSolver(&cm, config);

  auto t1 = clock::now();

  bool ok = tree_solver->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  // Build RHS = [A^T b; d_reduced] in the (possibly reduced) system.
  int num_primal = cm.GetNumberOfVariables();
  int system_size = cm.SizeOfKKTSystem();
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(system_size);

  Eigen::VectorXd atb = A.transpose() * b;
  if (cm.was_reduced()) {
    rhs.head(num_primal) = cm.ReduceVector(atb);
  } else {
    rhs.head(num_primal) = atb;
  }

  // Dual part: use the (possibly reduced) equality constraint RHS.
  int dual_offset = num_primal;
  for (const auto& eq_data : cm.equality_constraints().data) {
    int p = eq_data.b_.rows();
    for (int i = 0; i < p; ++i) {
      rhs(dual_offset + i) = eq_data.b_(i);
    }
    dual_offset += p;
  }

  Eigen::VectorXd sol = tree_solver->Solve(rhs);

  auto t3 = clock::now();

  if (cm.was_reduced()) {
    result.x = cm.ExpandSolution(sol.head(num_primal));
  } else {
    result.x = sol.head(num_vars);
  }

  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.assemble_and_factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();

  return result;
}

}  // namespace conex
