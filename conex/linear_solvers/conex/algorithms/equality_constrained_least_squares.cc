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
    const Eigen::MatrixXd& C,
    const Eigen::VectorXd& d) {
  using clock = std::chrono::high_resolution_clock;
  EqualityConstrainedLeastSquaresResult result;
  const int num_vars = A.cols();
  const int num_eq = C.rows();

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
  cm.AddCustomAssembler(assembler.get());

  // Add equality constraints Cx = d.
  std::vector<int> eq_vars(num_vars);
  std::iota(eq_vars.begin(), eq_vars.end(), 0);
  EqualityConstraints eq(C, d);
  cm.AddEqualityConstraint(eq, eq_vars);

  SolverConfiguration config;
  auto tree_solver = MakeTreeSolver(&cm, config);

  auto t1 = clock::now();

  bool ok = tree_solver->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  // RHS = [A^T b; d].
  int system_size = cm.SizeOfKKTSystem();
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(system_size);
  Eigen::VectorXd atb = A.transpose() * b;
  rhs.head(num_vars) = atb;
  for (int i = 0; i < num_eq; ++i) {
    rhs(num_vars + i) = d(i);
  }

  Eigen::VectorXd sol = tree_solver->Solve(rhs);

  auto t3 = clock::now();

  result.x = sol.head(num_vars);
  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.assemble_and_factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();

  return result;
}

}  // namespace conex
