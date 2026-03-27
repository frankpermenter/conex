#include "conex/algorithms/equality_constrained_least_squares.h"

#include <chrono>
#include <cmath>
#include <numeric>
#include <set>

#include "conex/common/conex.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/sparse_equality_constraint.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/structural_rank.h"
#include "conex/tree_solver/kkt_solver_factory.h"

namespace conex {

namespace {

// Check consistency of dropped equality rows before removal.
void CheckDroppedRowConsistency(
    const Eigen::SparseMatrix<double>& C,
    const Eigen::VectorXd& d,
    const std::vector<int>& row_map) {
  int nr = C.rows();
  int nr_new = static_cast<int>(row_map.size());
  if (nr_new == nr) return;

  std::vector<bool> kept(nr, false);
  for (int idx : row_map) kept[idx] = true;

  Eigen::MatrixXd C_kept(nr_new, C.cols());
  Eigen::VectorXd d_kept(nr_new);
  for (int i = 0; i < nr_new; ++i) {
    for (int j = 0; j < C.cols(); ++j) {
      C_kept(i, j) = C.coeff(row_map[i], j);
    }
    d_kept(i) = d(row_map[i]);
  }

  auto qr = C_kept.transpose().colPivHouseholderQr();

  for (int r = 0; r < nr; ++r) {
    if (kept[r]) continue;
    Eigen::VectorXd c_dropped(C.cols());
    for (int j = 0; j < C.cols(); ++j) c_dropped(j) = C.coeff(r, j);
    double d_dropped = d(r);

    Eigen::VectorXd lambda = qr.solve(c_dropped);
    double c_residual = (C_kept.transpose() * lambda - c_dropped).norm();
    double c_scale = std::max(c_dropped.norm(), 1.0);

    if (c_residual < 1e-10 * c_scale) {
      double d_predicted = lambda.dot(d_kept);
      double d_err = std::abs(d_predicted - d_dropped);
      double d_scale = std::max(std::abs(d_dropped), 1.0);
      CONEX_DEMAND(d_err < 1e-10 * d_scale,
                   "Inconsistent equality constraints: a structurally "
                   "dependent row is in the row space of the kept rows "
                   "but its right-hand side is incompatible.");
    }
  }
}

}  // namespace

EqualityConstrainedLeastSquaresResult EqualityConstrainedLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double>& C,
    const Eigen::VectorXd& d) {
  using clock = std::chrono::high_resolution_clock;
  EqualityConstrainedLeastSquaresResult result;
  const int num_vars = A.cols();

  auto t0 = clock::now();

  // Drop structurally dependent equality rows (with consistency check).
  std::vector<int> row_map;
  Eigen::SparseMatrix<double> C_reduced =
      DropStructurallyDependentRows(C, &row_map);
  CheckDroppedRowConsistency(C, d, row_map);

  Eigen::VectorXd d_reduced(row_map.size());
  for (int i = 0; i < static_cast<int>(row_map.size()); ++i) {
    d_reduced(i) = d(row_map[i]);
  }
  const int num_eq = C_reduced.rows();

  // Build SparseLinearConstraint for A (assembles A^T A).
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A, b_zero);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(num_vars);

  auto slc_assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(std::move(slc_assembler));

  // Build SparseEqualityConstraint (decomposes C into per-clique blocks).
  auto sec = std::make_unique<SparseEqualityConstraint>(C_reduced, d_reduced);

  std::set<int> eq_var_set;
  for (const auto& s : sec->row_supports()) {
    eq_var_set.insert(s.begin(), s.end());
  }
  std::vector<int> eq_primal(eq_var_set.begin(), eq_var_set.end());

  std::vector<int> dual_vars = cm.AllocateDualVariables(num_eq);

  auto eq_assembler = std::make_unique<SparseEqualityConstraintAssembler>(
      std::move(sec), eq_primal, dual_vars);
  cm.AddCustomAssembler(std::move(eq_assembler));

  SolverConfiguration config;
  auto tree_solver = MakeTreeSolver(&cm, config);

  auto t1 = clock::now();

  bool ok = tree_solver->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  // Build RHS = [A^T b; d_reduced].
  int system_size = cm.SizeOfKKTSystem();
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(system_size);
  rhs.head(num_vars) = A.transpose() * b;
  for (int i = 0; i < num_eq; ++i) {
    rhs(dual_vars[i]) = d_reduced(i);
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
