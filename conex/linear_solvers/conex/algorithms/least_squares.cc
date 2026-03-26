#include "conex/algorithms/least_squares.h"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <set>
#include <type_traits>
#include <unordered_map>

#include "conex/common/constraint_manager.h"
#include "conex/common/conex.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/common/structural_rank.h"
#include "conex/tree_solver/kkt_solver_factory.h"

namespace conex {

SparseLeastSquaresResult SparseLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  using clock = std::chrono::high_resolution_clock;
  SparseLeastSquaresResult result;
  const int num_vars_original = A.cols();

  auto t0 = clock::now();

  // Remove structurally rank-deficient columns.
  std::vector<int> col_map;
  Eigen::SparseMatrix<double> A_reduced =
      DropStructurallyDependentColumns(A, &col_map);
  const int num_vars = A_reduced.cols();

  Eigen::VectorXd rhs_reduced(num_vars);
  for (int i = 0; i < num_vars; ++i) {
    rhs_reduced(i) = rhs(col_map[i]);
  }

  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A_reduced.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A_reduced, b_zero);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(num_vars);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(assembler.get());

  auto t_grouped = clock::now();

  SolverConfiguration config;
  auto tree_solver = MakeTreeSolver(&cm, config);

  auto t1 = clock::now();

  bool ok = tree_solver->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  Eigen::VectorXd x_reduced = tree_solver->Solve(rhs_reduced);

  auto t3 = clock::now();

  // Expand solution: insert zeros for dropped columns.
  result.x = Eigen::VectorXd::Zero(num_vars_original);
  for (int i = 0; i < num_vars; ++i) {
    result.x(col_map[i]) = x_reduced(i);
  }

  result.grouping_us =
      std::chrono::duration<double, std::micro>(t_grouped - t0).count();
  result.add_constraints_us = 0;
  result.init_workspace_us = 0;
  result.clique_extraction_us = 0;
  result.finalize_us =
      std::chrono::duration<double, std::micro>(t1 - t_grouped).count();
  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.assemble_and_factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();

  return result;
}

namespace {

struct IsolatedDiagResult {
  std::vector<int> isolated_vars;
  std::vector<double> diag_values;
  std::vector<int> kept_vars;
  std::vector<int> kept_to_original;
};

IsolatedDiagResult FindIsolatedDiagonals(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::SparseMatrix<double>& A, int num_vars) {
  IsolatedDiagResult r;

  std::vector<bool> a_touches(num_vars, false);
  for (int k = 0; k < A.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
      a_touches[it.col()] = true;

  std::vector<bool> has_offdiag(num_vars, false);
  for (int k = 0; k < Q.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k); it; ++it)
      if (it.row() != it.col()) {
        has_offdiag[it.row()] = true;
        has_offdiag[it.col()] = true;
      }

  for (int i = 0; i < num_vars; ++i) {
    double qii = Q.coeff(i, i);
    if (!a_touches[i] && !has_offdiag[i] && qii != 0) {
      r.isolated_vars.push_back(i);
      r.diag_values.push_back(qii);
    } else {
      r.kept_vars.push_back(i);
    }
  }
  r.kept_to_original = r.kept_vars;
  return r;
}

IsolatedDiagResult FindIsolatedDiagonals(
    const Eigen::MatrixXd& /*Q*/,
    const Eigen::SparseMatrix<double>& /*A*/, int num_vars) {
  IsolatedDiagResult r;
  r.kept_vars.resize(num_vars);
  std::iota(r.kept_vars.begin(), r.kept_vars.end(), 0);
  r.kept_to_original = r.kept_vars;
  return r;
}

Eigen::SparseMatrix<double> ReduceQ(
    const Eigen::SparseMatrix<double>& Q,
    const std::vector<int>& kept_vars) {
  int m = static_cast<int>(kept_vars.size());
  std::unordered_map<int, int> old_to_new;
  for (int i = 0; i < m; ++i) old_to_new[kept_vars[i]] = i;

  std::vector<Eigen::Triplet<double>> trips;
  for (int k = 0; k < Q.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k); it; ++it) {
      auto ir = old_to_new.find(it.row());
      auto ic = old_to_new.find(it.col());
      if (ir != old_to_new.end() && ic != old_to_new.end())
        trips.emplace_back(ir->second, ic->second, it.value());
    }

  Eigen::SparseMatrix<double> Q_reduced(m, m);
  Q_reduced.setFromTriplets(trips.begin(), trips.end());
  return Q_reduced;
}

Eigen::SparseMatrix<double> ReduceA(
    const Eigen::SparseMatrix<double>& A,
    const std::vector<int>& kept_vars) {
  int m = static_cast<int>(kept_vars.size());
  std::unordered_map<int, int> old_to_new;
  for (int i = 0; i < m; ++i) old_to_new[kept_vars[i]] = i;

  std::vector<Eigen::Triplet<double>> trips;
  for (int k = 0; k < A.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      auto ic = old_to_new.find(it.col());
      if (ic != old_to_new.end())
        trips.emplace_back(it.row(), ic->second, it.value());
    }

  Eigen::SparseMatrix<double> A_reduced(A.rows(), m);
  A_reduced.setFromTriplets(trips.begin(), trips.end());
  return A_reduced;
}

template <typename QType>
SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquaresImpl(
    const QType& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  using clock = std::chrono::high_resolution_clock;
  SparseQuadraticTermLeastSquaresResult result;
  const int num_vars = A.cols();

  auto t0 = clock::now();

  auto isolated = FindIsolatedDiagonals(Q, A, num_vars);
  const bool has_isolated = !isolated.isolated_vars.empty();
  const int reduced_vars = static_cast<int>(isolated.kept_vars.size());

  Eigen::VectorXd x_full = Eigen::VectorXd::Zero(num_vars);
  for (size_t k = 0; k < isolated.isolated_vars.size(); ++k) {
    int i = isolated.isolated_vars[k];
    x_full(i) = rhs(i) / isolated.diag_values[k];
  }

  if (reduced_vars == 0) {
    result.x = x_full;
    auto t1 = clock::now();
    result.construction_time_us =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
    result.factor_time_us = 0;
    result.solve_time_us = 0;
    return result;
  }

  Eigen::SparseMatrix<double> A_reduced =
      has_isolated ? ReduceA(A, isolated.kept_vars) : A;
  Eigen::SparseMatrix<double> Q_reduced_storage;
  const auto& Q_for_solver = [&]() -> const QType& {
    if constexpr (std::is_same_v<QType, Eigen::SparseMatrix<double>>) {
      if (has_isolated) {
        Q_reduced_storage = ReduceQ(Q, isolated.kept_vars);
        return Q_reduced_storage;
      }
    }
    return Q;
  }();

  Eigen::VectorXd rhs_reduced(reduced_vars);
  for (int i = 0; i < reduced_vars; ++i)
    rhs_reduced(i) = rhs(isolated.kept_vars[i]);

  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A_reduced.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A_reduced, b_zero);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  for (int i = 0; i < reduced_vars; ++i) var_set.insert(i);
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(reduced_vars);

  auto a_assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(a_assembler.get());

  auto q_assembler = std::make_unique<SparseQuadraticTermAssembler>(
      Q_for_solver, all_vars);
  cm.AddCustomAssembler(q_assembler.get());

  auto t1 = clock::now();

  SolverConfiguration config;
  auto tree_solver = MakeTreeSolver(&cm, config);

  auto t2 = clock::now();

  bool ok = tree_solver->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t3 = clock::now();

  Eigen::VectorXd x_reduced = tree_solver->Solve(rhs_reduced);

  auto t4 = clock::now();

  for (int i = 0; i < reduced_vars; ++i)
    x_full(isolated.kept_vars[i]) = x_reduced(i);

  result.x = x_full;
  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t2 - t0).count();
  result.factor_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t4 - t3).count();

  return result;
}

}  // namespace

SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquares(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  return SparseQuadraticTermLeastSquaresImpl(Q, A, rhs);
}

SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquares(
    const Eigen::MatrixXd& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  return SparseQuadraticTermLeastSquaresImpl(Q, A, rhs);
}

}  // namespace conex
