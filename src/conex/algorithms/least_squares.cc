#include "conex/algorithms/least_squares.h"

#include <chrono>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "conex/common/model.h"
#include "conex/common/solver.h"

namespace conex {

SparseLeastSquaresResult SparseLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  using clock = std::chrono::high_resolution_clock;
  SparseLeastSquaresResult result;
  const int n = A.cols();

  auto t0 = clock::now();

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(A.rows()), vars);

  auto solver = Solver::Build(problem);

  auto t1 = clock::now();

  bool ok = solver.AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  std::vector<double> rhs_reduced_vec;
  {
    Eigen::VectorXd rhs_r = solver.ReduceVector(rhs);
    rhs_reduced_vec.assign(rhs_r.data(), rhs_r.data() + rhs_r.size());
  }
  { auto xv = solver.SolveLinearSystem(rhs_reduced_vec); result.x = Eigen::Map<Eigen::VectorXd>(xv.data(), xv.size()); }

  auto t3 = clock::now();

  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.assemble_and_factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();
  result.grouping_us = 0;
  result.add_constraints_us = 0;
  result.init_workspace_us = 0;
  result.clique_extraction_us = 0;
  result.finalize_us = 0;

  return result;
}

SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquares(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  using clock = std::chrono::high_resolution_clock;
  SparseQuadraticTermLeastSquaresResult result;
  const int n = A.cols();

  auto t0 = clock::now();

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(A.rows()), vars);
  problem.AddQuadraticCost(Q, vars);

  auto solver = Solver::Build(problem);

  auto t1 = clock::now();

  bool ok = solver.AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  std::vector<double> rhs_reduced_vec;
  {
    Eigen::VectorXd rhs_r = solver.ReduceVector(rhs);
    rhs_reduced_vec.assign(rhs_r.data(), rhs_r.data() + rhs_r.size());
  }
  { auto xv = solver.SolveLinearSystem(rhs_reduced_vec); result.x = Eigen::Map<Eigen::VectorXd>(xv.data(), xv.size()); }

  auto t3 = clock::now();

  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();

  return result;
}

SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquares(
    const Eigen::MatrixXd& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  Eigen::SparseMatrix<double> Qs = Q.sparseView();
  return SparseQuadraticTermLeastSquares(Qs, A, rhs);
}

}  // namespace conex
