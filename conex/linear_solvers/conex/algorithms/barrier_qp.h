#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

class Problem;
class KKTSolverBase;

struct BarrierQPResult {
  Eigen::VectorXd x;
  int outer_iterations;
  int total_newton_steps;
  double objective;
  double duality_gap;
  double solve_time_us;
};

struct TreeRHS;

// Barrier method on an already-built solver.
// All inputs/outputs in the solver's native TreeRHS format.
// c_rhs = linear cost, x0 = strictly feasible start.
// b is read from the solver via GetAffineTerm().
BarrierQPResult SolveBarrierQP(
    KKTSolverBase& kkt,
    const TreeRHS& c_rhs,
    TreeRHS& x0,
    int max_outer_iterations = 30,
    int max_newton_steps = 50,
    double mu = 10.0,
    double tolerance = 1e-8);

// Convenience: builds Problem + Solver from raw matrices.
BarrierQPResult SolveBarrierQP(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& x0,
    int max_outer_iterations = 30,
    int max_newton_steps = 50,
    double mu = 10.0,
    double tolerance = 1e-8);

}  // namespace conex
