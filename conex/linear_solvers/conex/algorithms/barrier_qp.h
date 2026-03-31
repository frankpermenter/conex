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

// Barrier method on an already-built solver.
// The Problem must have linear constraints (inequalities, with b stored
// as the affine term) and optionally quadratic costs.
// c is the linear cost (variable-space sized), x0 is a strictly feasible start.
// The inequality RHS b is read from the solver via GetAffineTerm().
BarrierQPResult SolveBarrierQP(
    KKTSolverBase& kkt,
    const Eigen::VectorXd& c,   // linear cost (variable-space)
    const Eigen::VectorXd& x0,  // strictly feasible start
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
