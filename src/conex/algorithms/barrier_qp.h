#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

namespace conex {

class Model;
class KKTSolverBase;

struct IterationStats {
  int newton_steps;      // Newton steps in this outer iteration
  double duality_gap;    // m / t
  double mu;             // 1 / t
};

struct BarrierQPResult {
  Eigen::VectorXd x;
  int outer_iterations;
  int total_newton_steps;
  double objective;
  double duality_gap;
  double solve_time_us;
  std::vector<IterationStats> iter_stats;
};

struct SolverRHS;

// Barrier method on an already-built solver.
// All inputs/outputs in the solver's native SolverRHS format.
// c_rhs = linear cost, x0 = strictly feasible start.
// b is read from the solver via GetAffineTerm().
BarrierQPResult SolveBarrierQP(
    KKTSolverBase& kkt,
    const SolverRHS& c_rhs,
    SolverRHS& x0,
    int max_outer_iterations = 30,
    int max_newton_steps = 50,
    double mu = 10.0,
    double tolerance = 1e-8);

// Convenience: builds Model + Solver from raw matrices.
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
