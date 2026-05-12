#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>

namespace conex {

// Solve the finite-horizon LQR problem:
//   min  Σ_{t=0}^{T-1} [x_t'Q x_t + u_t'R u_t]  +  x_T'Qf x_T
//   s.t. x_{t+1} = A x_t + B u_t,  t = 0, ..., T-1
//        x_0 = x_init
//
// Uses a sparse quadratic cost assembler (block-diagonal) and a sparse
// equality constraint assembler (block-banded dynamics) fed into the
// tree solver.  The resulting KKT system has chain structure.
struct LQRFromSparseMatricesResult {
  Eigen::MatrixXd x;   // nx x (T+1), state trajectory
  Eigen::MatrixXd u;   // nu x T, control trajectory
  double construction_time_us;
  double factor_time_us;
  double solve_time_us;
};

LQRFromSparseMatricesResult SolveLQRFromSparseMatrices(
    const Eigen::MatrixXd& A,      // nx x nx, dynamics
    const Eigen::MatrixXd& B,      // nx x nu, input matrix
    const Eigen::MatrixXd& Q,      // nx x nx, state cost (PSD)
    const Eigen::MatrixXd& R,      // nu x nu, control cost (PD)
    const Eigen::MatrixXd& Qf,     // nx x nx, terminal cost (PSD)
    const Eigen::VectorXd& x0,     // nx, initial state
    int T);                          // horizon length

}  // namespace conex
