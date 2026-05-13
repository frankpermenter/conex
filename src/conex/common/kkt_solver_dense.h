// Dense (Eigen-dependent) convenience methods for KKTSolverBase.
// Include this only in .cc files or tests that need Solve(MatrixXd),
// KKTMatrix(), or MakeBlockVariable().
#pragma once

#include <Eigen/Core>
#include "conex/common/kkt_solver_interface.h"

namespace conex {

// Free functions wrapping the raw-pointer virtual interface.
inline Eigen::MatrixXd KKTSolve(const KKTSolverBase& solver,
                                Eigen::Ref<const Eigen::MatrixXd> b,
                                bool permute = true) {
  Eigen::MatrixXd x = b;
  solver.DenseSolveInPlace(x.data(), x.rows(), x.cols(), permute);
  return x;
}

inline Eigen::MatrixXd KKTMatrix(const KKTSolverBase& solver,
                                 bool permute = false) {
  int n = solver.number_of_variables();
  Eigen::MatrixXd M(n, n);
  solver.DenseKKTMatrix(M.data(), n, permute);
  return M;
}

inline BlockVariable MakeBlockVariable(KKTSolverBase& solver, int cols = 1) {
  return BlockVariable(solver.MakePartition(), cols);
}

inline BlockVariable MakeBlockVariable(
    KKTSolverBase& solver, Eigen::Ref<const Eigen::MatrixXd> x) {
  auto bv = MakeBlockVariable(solver, x.cols());
  bv.ScatterFrom(x);
  return bv;
}

inline void SolveInto(KKTSolverBase& solver,
                      const BlockVariable& rhs,
                      BlockVariable& dest) {
  if (solver.DoSolveBlocked(rhs.partition(), dest.partition())) return;
  // Fallback: dense round-trip.
  Eigen::MatrixXd b = rhs.Gather();
  Eigen::MatrixXd x = KKTSolve(solver, b);
  dest.ScatterFrom(x);
}

}  // namespace conex
