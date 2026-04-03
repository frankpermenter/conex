#pragma once
#include <Eigen/Dense>
#include <list>
#include <memory>
#include <vector>

#include "conex/common/equality_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/common/supernodal_assembler_base.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {

// Directly constructs a tree solver for finite-horizon LQR, bypassing
// clique ordering.  The chain-structured KKT system is built explicitly.
//
// Clique t (t = 0..T-1):
//   supernode = {x_t, u_t, λ_t}   (+ λ_ic for t=0)
//   separator = {x_{t+1}}
//   parent    = clique t+1
//   Assemblers: cost [Q,0;0,R] on {x_t,u_t}
//               dynamics [-A,-B,I] on {x_t,u_t,x_{t+1}} with dual {λ_t}
//               (t=0 only) initial condition [I] on {x_0} with dual {λ_ic}
//
// Clique T (root):
//   supernode = {x_T}
//   separator = {}
//   Assembler: terminal cost Qf on {x_T}
class LQRTreeSolver {
 public:
  LQRTreeSolver(const Eigen::MatrixXd& A,
                const Eigen::MatrixXd& B,
                const Eigen::MatrixXd& Q,
                const Eigen::MatrixXd& R,
                const Eigen::MatrixXd& Qf,
                int T);

  int nx() const { return nx_; }
  int nu() const { return nu_; }
  int T() const { return T_; }
  int n_vars() const { return n_vars_; }

  bool AssembleAndFactor();
  Eigen::VectorXd Solve(const Eigen::VectorXd& x0);

  Eigen::MatrixXd ExtractStates(const Eigen::VectorXd& sol) const;

  const SymmetricLinearSystemTreeSolver& solver() const { return *solver_; }

 private:
  int nx_, nu_, T_, n_vars_;

  // Variable layout: [x_0, u_0, λ_0, x_1, u_1, λ_1, ..., x_T, λ_ic]
  int step() const { return 2 * nx_ + nu_; }
  int XIdx(int t) const { return t * step(); }
  int UIdx(int t) const { return t * step() + nx_; }
  int LIdx(int t) const { return t * step() + nx_ + nu_; }
  int XTIdx() const { return T_ * step(); }
  int LicIdx() const { return T_ * step() + nx_; }

  // Owned assembler storage.
  std::list<QuadraticCost> cost_assemblers_;
  std::list<EqualityConstraint> dynamics_assemblers_;

  std::unique_ptr<SymmetricLinearSystemTreeSolver> solver_;
};

}  // namespace conex
