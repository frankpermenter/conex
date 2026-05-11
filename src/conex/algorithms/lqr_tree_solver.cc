#include "conex/algorithms/lqr_tree_solver.h"

#include <numeric>

#include "conex/common/clique_tree.h"
#include "conex/common/error_checking_macros.h"
#include "conex/linear_solvers/assembler_adapter.h"

namespace conex {

LQRTreeSolver::LQRTreeSolver(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& Qf,
    int T)
    : nx_(A.rows()), nu_(B.cols()), T_(T) {
  // Variable layout: [x_0, u_0, λ_0, x_1, u_1, λ_1, ..., x_T, λ_ic]
  n_vars_ = T * step() + 2 * nx_;

  // Build CliqueTree: chain 0 → 1 → ... → T.
  CliqueTree tree;
  tree.supernodes.resize(T + 1);
  tree.separators.resize(T + 1);
  tree.node_to_parent.resize(T + 1);
  tree.post_order_position_to_clique.resize(T + 1);
  std::iota(tree.post_order_position_to_clique.begin(),
            tree.post_order_position_to_clique.end(), 0);

  // Cliques t = 0..T-1.
  for (int t = 0; t < T; ++t) {
    std::vector<int> sn;
    for (int i = 0; i < nx_; ++i) sn.push_back(XIdx(t) + i);
    for (int i = 0; i < nu_; ++i) sn.push_back(UIdx(t) + i);
    for (int i = 0; i < nx_; ++i) sn.push_back(LIdx(t) + i);
    if (t == 0) {
      // Add initial condition dual to clique 0's supernode.
      for (int i = 0; i < nx_; ++i) sn.push_back(LicIdx() + i);
    }
    tree.supernodes[t] = sn;

    std::vector<int> sep;
    for (int i = 0; i < nx_; ++i) sep.push_back(XIdx(t + 1) + i);
    tree.separators[t] = sep;

    tree.node_to_parent[t] = t + 1;
  }

  // Root clique T: supernode {x_T}, no separator.
  {
    std::vector<int> sn;
    for (int i = 0; i < nx_; ++i) sn.push_back(XTIdx() + i);
    tree.supernodes[T] = sn;
    tree.separators[T] = {};
    tree.node_to_parent[T] = -1;
  }

  // Build assemblers and adapters.
  solver_ = std::make_unique<SymmetricLinearSystemTreeSolver>();

  // Dynamics constraint at timestep t: [-A, -B, I] on {x_t, u_t, x_{t+1}}.
  Eigen::MatrixXd C_dyn(nx_, nx_ + nu_ + nx_);
  C_dyn << -A, -B, Eigen::MatrixXd::Identity(nx_, nx_);
  Eigen::VectorXd d_zero = Eigen::VectorXd::Zero(nx_);

  // Initial condition: I on {x_0}.
  Eigen::MatrixXd C_ic = Eigen::MatrixXd::Identity(nx_, nx_);

  // Cost block: [Q, 0; 0, R].
  Eigen::MatrixXd QR = Eigen::MatrixXd::Zero(nx_ + nu_, nx_ + nu_);
  QR.topLeftCorner(nx_, nx_) = Q;
  QR.bottomRightCorner(nu_, nu_) = R;

  for (int t = 0; t < T; ++t) {
    // Cost assembler: PD, variables = {x_t, u_t}.
    std::vector<int> cost_vars;
    for (int i = 0; i < nx_; ++i) cost_vars.push_back(XIdx(t) + i);
    for (int i = 0; i < nu_; ++i) cost_vars.push_back(UIdx(t) + i);
    cost_assemblers_.emplace_back(QR, cost_vars);
    auto cost_adapter = std::make_unique<AssemblerAdapter>(
        &cost_assemblers_.back());
    cost_adapter->set_contribution_type(ContributionType::kPositiveDefinite);
    solver_->push_back(std::move(cost_adapter));

    // Dynamics assembler: indefinite, primal = {x_t, u_t, x_{t+1}}, dual = {λ_t}.
    std::vector<int> dyn_primal;
    for (int i = 0; i < nx_; ++i) dyn_primal.push_back(XIdx(t) + i);
    for (int i = 0; i < nu_; ++i) dyn_primal.push_back(UIdx(t) + i);
    for (int i = 0; i < nx_; ++i) dyn_primal.push_back(XIdx(t + 1) + i);
    std::vector<int> dyn_dual;
    for (int i = 0; i < nx_; ++i) dyn_dual.push_back(LIdx(t) + i);
    dynamics_assemblers_.emplace_back(C_dyn, d_zero, dyn_primal, dyn_dual);
    auto dyn_adapter = std::make_unique<AssemblerAdapter>(
        &dynamics_assemblers_.back());
    dyn_adapter->set_contribution_type(ContributionType::kIndefinite);
    solver_->push_back(std::move(dyn_adapter));

    // Initial condition assembler (clique 0 only).
    if (t == 0) {
      std::vector<int> ic_primal;
      for (int i = 0; i < nx_; ++i) ic_primal.push_back(XIdx(0) + i);
      std::vector<int> ic_dual;
      for (int i = 0; i < nx_; ++i) ic_dual.push_back(LicIdx() + i);
      dynamics_assemblers_.emplace_back(C_ic, d_zero, ic_primal, ic_dual);
      auto ic_adapter = std::make_unique<AssemblerAdapter>(
          &dynamics_assemblers_.back());
      ic_adapter->set_contribution_type(ContributionType::kIndefinite);
      solver_->push_back(std::move(ic_adapter));
    }
  }

  // Terminal cost: Qf on {x_T}.
  {
    std::vector<int> term_vars;
    for (int i = 0; i < nx_; ++i) term_vars.push_back(XTIdx() + i);
    cost_assemblers_.emplace_back(Qf, term_vars);
    auto term_adapter = std::make_unique<AssemblerAdapter>(
        &cost_assemblers_.back());
    term_adapter->set_contribution_type(ContributionType::kPositiveDefinite);
    solver_->push_back(std::move(term_adapter));
  }

  solver_->SetUseGenericFactorization(false);
  solver_->SetUseLUForIndefinite(false);
  solver_->FinalizeStructure(tree);
  solver_->SetFactorizationMode(true);
  solver_->EnableAutoUpdateAtAssemble(true);
}

bool LQRTreeSolver::AssembleAndFactor() {
  return solver_->AssembleAndFactor();
}

Eigen::VectorXd LQRTreeSolver::Solve(const Eigen::VectorXd& x0) {
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n_vars_);
  for (int i = 0; i < nx_; ++i)
    rhs(LicIdx() + i) = x0(i);
  return solver_->Solve(rhs);
}

Eigen::MatrixXd LQRTreeSolver::ExtractStates(
    const Eigen::VectorXd& sol) const {
  Eigen::MatrixXd x(nx_, T_ + 1);
  for (int t = 0; t <= T_; ++t)
    x.col(t) = sol.segment(XIdx(t), nx_);
  return x;
}

}  // namespace conex
