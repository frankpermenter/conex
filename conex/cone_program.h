#pragma once
#include "conex/conex.h"
#include "conex/constraint.h"
#include "conex/constraint_manager.h"
#include "conex/equality_constraint.h"
#include "conex/error_checking_macros.h"
#include "conex/kkt_solver_interface.h"
#include "workspace.h"

namespace conex {

class Program {
 public:
  Program(ConstraintManager&& constraints) {
    workspace_data_ = &memory_;
    kkt_system_manager_ = std::move(constraints);
  }

  Program(int number_of_variables) {
    workspace_data_ = &memory_;
    SetNumberOfVariables(number_of_variables);
  }

  Program(int number_of_variables, Eigen::VectorXd* data) {
    workspace_data_ = data;
    SetNumberOfVariables(number_of_variables);
  }

  void SetNumberOfVariables(int m) {
    CONEX_DEMAND(m >= 0, "Number of variables must be nonnegative.");
    kkt_system_manager_.SetNumberOfVariables(m);
  }

  int GetNumberOfVariables() const {
    return kkt_system_manager_.GetNumberOfVariables();
  }

  template <typename T>
  void GetDualVariable(int i, T* xi) {
    int cnt = 0;
    for (auto& ci : kkt_system_manager_.cone_inequalities()) {
      if (cnt == i) {
        ci->constraint()->get_dual_variable(xi->data());
        if (!status_.primal_infeasible) {
          xi->array() /=
              (stats->sqrt_inv_mu[stats->num_iter - 1] * stats->b_scaling());
        }
        return;
      }
      cnt++;
    }
  }

  int GetDualVariableSize(int i) {
    int cnt = 0;
    for (auto& ci : kkt_system_manager_.cone_inequalities()) {
      if (cnt == i) {
        return ci->constraint()->dual_variable_size();
      }
      cnt++;
    }
    CONEX_RETURN_ON_FAIL(false, "Invalid Constraint");
  }

  CONEX_STATUS UpdateLinearOperatorOfConstraint(int i, double value,
                                                int variable, int row, int col,
                                                int hyper_complex_dim);

  CONEX_STATUS UpdateAffineTermOfConstraint(int i, double value, int row,
                                            int col, int hyper_complex_dim);

  void InitializeWorkspace();

  template <typename T>
  CONEX_ID AddConstraint(T&& d) {
    if constexpr (!std::is_same<T, EqualityConstraints>::value) {
      return kkt_system_manager_.AddConstraint<T>(std::forward<T>(d));
    } else {
      return kkt_system_manager_.AddEqualityConstraint(
          std::forward<EqualityConstraints>(d));
    }
  }

  template <typename T>
  CONEX_ID AddConstraint(T&& d, const std::vector<int>& variables) {
    if constexpr (!std::is_same<T, EqualityConstraints>::value) {
      return kkt_system_manager_.AddConstraint<T>(std::forward<T>(d),
                                                  variables);
    } else {
      return kkt_system_manager_.AddEqualityConstraint(
          std::forward<EqualityConstraints>(d), variables);
    }
  }

  int NumberOfConstraints() {
    return kkt_system_manager_.cone_inequalities().size();
  }
  ConexStatus Status() { return status_; }

  bool AddLinearCost(const Eigen::VectorXd& b);
  bool AddLinearCost(const Eigen::VectorXd& b,
                     const std::vector<int>& variables);
  void ClearLinearCosts();
  CONEX_ID AddQuadraticCost(const Eigen::MatrixXd& Q,
                            const std::vector<int>& variables);
  CONEX_ID AddQuadraticCost(const Eigen::MatrixXd& Q);

  int UpdateQuadraticCost(int cost_id, double value, int row, int col);
  int NumberOfQuadraticCosts() const;

  friend DenseMatrix GetFeasibleObjective(Program* prog);
  friend bool Solve(Program& prog, const SolverConfiguration& config,
                    double* primal_variable);
  friend bool Solve(const DenseMatrix& b, Program& prog,
                    const SolverConfiguration& config, double* primal_variable);
  friend bool Initialize(Program& prog, const SolverConfiguration& config);

  Eigen::VectorXd* workspace_memory() { return workspace_data_; }

  const WorkspaceStats& statistics() const { return *stats; }

  const ConstraintManager& constraint_manager() const {
    return kkt_system_manager_;
  };

 private:
  ConstraintManager kkt_system_manager_;
  SchurComplementSystem sys;
  std::unique_ptr<WorkspaceStats> stats;
  std::vector<Workspace> workspaces;
  std::unique_ptr<KKTSolverBase> solver;
  Eigen::VectorXd memory_;
  Eigen::VectorXd* workspace_data_;
  bool is_initialized = false;
  bool contains_quadratic_costs_ = false;
  ConexStatus status_;
};

DenseMatrix GetFeasibleObjective(Program* prog);

bool Solve(Program& prog, const SolverConfiguration& config,
           double* primal_variable);

Eigen::VectorXd Solve(
    Program& prog, const SolverConfiguration& config = SolverConfiguration());

bool Solve(const DenseMatrix& b, Program& prog,
           const SolverConfiguration& config, double* primal_variable);

bool Initialize(Program& prog, const SolverConfiguration& config);
}  // namespace conex
