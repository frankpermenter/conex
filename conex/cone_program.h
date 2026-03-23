#pragma once
#include <type_traits>

#include "conex/conex.h"
#include "conex/constraint.h"
#include "conex/constraint_manager.h"
#include "conex/equality_constraint.h"
#include "conex/error_checking_macros.h"
#include "conex/kkt_solver_interface.h"
#include "workspace.h"

namespace conex {

class SparseLinearConstraint;

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

  Eigen::MatrixXd GetDualVariable(int i);

  template <typename T>
  void GetDualVariable(int i, T* xi) {
    int cnt = 0;
    for (auto& ci : kkt_system_manager_.cone_inequalities()) {
      if (cnt == i) {
        ci->constraint()->get_dual_variable(xi->data());
        if (!status_.primal_infeasible) {
          xi->array() /=
              (workspace_.stats->sqrt_inv_mu[workspace_.stats->num_iter - 1] *
               workspace_.stats->b_scaling());
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
  auto AddConstraint(T&& d) {
    using Type = std::decay_t<T>;
    if constexpr (std::is_same_v<Type, SparseLinearConstraint>) {
      AddSparseLinearConstraint(std::move(d));
    } else if constexpr (std::is_same_v<Type, EqualityConstraints>) {
      return kkt_system_manager_.AddEqualityConstraint(
          std::forward<EqualityConstraints>(d));
    } else {
      return kkt_system_manager_.AddConstraint<T>(std::forward<T>(d));
    }
  }

  template <typename T>
  CONEX_ID AddConstraint(T&& d, const std::vector<int>& variables) {
    if constexpr (!std::is_same_v<std::decay_t<T>, EqualityConstraints>) {
      return kkt_system_manager_.AddConstraint<T>(std::forward<T>(d),
                                                  variables);
    } else {
      return kkt_system_manager_.AddEqualityConstraint(
          std::forward<EqualityConstraints>(d), variables);
    }
  }

  // Register a SparseLinearConstraint as a custom assembler.  The actual
  // decomposition into per-clique LinearConstraints happens inside the
  // solver factory (MakeTreeSolver calls Decompose(maximal_cliques)).
  void AddSparseLinearConstraint(SparseLinearConstraint&& slc);

  int NumberOfConstraints() {
    return kkt_system_manager_.cone_inequalities().size() +
           kkt_system_manager_.num_custom_assemblers();
  }
  ConexStatus& Status() { return status_; }

  bool AddLinearCost(const Eigen::VectorXd& b);
  bool AddLinearCost(const Eigen::VectorXd& b,
                     const std::vector<int>& variables);
  void ClearLinearCosts();
  CONEX_ID AddQuadraticCost(const Eigen::MatrixXd& Q,
                            const std::vector<int>& variables);
  CONEX_ID AddQuadraticCost(const Eigen::MatrixXd& Q);

  int UpdateQuadraticCost(int cost_id, double value, int row, int col);
  int NumberOfQuadraticCosts() const;
  bool contains_quadratic_costs() { return NumberOfQuadraticCosts() > 0; }

  friend DenseMatrix GetFeasibleObjective(Program* prog);

  friend void SolveHSD(Program& prog, const Eigen::VectorXd& bin,
                       const SolverConfiguration& config, Eigen::VectorXd* yout,
                       double* tau, double* kappa);
  friend bool Solve(Program& prog, const SolverConfiguration& config,
                    double* primal_variable);
  friend bool Solve(const DenseMatrix& b, Program& prog,
                    const SolverConfiguration& config, double* primal_variable);
  friend bool Initialize(Program& prog, const SolverConfiguration& config);

  Eigen::VectorXd* workspace_memory() { return workspace_data_; }

  const WorkspaceStats& statistics() const { return *workspace_.stats; }
  WorkspaceStats& statistics() { return *workspace_.stats; }

  const ConstraintManager& constraint_manager() const {
    return kkt_system_manager_;
  };

  ConstraintManager& constraint_manager() { return kkt_system_manager_; };
  const SchurComplementSystem& kkt_system_residual() const {
    return workspace_.sys;
  };

  SchurComplementSystem& kkt_system_residual() { return workspace_.sys; };
  KKTSolverBase* kkt_solver() { return solver.get(); };

 private:
  struct IPMWorkSpace {
    SchurComplementSystem sys;
    std::unique_ptr<WorkspaceStats> stats;
  };
  IPMWorkSpace workspace_;

  ConstraintManager kkt_system_manager_;
  std::vector<Workspace> workspaces;
  std::unique_ptr<KKTSolverBase> solver;
  Eigen::VectorXd memory_;
  Eigen::VectorXd* workspace_data_;
  bool is_initialized = false;
  ConexStatus status_;
};

DenseMatrix GetFeasibleObjective(Program* prog);

bool Solve(Program& prog, const SolverConfiguration& config,
           double* primal_variable);

Eigen::VectorXd Solve(
    Program& prog, const SolverConfiguration& config = SolverConfiguration());

bool Solve(const DenseMatrix& b, Program& prog,
           const SolverConfiguration& config, double* primal_variable);

void AssembleSchurComplementResiduals(const ConstraintManager& kkt,
                                      SchurComplementSystem* s);

void PrepareStep(ConstraintManager* kkt,
                 const StepOptions& newton_step_parameters, const Ref& y,
                 StepInfo* info);

bool Initialize(Program& prog, const SolverConfiguration& config);

void TakeStep(std::vector<SupernodalAssemblerConstraint*>* constraints,
              const StepOptions& newton_step_parameters);

}  // namespace conex
