#pragma once
#include "conex/constraint.h"
#include "conex/constraint_manager.h"
#include "conex/equality_constraint.h"
#include "conex/error_checking_macros.h"
#include "conex/kkt_solver.h"
#include "conex/supernodal_assembler.h"
#include "workspace.h"

namespace conex {

enum : int {
  CONEX_INITIALIZATION_MODE_COLDSTART = 0,
  CONEX_INITIALIZATION_MODE_WARMSTART = 1,
};

struct SolverConfiguration {
  int prepare_dual_variables = 0;
  int initialization_mode = 0;
  // TODO(FrankPermenter): Remove inv_sqrt_mu_max
  double inv_sqrt_mu_max = 1000;
  double minimum_mu = 1e-15;
  double maximum_mu = 1e4;
  double divergence_upper_bound = 1;
  int enable_line_search = 0;
  double dinf_upper_bound = 1;
  int final_centering_steps = 5;
  double final_centering_tolerance = .01;
  int initial_centering_steps_warmstart = 0;
  int initial_centering_steps_coldstart = 0;
  double warmstart_abort_threshold = 2;
  int max_iterations = 25;
  double infeasibility_threshold = 1e5;
  double kkt_error_tolerance = 1e10;
  int kkt_solver = 0; /* 0 = LLT, 1 = LDLT, 2 = QR*/
  int enable_rescaling = 1;
  int iterative_refinement_iterations = 0;
};

struct ConexStatus {
  int solved = 0;
  int num_iterations;
  int primal_infeasible = 0;
  int dual_infeasible = 0;
};

class Program {
 public:
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
    linear_cost_ = Eigen::VectorXd::Zero(m);
  }

  int GetNumberOfVariables() {
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

  void InitializeWorkspace() {
    workspaces = kkt_system_manager_.workspace();

    workspaces.emplace_back(stats.get());
    workspaces.emplace_back(&sys);
    auto size = SizeOf(workspaces);
    if (size > workspace_data_->size()) {
      workspace_data_->resize(size);
    }
    Initialize(&workspaces, workspace_data_->data());

    is_initialized = true;
  }

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

  Eigen::VectorXd linear_cost_;
};

DenseMatrix GetFeasibleObjective(Program* prog);

bool Solve(Program& prog, const SolverConfiguration& config,
           double* primal_variable);

bool Solve(const DenseMatrix& b, Program& prog,
           const SolverConfiguration& config, double* primal_variable);

bool Initialize(Program& prog, const SolverConfiguration& config);
}  // namespace conex
