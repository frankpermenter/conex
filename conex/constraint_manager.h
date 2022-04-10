#pragma once

#include <any>
#include <list>
#include "conex/equality_constraint.h"

#include "conex/error_checking_macros.h"

namespace conex {

inline int IsUnique(int N, const std::vector<int>& x) {
  Eigen::VectorXd y(N);
  y.setZero();
  for (auto& xi : x) {
    if (xi >= N) {
      return false;
    }
    y(xi)++;
    if (y(xi) > 1) {
      return false;
    }
  }
  return true;
}

class ConstraintManager {
 public:
  ConstraintManager(int max_number_of_variables)
      : max_number_of_variables_(max_number_of_variables),
        dual_variable_start_(max_number_of_variables_) {}

  ConstraintManager(){};

  void SetNumberOfVariables(int N) {
    max_number_of_variables_ = N;
    dual_variable_start_ = N;
  }

  int GetNumberOfVariables() { return max_number_of_variables_; }

  int SizeOfKKTSystem() {
    int num_dual_vars = 0;
    for (auto dv : dual_vars_) {
      num_dual_vars += dv.size();
    }
    return max_number_of_variables_ + num_dual_vars;
  };

  template <typename T>
  bool AddConstraint(T&& x) {
    std::vector<int> clique(max_number_of_variables_);
    for (size_t i = 0; i < clique.size(); i++) {
      clique[i] = i;
    }
    AddConstraint(std::forward<T>(x), clique);
    return CONEX_SUCCESS;
  }

  template <typename T>
  bool AddConstraint(T&& x, const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return CONEX_FAILURE;
    }

    constraint_storage_.push_back(x);
    constraints_.emplace_back(
        std::any_cast<typename std::remove_reference<T>::type>(
            &constraint_storage_.back()));
    supernodal_assemblers_.emplace_back(variables.size(), &constraints_.back());
    supernodal_assemblers_ptr_.push_back(&supernodal_assemblers_.back());

    cliques_.push_back(variables);
    dual_vars_.push_back({});

    cone_inequalities_.push_back(&constraints_.back());
    return CONEX_SUCCESS;
  }

  template <typename T>
  bool AddQuadraticCost(const T& Qi, const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return CONEX_FAILURE;
    }

    quadratic_costs_.emplace_back(Qi);
    supernodal_assemblers_ptr_.push_back(&quadratic_costs_.back());

    cliques_.push_back(variables);
    dual_vars_.push_back({});
    return CONEX_SUCCESS;
  }

  bool AddEqualityConstraint(EqualityConstraints&& x,
                             const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return CONEX_FAILURE;
    }
    const int m = x.SizeOfDualVariable();

    equality_constraints_.emplace_back(x.A_, x.b_);

    supernodal_assemblers_ptr_.push_back(&equality_constraints_.back());

    cliques_.push_back(variables);
    dual_vars_.push_back({});
    for (int i = 0; i < m; i++) {
      cliques_.back().push_back(i + dual_variable_start_);
      dual_vars_.back().push_back(i + dual_variable_start_);
    }
    dual_variable_start_ += m;
    return CONEX_SUCCESS;
  }

  std::vector<Workspace> workspace() {
    std::vector<Workspace> workspaces;
    for (auto& c : constraints_) {
      workspaces.push_back(c.workspace());
    }
    for (auto& c : supernodal_assemblers_ptr_) {
      workspaces.emplace_back(&c->submatrix_data_);
    }
    return workspaces;
  }

  bool AddEqualityConstraint(EqualityConstraints&& x) {
    std::vector<int> clique(max_number_of_variables_);
    for (size_t i = 0; i < clique.size(); i++) {
      clique[i] = i;
    }
    AddEqualityConstraint(std::forward<EqualityConstraints>(x), clique);
    return CONEX_SUCCESS;
  }

  void InitializeWorkspace() {
    auto workspaces = workspace();
    auto size = SizeOf(workspaces);
    if (size > workspace_memory_.size()) {
      workspace_memory_.resize(size);
    }
    Initialize(&workspaces, workspace_memory_.data());
  }

  std::vector<Constraint*>& cone_inequalities() { return cone_inequalities_; }
  std::vector<SupernodalAssemblerBase*>& clique_assemblers() {
    return supernodal_assemblers_ptr_;
  }

  std::list<SupernodalAssemblerStatic>& quadratic_costs() {
    return quadratic_costs_;
  }

  const std::list<SupernodalAssemblerStatic>& quadratic_costs() const {
    return quadratic_costs_;
  }

  const std::list<SupernodalAssemblerEqualities>& equality_constraints() const {
    return equality_constraints_;
  }

  const std::vector<std::vector<int>>& equality_constraint_multipliers() const {
    return dual_vars_;
  }

  const std::vector<std::vector<int>>& variables() const { return cliques_; }

 private:
  std::vector<std::vector<int>> dual_vars_;
  std::vector<std::vector<int>> cliques_;
  std::list<SupernodalAssembler> supernodal_assemblers_;
  std::list<SupernodalAssemblerStatic> quadratic_costs_;
  std::list<SupernodalAssemblerEqualities> equality_constraints_;

  // Stores and owns the constraints.
  std::list<std::any> constraint_storage_;

  // Provides type-erased interface to constraints.
  // forwards to objects in constraint_storage_.
  std::list<Constraint> constraints_;

  // Provides random access to constraints_.
  std::vector<Constraint*> cone_inequalities_;

  // Provides type-erased interface to supernodal assemblers.
  std::vector<SupernodalAssemblerBase*> supernodal_assemblers_ptr_;

  int max_number_of_variables_ = 0;
  int dual_variable_start_ = 0;
  Eigen::VectorXd workspace_memory_;
};

}  // namespace conex
