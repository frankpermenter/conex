#pragma once

#include <any>
#include <list>
#include <numeric>
#include "conex/equality_constraint.h"

#include "conex/error_checking_macros.h"
#define CONEX_ID int

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

  int GetNumberOfVariables() const { return max_number_of_variables_; }

  int SizeOfKKTSystem() const {
    int num_aux_vars = 0;
    for (auto e : supernodal_assemblers_ptr_) {
      num_aux_vars += e->number_of_auxiliary_variables();
    }
    return max_number_of_variables_ + num_aux_vars;
  };

  template <typename T>
  CONEX_ID AddConstraint(T&& x) {
    std::vector<int> clique(max_number_of_variables_);
    for (size_t i = 0; i < clique.size(); i++) {
      clique[i] = i;
    }
    return AddConstraint(std::forward<T>(x), clique);
  }

  template <typename T>
  CONEX_ID AddConstraint(T&& x, const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return CONEX_FAILURE;
    }

    constraint_storage_.push_back(x);
    constraints_.emplace_back(
        std::any_cast<typename std::remove_reference<T>::type>(
            &constraint_storage_.back()));
    supernodal_assemblers_.emplace_back(variables, &constraints_.back());
    supernodal_assemblers_ptr_.push_back(&supernodal_assemblers_.back());

    cone_inequalities_.push_back(&constraints_.back());
    cone_inequality_assemblers_.push_back(&supernodal_assemblers_.back());
    return constraints_.size() - 1;
  }

  template <typename T>
  CONEX_ID AddQuadraticCost(const T& Qi, const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return CONEX_FAILURE;
    }

    quadratic_costs_.emplace_back(Qi, variables);
    supernodal_assemblers_ptr_.push_back(&quadratic_costs_.back());

    return quadratic_costs_.size() - 1;
  }

  CONEX_ID AddEqualityConstraint(EqualityConstraints&& x,
                                 const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return CONEX_FAILURE;
    }

    equality_constraints_.emplace_back(x.A_, x.b_, variables);

    supernodal_assemblers_ptr_.push_back(&equality_constraints_.back());

    return equality_constraints_.size() - 1;
  }

  std::vector<Workspace> workspace() {
    std::vector<Workspace> workspaces;
    for (auto& c : constraints_) {
      workspaces.push_back(c.workspace());
    }
    for (auto& c : supernodal_assemblers_ptr_) {
      workspaces.emplace_back(c->submatrix_data());
    }
    return workspaces;
  }

  CONEX_ID AddEqualityConstraint(EqualityConstraints&& x) {
    std::vector<int> clique(max_number_of_variables_);
    for (size_t i = 0; i < clique.size(); i++) {
      clique[i] = i;
    }
    return AddEqualityConstraint(std::forward<EqualityConstraints>(x), clique);
  }

  void InitializeWorkspace() {
    auto workspaces = workspace();
    auto size = SizeOf(workspaces);
    if (size > workspace_memory_.size()) {
      workspace_memory_.resize(size);
    }
    Initialize(&workspaces, workspace_memory_.data());
  }

  const std::vector<SupernodalAssembler*>& cone_inequalities() const {
    return cone_inequality_assemblers_;
  }

  std::vector<SupernodalAssembler*>& cone_inequalities() {
    return cone_inequality_assemblers_;
  }

  const std::vector<SupernodalAssemblerBase*>& clique_assemblers() const {
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
    int offset = max_number_of_variables_;
    dual_vars_.clear();
    for (auto e : supernodal_assemblers_ptr_) {
      int n = e->number_of_auxiliary_variables();
      std::vector<int> temp(n);
      std::iota(temp.begin(), temp.end(), offset);
      dual_vars_.push_back(temp);
      offset += n;
    }
    return dual_vars_;
  }

  const std::vector<std::vector<int>>& variables() const {
    cliques_.clear();
    std::vector<std::vector<int>> dual_vars = equality_constraint_multipliers();
    int i = 0;
    for (auto e : supernodal_assemblers_ptr_) {
      cliques_.push_back({});
      auto& c = cliques_.back();
      c = e->variables();
      c.insert(c.end(), dual_vars.at(i).begin(), dual_vars.at(i).end());
      i++;
    }
    return cliques_;
  }

 private:
  mutable std::vector<std::vector<int>> dual_vars_;
  mutable std::vector<std::vector<int>> cliques_;
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
  std::vector<SupernodalAssembler*> cone_inequality_assemblers_;

  // Provides type-erased interface to supernodal assemblers.
  std::vector<SupernodalAssemblerBase*> supernodal_assemblers_ptr_;

  int max_number_of_variables_ = 0;
  int dual_variable_start_ = 0;
  Eigen::VectorXd workspace_memory_;
};

}  // namespace conex
