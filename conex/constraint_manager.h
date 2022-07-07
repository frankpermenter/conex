#pragma once

#include <any>
#include <list>
#include <numeric>

#include "conex/constraint.h"
#include "conex/equality_constraint.h"
#include "conex/error_checking_macros.h"
#include "conex/quadratic_cost.h"
#include "conex/workspace.h"
#define CONEX_ID int

namespace conex {

class EqualityConstraintManager {
 public:
  std::vector<EqualityConstraints> data;
  std::vector<std::vector<int>> variables;
  std::vector<std::vector<int>> dual_variables;
  std::list<SupernodalAssemblerEqualities> assemblers;
};

class ConstraintManager {
 public:
  ConstraintManager(int max_number_of_variables)
      : max_number_of_variables_(max_number_of_variables),
        new_dual_variable_start_(max_number_of_variables_),
        linear_cost_vector_(max_number_of_variables_) {
    ClearLinearCost();
  }

  ConstraintManager(){};

  void ClearLinearCost() { linear_cost_vector_.setZero(); }

  bool AddLinearCost(const Eigen::VectorXd& b) {
    CONEX_RETURN_ON_FAIL(
        GetNumberOfVariables() == b.rows(),
        "Cost vector dimension does not equal number of variables");
    linear_cost_vector_ += b;
    return CONEX_SUCCESS;
  }

  bool AddLinearCost(const Eigen::VectorXd& b, const std::vector<int>& vars) {
    CONEX_DEMAND(static_cast<int>(vars.size()) == b.rows(),
                 "Cost vector dimension does not equal number of variables");
    int cnt = 0;
    for (auto i : vars) {
      linear_cost_vector_(i) += b(cnt++);
    }
    return false;
  }

  Eigen::VectorXd GetLinearCostVector() const { return linear_cost_vector_; }

  void SetNumberOfVariables(int N) {
    max_number_of_variables_ = N;
    new_dual_variable_start_ = N;
    linear_cost_vector_.resize(N);
    ClearLinearCost();
  }

  int GetNumberOfVariables() const { return max_number_of_variables_; }

  int SizeOfKKTSystem() const;

  template <typename T>
  CONEX_ID AddConstraint(T&& x) {
    std::vector<int> clique(max_number_of_variables_);
    for (size_t i = 0; i < clique.size(); i++) {
      clique[i] = i;
    }
    return AddConstraint(std::forward<T>(x), clique);
  }

  template <typename T>
  CONEX_ID AddConstraint(const T& x, const std::vector<int>& variables) {
    CONEX_DEMAND(Validate(variables) == CONEX_SUCCESS,
                 "Failed to add constraint.");
    using Type = typename std::remove_reference<T>::type;
    std::unique_ptr<ConstraintBase> pointer = std::make_unique<Type>(x);
    CONEX_CHECK(pointer->number_of_variables() ==
                static_cast<int>(variables.size()));
    constraint_storage_.emplace_back(std::move(pointer));
    constraints_.emplace_back(
        dynamic_cast<Type*>(constraint_storage_.back().get()));
    constraint_assemblers_.emplace_back(variables, &constraints_.back(),
                                        constraint_storage_.back().get());

    cone_inequalities_.push_back(&constraints_.back());
    cone_inequality_assemblers_.push_back(&constraint_assemblers_.back());
    return constraints_.size() - 1;
  }

  template <typename T>
  CONEX_ID AddQuadraticCost(const T& Qi, const std::vector<int>& variables) {
    CONEX_DEMAND(Validate(variables) == CONEX_SUCCESS,
                 "Failed to add constraint.");

    quadratic_costs_.emplace_back(Qi, variables);

    return quadratic_costs_.size() - 1;
  }

  std::vector<Workspace> workspace() {
    std::vector<Workspace> workspaces;
    for (auto& c : constraints_) {
      workspaces.push_back(c.workspace());
    }
    for (auto& c : clique_assemblers()) {
      workspaces.emplace_back(c->submatrix_data());
    }
    return workspaces;
  }

  CONEX_ID AddEqualityConstraint(const EqualityConstraints& x,
                                 const std::vector<int>& variables);
  CONEX_ID AddEqualityConstraint(const EqualityConstraints& x);

  void InitializeWorkspace();

  const std::vector<SupernodalAssemblerConstraint*>& cone_inequalities() const {
    return cone_inequality_assemblers_;
  }

  std::vector<SupernodalAssemblerConstraint*>& cone_inequalities() {
    return cone_inequality_assemblers_;
  }

  std::vector<SupernodalAssemblerBase*> clique_assemblers();
  std::vector<const SupernodalAssemblerBase*> clique_assemblers() const;

  std::list<SupernodalAssemblerQuadratic>& quadratic_costs() {
    return quadratic_costs_;
  }

  const std::list<SupernodalAssemblerQuadratic>& quadratic_costs() const {
    return quadratic_costs_;
  }

  const EqualityConstraintManager& equality_constraints() const {
    return equality_constraints_;
  }

  EqualityConstraintManager& equality_constraints() {
    return equality_constraints_;
  }
  const std::vector<std::vector<int>>& equality_constraint_multipliers() const;
  const std::vector<std::vector<int>>& variables() const;

 private:
  CONEX_STATUS Validate(const std::vector<int>& variables);
  mutable std::vector<std::vector<int>> dual_vars_;
  mutable std::vector<std::vector<int>> cliques_;
  std::list<SupernodalAssemblerQuadratic> quadratic_costs_;
  std::list<SupernodalAssemblerConstraint> constraint_assemblers_;

  // Stores and owns the constraints.
  std::vector<std::unique_ptr<ConstraintBase>> constraint_storage_;

  // Provides type-erased interface to constraints.
  // forwards to objects in constraint_storage_.
  std::list<Constraint> constraints_;

  // Provides random access to constraints_.
  std::vector<Constraint*> cone_inequalities_;
  std::vector<SupernodalAssemblerConstraint*> cone_inequality_assemblers_;
  EqualityConstraintManager equality_constraints_;

  int max_number_of_variables_ = 0;
  int new_dual_variable_start_ = 0;
  Eigen::VectorXd workspace_memory_;
  Eigen::VectorXd linear_cost_vector_;
};

void MakeObjectiveStrictlyConvex(ConstraintManager* x, double eps = 1e-3);
}  // namespace conex
