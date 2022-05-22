#pragma once

#include <any>
#include <list>

#include "conex/equality_constraint.h"
#include "conex/error_checking_macros.h"
#include "conex/supernodal_assembler.h"
#define CONEX_ID int

namespace conex {

class CliqueTree;
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
        new_dual_variable_start_(max_number_of_variables_) {}

  ConstraintManager(){};

  void SetNumberOfVariables(int N) {
    CONEX_CHECK(N > 0);
    max_number_of_variables_ = N;
    new_dual_variable_start_ = N;
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
  CONEX_ID AddConstraint(T&& x, const std::vector<int>& variables) {
    CONEX_DEMAND(Validate(variables) == CONEX_SUCCESS,
                 "Failed to add constraint.");

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
    CONEX_DEMAND(Validate(variables) == CONEX_SUCCESS,
                 "Failed to add constraint.");

    quadratic_costs_.emplace_back(Qi, variables);
    supernodal_assemblers_ptr_.push_back(&quadratic_costs_.back());

    return quadratic_costs_.size() - 1;
  }

  CONEX_ID AddEqualityConstraint(const EqualityConstraints& x,
                                 const std::vector<int>& variables);

  CONEX_ID AddEqualityConstraint(const EqualityConstraints& x) {
    std::vector<int> clique(max_number_of_variables_);
    for (size_t i = 0; i < clique.size(); i++) {
      clique[i] = i;
    }
    return AddEqualityConstraint(x, clique);
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
    return equality_constraints_.assemblers;
  }

  const std::vector<std::vector<int>>& equality_constraint_multipliers() const;
  std::vector<std::vector<int>> variables() const;
  void PartitionEqualityConstraints();

 private:
  CONEX_STATUS Validate(const std::vector<int>& variables);
  mutable std::vector<std::vector<int>> dual_vars_;
  std::list<SupernodalAssembler> supernodal_assemblers_;
  std::list<SupernodalAssemblerStatic> quadratic_costs_;

  // Stores and owns the constraints.
  std::list<std::any> constraint_storage_;

  // Provides type-erased interface to constraints.
  // forwards to objects in constraint_storage_.
  std::list<Constraint> constraints_;

  // Provides random access to constraints_.
  std::vector<Constraint*> cone_inequalities_;
  std::vector<SupernodalAssembler*> cone_inequality_assemblers_;
  EqualityConstraintManager equality_constraints_;

  // Provides type-erased interface to supernodal assemblers.
  std::vector<SupernodalAssemblerBase*> supernodal_assemblers_ptr_;

  int max_number_of_variables_ = 0;
  int new_dual_variable_start_ = 0;
  Eigen::VectorXd workspace_memory_;
  void PartitionEqualityConstraint(const Eigen::MatrixXd& A,
                                   const Eigen::MatrixXd& b,
                                   const std::vector<int>& variables,
                                   const CliqueTree* primal_tree,
                                   const std::vector<int>& multipliers);
};

}  // namespace conex
