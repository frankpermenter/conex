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

  std::vector<SupernodalAssemblerBase*>& CliqueAssemblers() {
    return supernodal_assemblers_ptr_;
  }

  int SizeOfKKTSystem() {
    int num_dual_vars = 0;
    for (auto dv : dual_vars) {
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

    inequality_constraints_.push_back(x);
    constraints_.emplace_back(
        std::any_cast<typename std::remove_reference<T>::type>(
            &inequality_constraints_.back()));
    supernodal_assemblers_.emplace_back(variables.size(), &constraints_.back());
    supernodal_assemblers_ptr_.push_back(&supernodal_assemblers_.back());

    cliques.push_back(variables);
    dual_vars.push_back({});

    cone_inequalities_.push_back(&constraints_.back());
    return CONEX_SUCCESS;
  }

  template <typename T>
  bool AddQuadraticCost(const T& Qi, const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return CONEX_FAILURE;
    }

    static_supernodal_assemblers_.emplace_back(Qi);
    supernodal_assemblers_ptr_.push_back(&static_supernodal_assemblers_.back());

    cliques.push_back(variables);
    dual_vars.push_back({});
    return CONEX_SUCCESS;
  }

  bool AddEqualityConstraint(EqualityConstraints&& x,
                             const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return CONEX_FAILURE;
    }
    const int m = x.SizeOfDualVariable();

    inequality_constraints_.push_back(x);
    constraints_.emplace_back(
        std::any_cast<EqualityConstraints>(&inequality_constraints_.back()));
    supernodal_assemblers_.emplace_back(variables.size() + m,
                                        &constraints_.back());
    supernodal_assemblers_ptr_.push_back(&supernodal_assemblers_.back());

    cliques.push_back(variables);
    dual_vars.push_back({});
    for (int i = 0; i < m; i++) {
      cliques.back().push_back(i + dual_variable_start_);
      dual_vars.back().push_back(i + dual_variable_start_);
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

  // Use a list so that we do not trigger reallocations.
  std::vector<std::vector<int>> cliques;
  std::vector<std::vector<int>> dual_vars;

  std::vector<Constraint*> cone_inequalities_;
  std::vector<SupernodalAssemblerBase*> supernodal_assemblers_ptr_;

  // Stores type-erased interface
  std::list<Constraint> constraints_;
  std::list<SupernodalAssembler> supernodal_assemblers_;
  std::list<SupernodalAssemblerStatic> static_supernodal_assemblers_;

 private:
  // Stores the provided constraint.
  std::list<std::any> inequality_constraints_;

  int max_number_of_variables_ = 0;
  int dual_variable_start_ = 0;
  Eigen::VectorXd workspace_memory_;
};

inline void AssembleSchurComplementResiduals(ConstraintManager* kkt,
                                             SchurComplementSystem* s) {
  s->setZero();
  int i = 0;
  for (auto& ci : kkt->supernodal_assemblers_) {
    auto* rhs_i = &ci.submatrix_data_;
    s->inner_product_of_w_and_c += rhs_i->inner_product_of_w_and_c;
    s->inner_product_of_c_and_Qc += rhs_i->inner_product_of_c_and_Qc;
    int cnt = 0;
    for (auto k : kkt->cliques.at(i)) {
      s->AW(k) += rhs_i->AW(cnt);
      s->AQc(k) += rhs_i->AQc(cnt);
      cnt++;
    }
    i++;
  }
}

}  // namespace conex
