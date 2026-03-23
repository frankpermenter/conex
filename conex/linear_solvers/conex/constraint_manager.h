#pragma once

#include <list>
#include <numeric>
#include <type_traits>
#include <utility>

#include "conex/constraint.h"
#include "conex/equality_constraint.h"
#include "conex/error_checking_macros.h"
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
        new_dual_variable_start_(max_number_of_variables_) {}

  ConstraintManager(){};

  int GetNumberOfVariables() const { return max_number_of_variables_; }

  int SizeOfKKTSystem() const;

  CONEX_ID AddEqualityConstraint(const EqualityConstraints& x,
                                 const std::vector<int>& variables);
  CONEX_ID AddEqualityConstraint(const EqualityConstraints& x);

  std::vector<SupernodalAssemblerConstraint*>& cone_inequalities() {
    return cone_inequality_assemblers_;
  }

  // Register a custom assembler (caller retains ownership).
  void AddCustomAssembler(SupernodalAssemblerBase* assembler) {
    custom_assemblers_.push_back(assembler);
  }

  // Register a custom assembler (takes ownership).
  void AddCustomAssembler(std::unique_ptr<SupernodalAssemblerBase> assembler) {
    custom_assemblers_.push_back(assembler.get());
    owned_custom_assemblers_.push_back(std::move(assembler));
  }

  int num_custom_assemblers() const {
    return static_cast<int>(custom_assemblers_.size());
  }

  std::vector<SupernodalAssemblerBase*> clique_assemblers();
  std::vector<const SupernodalAssemblerBase*> clique_assemblers() const;

  const EqualityConstraintManager& equality_constraints() const {
    return equality_constraints_;
  }

  EqualityConstraintManager& equality_constraints() {
    return equality_constraints_;
  }
  const std::vector<std::vector<int>>& equality_constraint_multipliers() const;

 private:
  CONEX_STATUS Validate(const std::vector<int>& variables);
  mutable std::vector<std::vector<int>> dual_vars_;

  // Stores and owns all constraints through a single virtual interface.
  std::vector<std::unique_ptr<Constraint>> constraint_storage_;

  std::vector<SupernodalAssemblerConstraint*> cone_inequality_assemblers_;
  EqualityConstraintManager equality_constraints_;

  std::vector<SupernodalAssemblerBase*> custom_assemblers_;
  std::vector<std::unique_ptr<SupernodalAssemblerBase>> owned_custom_assemblers_;
  int max_number_of_variables_ = 0;
  int new_dual_variable_start_ = 0;
};

}  // namespace conex
