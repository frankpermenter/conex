#pragma once

#include <memory>
#include <numeric>
#include <vector>

#include "conex/common/error_checking_macros.h"
#include "conex/common/supernodal_assembler_base.h"
#include <Eigen/Dense>

namespace conex {

class ConstraintManager {
 public:
  ConstraintManager(int max_number_of_variables)
      : max_number_of_variables_(max_number_of_variables),
        new_dual_variable_start_(max_number_of_variables_) {}

  ConstraintManager(){};

  int GetNumberOfVariables() const { return max_number_of_variables_; }
  int SizeOfKKTSystem() const;

  // Allocate dual variable indices.  Returns a vector of size count
  // with consecutive indices starting at the current dual variable offset.
  std::vector<int> AllocateDualVariables(int count) {
    std::vector<int> vars(count);
    std::iota(vars.begin(), vars.end(), new_dual_variable_start_);
    new_dual_variable_start_ += count;
    return vars;
  }

  // Register a custom assembler (caller retains ownership).
  void AddCustomAssembler(CliqueProvider* assembler) {
    custom_assemblers_.push_back(assembler);
  }

  // Register a custom assembler (takes ownership).
  void AddCustomAssembler(std::unique_ptr<CliqueProvider> assembler) {
    custom_assemblers_.push_back(assembler.get());
    owned_custom_assemblers_.push_back(std::move(assembler));
  }

  int num_custom_assemblers() const {
    return static_cast<int>(custom_assemblers_.size());
  }

  std::vector<CliqueProvider*> clique_assemblers();
  std::vector<const CliqueProvider*> clique_assemblers() const;

 private:
  std::vector<CliqueProvider*> custom_assemblers_;
  std::vector<std::unique_ptr<CliqueProvider>> owned_custom_assemblers_;
  int max_number_of_variables_ = 0;
  int new_dual_variable_start_ = 0;
};

}  // namespace conex
