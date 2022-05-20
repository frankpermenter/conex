#include "conex/constraint_manager.h"
#include "conex/debug_macros.h"

namespace conex {
using T = ConstraintManager;
namespace {
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
}  // namespace

int T::SizeOfKKTSystem() const {
  return new_dual_variable_start_;
};

const std::vector<std::vector<int>>& T::variables() const {
  cliques_.clear();
  for (auto e : supernodal_assemblers_ptr_) {
    cliques_.push_back({});
    auto& c = cliques_.back();
    c = e->variables();
  }
  return cliques_;
}

const std::vector<std::vector<int>>& T::equality_constraint_multipliers() const {
  throw std::runtime_error("Obsolete");
}

CONEX_STATUS T::Validate(const std::vector<int>& variables) {
  if (!IsUnique(max_number_of_variables_, variables)) {
    return CONEX_FAILURE;
  }
  return CONEX_SUCCESS;
}

CONEX_ID T::AddEqualityConstraint(const EqualityConstraints& x,
                                  const std::vector<int>& variables) {
  if (!IsUnique(max_number_of_variables_, variables)) {
    return CONEX_FAILURE;
  }
  equality_constraint_multipliers_.emplace_back(x.A_.rows());
  std::iota(equality_constraint_multipliers_.back().begin(),
            equality_constraint_multipliers_.back().end(), new_dual_variable_start_);
  new_dual_variable_start_ += equality_constraint_multipliers_.back().size();
 

 #if 1
  std::vector<int>  primal_dual_variables; primal_dual_variables.reserve(1 + x.A_.rows());
  primal_dual_variables.push_back(variables.at(0));
  std::copy(equality_constraint_multipliers_.back().begin(),
            equality_constraint_multipliers_.back().end(),
            std::back_inserter(primal_dual_variables));

  equality_constraints_.emplace_back(x.A_.col(0), x.b_, primal_dual_variables);
  supernodal_assemblers_ptr_.push_back(&equality_constraints_.back());
  for (size_t i = 1; i < variables.size(); i++) {
    primal_dual_variables.at(0) = variables.at(i);
    equality_constraints_.emplace_back(x.A_.col(i), 0*x.b_, primal_dual_variables);
    supernodal_assemblers_ptr_.push_back(&equality_constraints_.back());
  }
  #else
  std::vector<int>  primal_dual_variables = variables; 
  std::copy(equality_constraint_multipliers_.back().begin(),
            equality_constraint_multipliers_.back().end(),
            std::back_inserter(primal_dual_variables));

  equality_constraints_.emplace_back(x.A_, x.b_, primal_dual_variables);
  supernodal_assemblers_ptr_.push_back(&equality_constraints_.back());

  #endif

  return equality_constraints_.size() - 1;
}
}  // namespace conex
