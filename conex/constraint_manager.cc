#include "conex/constraint_manager.h"
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

CONEX_STATUS T::Validate(const std::vector<int>& variables) {
  if (!IsUnique(max_number_of_variables_, variables)) {
    return CONEX_FAILURE;
  }
  return CONEX_SUCCESS;
}

int T::SizeOfKKTSystem() const {
  int num_aux_vars = 0;
  for (const auto& e : supernodal_assemblers_ptr_) {
    num_aux_vars += e->dual_variables().size();
  }
  return max_number_of_variables_ + num_aux_vars;
};

CONEX_ID T::AddEqualityConstraint(const EqualityConstraints& x,
                                  const std::vector<int>& variables) {
  CONEX_DEMAND(Validate(variables) == CONEX_SUCCESS,
               "Failed to add constraint.");

  equality_constraints_.data.push_back(x);
  equality_constraints_.dual_variables.emplace_back(x.A_.rows());
  std::iota(equality_constraints_.dual_variables.back().begin(),
            equality_constraints_.dual_variables.back().end(),
            new_dual_variable_start_);
  new_dual_variable_start_ +=
      equality_constraints_.dual_variables.back().size();
  equality_constraints_.variables.push_back(variables);

  equality_constraints_.assemblers.emplace_back(
      x.A_, x.b_, variables, equality_constraints_.dual_variables.back());

  supernodal_assemblers_ptr_.push_back(
      &equality_constraints_.assemblers.back());

  std::unique_ptr<ConstraintBase> pointer =
      std::make_unique<EqualityConstraints>(x);
  CONEX_CHECK(pointer->number_of_variables() ==
              static_cast<int>(variables.size()));
  constraint_storage_.emplace_back(std::move(pointer));

  return equality_constraints_.data.size() - 1;
}

CONEX_ID T::AddEqualityConstraint(const EqualityConstraints& x) {
  std::vector<int> clique(max_number_of_variables_);
  for (size_t i = 0; i < clique.size(); i++) {
    clique[i] = i;
  }
  return AddEqualityConstraint(x, clique);
}

void T::InitializeWorkspace() {
  auto workspaces = workspace();
  auto size = SizeOf(workspaces);
  if (size > workspace_memory_.size()) {
    workspace_memory_.resize(size);
  }
  Initialize(&workspaces, workspace_memory_.data());
}

const std::vector<std::vector<int>>& T::equality_constraint_multipliers()
    const {
  dual_vars_.clear();
  for (auto e : supernodal_assemblers_ptr_) {
    dual_vars_.push_back(e->dual_variables());
  }
  return dual_vars_;
}

const std::vector<std::vector<int>>& T::variables() const {
  cliques_.clear();
  for (auto e : supernodal_assemblers_ptr_) {
    cliques_.push_back({});
    auto& c = cliques_.back();
    c = e->variables();
  }
  return cliques_;
}

const std::vector<std::vector<int>>& T::primal_variables() const {
  cliques_.clear();
  for (auto e : supernodal_assemblers_ptr_) {
    cliques_.push_back({});
    auto& c = cliques_.back();
    c = e->primal_variables();
  }
  return cliques_;
}

}  // namespace conex
