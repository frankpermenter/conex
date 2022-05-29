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
    num_aux_vars += e->number_of_auxiliary_variables();
  }
  return max_number_of_variables_ + num_aux_vars;
};

CONEX_ID T::AddEqualityConstraint(const EqualityConstraints& x,
                                  const std::vector<int>& variables) {
  CONEX_DEMAND(Validate(variables) == CONEX_SUCCESS,
               "Failed to add constraint.");

  equality_constraints_.emplace_back(x.A_, x.b_, variables);

  supernodal_assemblers_ptr_.push_back(&equality_constraints_.back());

  return equality_constraints_.size() - 1;
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

const std::vector<std::vector<int>>& T::variables() const {
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

}  // namespace conex
