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

#if 0
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

const std::vector<std::vector<int>>& T::equality_constraint_multipliers() const {
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
#endif

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
  //equality_constraint_multipliers_.emplace_back(x.A_.rows());
  equality_constraints_.emplace_back(x.A_, x.b_, variables);
  supernodal_assemblers_ptr_.push_back(&equality_constraints_.back());

  //equality_constraints_.emplace_back(x.A_.col(0), x.b_, variables.at(0));
  // for (int i = 1; i < variables.size(); i++) {
  //  equality_constraints_.emplace_back(x.A_.col(i), 0*x.b_, variables.at(i));
  //}

  //supernodal_assemblers_ptr_.push_back(&equality_constraints_.back());
  return equality_constraints_.size() - 1;
}
}  // namespace conex
