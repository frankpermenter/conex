#include "conex/constraint_manager.h"

#include <numeric>

#include "conex/clique_ordering.h"
#include "conex/debug_macros.h"

namespace conex {
using std::vector;
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
using std::vector;
int T::SizeOfKKTSystem() const { return new_dual_variable_start_; };

std::vector<std::vector<int>> T::variables() const {
  vector<std::vector<int>> cliques_;
  for (auto e : supernodal_assemblers_ptr_) {
    cliques_.push_back({});
    auto& c = cliques_.back();
    c = e->variables();
  }
  return cliques_;
}

const std::vector<std::vector<int>>& T::equality_constraint_multipliers()
    const {
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
  CONEX_CHECK(max_number_of_variables_ > 0);
  CONEX_CHECK(static_cast<int>(variables.size()) == x.A_.cols());
  if (!IsUnique(max_number_of_variables_, variables)) {
    return CONEX_FAILURE;
  }
  equality_constraints_.data.push_back(x);
  equality_constraints_.dual_variables.emplace_back(x.A_.rows());
  std::iota(equality_constraints_.dual_variables.back().begin(),
            equality_constraints_.dual_variables.back().end(),
            new_dual_variable_start_);
  new_dual_variable_start_ +=
      equality_constraints_.dual_variables.back().size();
  equality_constraints_.variables.push_back(variables);

  return equality_constraints_.data.size() - 1;
}

void T::PartitionEqualityConstraints() {
  if (equality_constraints_.data.size() == 0) {
    return;
  }
  int i = 0;

  vector<vector<int>> primal_cliques;
  for (const auto& c : clique_assemblers()) {
    if (c->is_positive_definite()) {
      primal_cliques.push_back(c->variables());
    }
  }
  CliqueTree tree = MakeCliqueTree(primal_cliques);
  for (auto& x : equality_constraints_.data) {
    auto& variables = equality_constraints_.variables.at(i);
    PartitionEqualityConstraint(x.A_, x.b_, variables, &tree,
                                equality_constraints_.dual_variables.at(i));
    i++;
  }
}

void T::PartitionEqualityConstraint(const Eigen::MatrixXd& A,
                                    const Eigen::MatrixXd& b,
                                    const vector<int>& variables,
                                    const CliqueTree* primal_tree,
                                    const vector<int>& multipliers) {
  std::vector<std::vector<int>> variable_groups;
  size_t total_found = 0;
  for (auto& s : primal_tree->supernodes) {
    bool found = false;
    for (auto& v : variables) {
      if (std::find(s.begin(), s.end(), v) != s.end()) {
        total_found++;
        if (!found) {
          variable_groups.push_back(std::vector<int>{v});
        } else {
          variable_groups.back().push_back(v);
        }
        found = true;
      }
    }
  }
  CONEX_CHECK(total_found == variables.size());

  auto columns_of_A = [A, variables, variable_groups](int group_number) {
    const auto& cols = variable_groups.at(group_number);
    Eigen::MatrixXd y(A.rows(), cols.size());
    int i = 0;
    for (auto& c : cols) {
      int index = std::distance(
          variables.begin(), std::find(variables.begin(), variables.end(), c));
      y.col(i) = A.col(index);
      ++i;
    }
    return y;
  };

  equality_constraints_.assemblers.emplace_back(
      columns_of_A(0), b, variable_groups.at(0), multipliers);
  supernodal_assemblers_ptr_.push_back(
      &equality_constraints_.assemblers.back());
  for (size_t i = 1; i < variable_groups.size(); i++) {
    equality_constraints_.assemblers.emplace_back(
        columns_of_A(i), b * 0, variable_groups.at(i), multipliers);
    supernodal_assemblers_ptr_.push_back(
        &equality_constraints_.assemblers.back());
  }
}

void T::InitializeWorkspace(const SolverConfiguration& config) {
  if (config.kkt_solver != CONEX_KKT_SOLVER_CG) {
    PartitionEqualityConstraints();
  }
  auto workspaces = workspace();
  auto size = SizeOf(workspaces);
  if (size > workspace_memory_.size()) {
    workspace_memory_.resize(size);
  }
  Initialize(&workspaces, workspace_memory_.data());
}

}  // namespace conex
