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

int T::SizeOfKKTSystem() const { return new_dual_variable_start_; };

namespace {
struct SparseEqualities {
  Eigen::MatrixXd nonzero_columns;
  std::vector<int> variables;
};

SparseEqualities RemoveNonZeroColumns(const EqualityConstraints& x,
                                      const std::vector<int> vars) {
  SparseEqualities y;
  std::vector<int> columns_keep;
  for (int i = 0; i < x.constraint_matrix().cols(); i++) {
    if (x.constraint_matrix().col(i).norm() > 0) {
      columns_keep.push_back(i);
    }
  }
  y.nonzero_columns.resize(x.constraint_matrix().rows(), columns_keep.size());
  int i = 0;
  for (auto& nonzero_col_index : columns_keep) {
    y.nonzero_columns.col(i++) = x.constraint_matrix().col(nonzero_col_index);
    y.variables.push_back(vars.at(nonzero_col_index));
  }
  return y;
}
}  // namespace

CONEX_ID T::AddEqualityConstraint(const EqualityConstraints& x_in,
                                  const std::vector<int>& vars_in) {
  SparseEqualities y = RemoveNonZeroColumns(x_in, vars_in);
  EqualityConstraints x(y.nonzero_columns, x_in.affine_term());
  std::vector<int> variables = y.variables;
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

  std::unique_ptr<Constraint> pointer =
      std::make_unique<EqualityConstraints>(std::move(x));
  CONEX_CHECK(pointer->number_of_variables() ==
              static_cast<int>(variables.size()));
  pointer->set_variable_indices(variables);
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
  for (const auto& e : clique_assemblers()) {
    dual_vars_.push_back(e->dual_variables());
  }
  return dual_vars_;
}

const std::vector<std::vector<int>>& T::variables() const {
  cliques_.clear();
  for (auto e : clique_assemblers()) {
    cliques_.push_back({});
    auto& c = cliques_.back();
    c = e->variables();
  }
  return cliques_;
}

std::vector<SupernodalAssemblerBase*> T::clique_assemblers() {
  std::vector<SupernodalAssemblerBase*> supernodal_assemblers_pointers_;
  for (auto& q : quadratic_costs_) {
    supernodal_assemblers_pointers_.push_back(&q);
  }
  for (auto& q : constraint_assemblers_) {
    supernodal_assemblers_pointers_.push_back(&q);
  }
  for (auto& q : equality_constraints_.assemblers) {
    supernodal_assemblers_pointers_.push_back(&q);
  }
  return supernodal_assemblers_pointers_;
}

std::vector<const SupernodalAssemblerBase*> T::clique_assemblers() const {
  std::vector<const SupernodalAssemblerBase*> supernodal_assemblers_pointers_;
  for (auto& q : quadratic_costs_) {
    supernodal_assemblers_pointers_.push_back(&q);
  }
  for (auto& q : constraint_assemblers_) {
    supernodal_assemblers_pointers_.push_back(&q);
  }
  for (auto& q : equality_constraints_.assemblers) {
    supernodal_assemblers_pointers_.push_back(&q);
  }
  return supernodal_assemblers_pointers_;
}
namespace {

void IncrementSubvector(std::vector<int>* y, const std::vector<int>& indices) {
  for (auto i : indices) {
    y->at(i)++;
  }
}
class PrimalVariables {
 public:
  PrimalVariables(ConstraintManager* kkt)
      : degree(kkt->GetNumberOfVariables(), 0), kkt_(kkt) {
    for (const auto& c : kkt->clique_assemblers()) {
      if (c->is_positive_definite()) {
        cliques_of_G.push_back(c->primal_variables());
        clique_assemblers_of_G.push_back(c);
        IncrementSubvector(&degree, c->primal_variables());
        CONEX_DEMAND(
            c->dual_variables().size() == 0,
            "Auxiliary variables only supported for equality constraints");
      }
    }
  }
  bool ValidateStrictConvexity() {
    for (const auto d : degree) {
      CONEX_DEMAND(
          d > 0,
          "Primal schur-complement matrix is not positive definite.  "
          "Please presolve variables using equality constraints or add "
          "inequalities/quadratic penalty terms.");
    }
    return true;
  }
  void MakeStrictlyConvex(double eps) {
    int i = 0;
    for (const auto d : degree) {
      if (d == 0) {
        kkt_->AddQuadraticCost(Eigen::MatrixXd::Identity(1, 1) * eps, {i});
      }
      i++;
    }
  }
  std::vector<std::vector<int>> cliques_of_G;
  std::vector<SupernodalAssemblerBase*> clique_assemblers_of_G;
  std::vector<int> degree;
  ConstraintManager* kkt_;
};

}  // namespace
void MakeObjectiveStrictlyConvex(ConstraintManager* x, double eps) {
  PrimalVariables(x).MakeStrictlyConvex(eps);
}
}  // namespace conex
