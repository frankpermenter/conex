#pragma once
#include <vector>

#include "conex/common/model.h"

namespace conex {

// Specifies a custom clique tree topology and maps constraints to cliques.
// Used with MakeSolver(problem, tree_spec, config) for problems with
// known structure (LQR, stochastic opt, network flow, etc.).
class TreeSpec {
 public:
  // Add a clique.  parent = -1 for root.  Returns clique id.
  int AddClique(int parent = -1) {
    int id = static_cast<int>(parents_.size());
    parents_.push_back(parent);
    return id;
  }

  // Assign a constraint to a clique.
  void Assign(ConstraintId constraint, int clique) {
    if (static_cast<int>(assignments_.size()) <= constraint)
      assignments_.resize(constraint + 1, -1);
    assignments_[constraint] = clique;
  }

  int num_cliques() const { return static_cast<int>(parents_.size()); }
  int parent(int clique) const { return parents_.at(clique); }
  int clique_of(ConstraintId id) const { return assignments_.at(id); }

  const std::vector<int>& parents() const { return parents_; }
  const std::vector<int>& assignments() const { return assignments_; }

 private:
  std::vector<int> parents_;
  std::vector<int> assignments_;  // constraint_id → clique_id
};

}  // namespace conex
