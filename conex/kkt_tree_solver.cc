#include "conex/kkt_tree_solver.h"

#include "conex/tree_utils.h"

namespace conex {

namespace {
class DistanceToRootRecursion{
 public:
  DistanceToRootRecursion(const std::vector<int>& parent) : parent_(parent),
  distance_(parent.size(), -1) {}
  int ComputeDistanceToRootHelper(int i) {
    if (distance_[i] >= 0) {
      return distance_[i];
    } else {
     int distance_of_new = 0;
      if (parent_.at(i) != -1) {
        distance_of_new = 1 + ComputeDistanceToRootHelper(parent_.at(i)); 
      }
      distance_[i] = distance_of_new;
      return distance_[i];
    }
  }
  std::vector<int> Compute() {
    for (size_t i = 0; i < parent_.size(); i++) {
      ComputeDistanceToRootHelper(i);
    }
    return distance_;
  }
 private:
  std::vector<int> parent_;
  std::vector<int> distance_;
};

std::vector<int> ComputeDistanceToRoot(const std::vector<int>& parent) {
  std::vector<int> distance_to_root;
  return DistanceToRootRecursion(parent).Compute();
}

void FillIn(std::vector<int> system_to_parent, int num_variables,
            std::vector<KKTSubsystem*>* systems) {

  std::vector<int> system_to_distance_to_root = ComputeDistanceToRoot(system_to_parent);
  std::vector<int> eliminated(num_variables, -1);

  // Detect if variable is a supernode of clique i and
  // clique j.  If so, apply running intersection property
  // to the path from clique i and to clique j:
  //  1) Make a supernode of the clique closest to the root.
  //  2) Make a separator of all other cliques.
  for (size_t i = 0; i < systems->size(); ++i) {
    for (int v : systems->at(i)->supernodes()) {
      const bool variable_already_eliminated = eliminated.at(v) > -1;
      if (variable_already_eliminated) {
        auto path_in_tree = PathInForest(i, eliminated.at(v), system_to_parent, 
                                                         system_to_distance_to_root);
        for (size_t j = 0; j < path_in_tree.size() - 1; j++) {
          auto e = path_in_tree.at(j);
          systems->at(e)->AddSeparator(v);
        }
        eliminated.at(v) = path_in_tree.back();
      } else {
        eliminated.at(v) = i;
      }
    }
  }

  for (auto& s : *systems) {
    s->SetSupernodes({});
  }

  for (size_t i = 0; i < eliminated.size(); i++) {
    if (eliminated.at(i) != -1) {
      systems->at(eliminated.at(i))->AddSupernode(i);
    }
  }
}

void IntersectionOfSorted(const std::vector<int>& v1,
                          const std::vector<int>& v2, std::vector<int>* v3) {
  v3->clear();
  std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                        back_inserter(*v3));
}

int LinearIndex(int i, int j, int n) {
  if (i > j) {
    return j * n + i;
  } else {
    return i * n + j;
  }
}

int GetUnvisited(const std::vector<int>& x) {
  int cnt = 0;
  for (auto xi : x) {
    if (xi == 0) {
      return cnt;
    }
    cnt++;
  }
  return -1;
}
template <typename T>
class SymmetricMatrix {
 public:
  SymmetricMatrix(int n) : n_(n), data_(n * n) {}
  vector<int>& operator()(int a, int b) {
    return data_.at(LinearIndex(a, b, n_));
  }
  const vector<int>& operator()(int a, int b) const {
    return data_.at(LinearIndex(a, b, n_));
  }
  int n_;
  vector<T> data_;
};

class Weight {
 public:
  Weight(SymmetricMatrix<vector<int>>& intersections,
         const vector<KKTSubsystem*>& cliques_sorted)
      : intersections_(intersections), subsystems_(cliques_sorted) {}
  int num_nodes_;
  SymmetricMatrix<vector<int>>& intersections_;
  const vector<KKTSubsystem*>& subsystems_;

  size_t get_weight(int active, int i) {
    // Weight is the size of intersection.
    if (intersections_(active, i).size() == 0) {
      IntersectionOfSorted(subsystems_.at(active)->shared_variables(),
                           subsystems_.at(i)->shared_variables(),
                           &intersections_(active, i));
    }
    return intersections_(active, i).size();
  }
};

/* 
Visit nodes of clique intersection graph using weighted DFS.
*/
int PickCliqueOrderHelper(const std::vector<KKTSubsystem*>& subsystems,
                          int root_in, bool validate_leaf_nodes,
                          SymmetricMatrix<vector<int>>* intersections_ptr,
                          RootedTree* tree_ptr) {
  auto& tree = *tree_ptr;
  auto& intersections = *intersections_ptr;
  int n = subsystems.size();
  Weight edge_weights(intersections, subsystems);
  CONEX_ASSERT(root_in < n, "Invalid root node.");

  vector<int> visited(n, 0);
  std::stack<int> node_stack;
  int root = root_in;
  if (root < 0) {
    root = 0;
  }

  node_stack.push(root);
  int num_visited = 0;
  while (num_visited < n) {
    int active = node_stack.top(); 
    if (visited.at(active) == 0) {
      visited.at(active) = 1;
      num_visited++;
      tree.parent.at(active) = -1;
    }

    // Find unvisited neighbor with maximum weight.
    size_t max_weight = 1;
    vector<int> argmax;
    for (int i = 0; i < n; i++) {
      if (i == active || visited.at(i) == 1) {
        continue;
      }

      auto current_weight = edge_weights.get_weight(active, i);
      if (current_weight >= max_weight) {
        if (current_weight > max_weight) {
          argmax.clear();
          max_weight = current_weight;
        }
        argmax.push_back(i);
      }
    }

    for (auto e : argmax) {
      node_stack.push(e);
      visited.at(e) = 1;
      num_visited++;
      tree.parent.at(e) = active;
    }

    // Process leaf node.
    if (argmax.size() == 0) {
      node_stack.pop();
      // If node is invalid leaf node, move it up the
      // tree until a valid leaf is reached.  This
      // leads to the following transformation:
      //
      //     R            I*
      //    I  V          I
      //    I  V   =>     I
      //   *I  V          R
      //                  V
      //                  V
      //                  V
      if (validate_leaf_nodes) {
        int final_leaf_position = active;
        while (!subsystems.at(final_leaf_position)->is_valid_leaf()) {
          final_leaf_position = tree.parent.at(final_leaf_position);
          if (final_leaf_position == -1) {
            throw std::runtime_error("System is not full rank.");
          }
        }
        if (active != final_leaf_position) {
          tree.SwapPositions(active, final_leaf_position);
        }
      }

      if (node_stack.size() == 0) {
        auto node = GetUnvisited(visited);
        if (node == -1) {
          break;
        } else {
          node_stack.push(node);
        }
      }
    }
  }
  DUMP(tree.parent);
  return -1;
}

}  // namespace 


using T = SymmetricLinearSystemTreeSolver;

void T::DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                       bool in_original_order) const {
  if (in_original_order) {
    Eigen::PermutationMatrix<-1> P(number_of_variables());
    P.indices() = Eigen::Map<const Eigen::VectorXi>(
        variable_to_elimination_position_.data(), number_of_variables());
    b = P * b;
  }

  for (auto root : roots_) {
    root->ApplyInverseOfLeftFactor(b);
    root->ApplyInverseOfRightFactor(b);
  }

  if (in_original_order) {
    Eigen::PermutationMatrix<-1> P(number_of_variables());
    P.indices() = Eigen::Map<const Eigen::VectorXi>(
        variable_to_elimination_position_.data(), number_of_variables());
    b = P.transpose() * b;
  }
}

void T::DoAssemble() {
  for (auto root : roots_) {
    root->Assemble();
  }
}

bool T::DoFactor() {
  for (auto root : roots_) {
    root->Factor();
  }
  return true;
}

void T::Finalize(const Options& options) {
  RootedTree tree(subsystems_.size());
  SymmetricMatrix<vector<int>> intersections(subsystems_.size());
  PickCliqueOrderHelper(subsystems_, options.root_node,
                        options.validate_leaf_nodes, &intersections, &tree);
  Finalize(tree.parent, options.check_for_zero_pivots);
}

void T::SetEliminationTree(const std::vector<int>& parent) {
  roots_.clear();
  for (auto s : subsystems_) {
    s->Reset();
  }
  for (size_t i = 0; i < parent.size(); ++i) {
    if (parent[i] >= 0) {
      CONEX_DEMAND(parent[i] != static_cast<int>(i),
                   "Tree is malformed: node cannot be own parent.");
      subsystems_.at(parent[i])->AddChild(subsystems_.at(i));
    } else {
      roots_.push_back(subsystems_.at(i));
    }
  }
}

void T::FinalizeHelper(const std::vector<int>& parent) {
  CONEX_DEMAND(parent.size() == subsystems_.size(),
               "Size of parent vector must equal number of subsystems.");

  SetEliminationTree(parent);
  // Set supernodes from parent.
  for (size_t i = 0; i < parent.size(); ++i) {
    if (parent[i] >= 0) {
      std::vector<int> v1 = subsystems_.at(parent[i])->shared_variables();
      std::vector<int> v2 = subsystems_.at(i)->shared_variables();
      std::sort(v1.begin(), v1.end());
      std::sort(v2.begin(), v2.end());
      std::vector<int> separators;
      std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                            std::back_inserter(separators));
      subsystems_.at(i)->SetSeparators(separators);

      std::vector<int> supernodes;
      std::set_difference(v2.begin(), v2.end(), separators.begin(),
                          separators.end(), std::back_inserter(supernodes));
      subsystems_.at(i)->SetSupernodes(supernodes);
    } else {
      std::vector<int> v2 = subsystems_.at(i)->shared_variables();
      std::sort(v2.begin(), v2.end());
      subsystems_.at(i)->SetSupernodes(v2);
      subsystems_.at(i)->SetSeparators({});
    }
  }

 FillIn(parent, number_of_variables(), &subsystems_);

  // Post-order
  variable_to_elimination_position_.resize(number_of_variables());
  int first = 0;
  for (auto r : roots_) {
    r->ComputePostOrdering(first, &variable_to_elimination_position_);
  }
  for (auto s : subsystems_) {
    s->SetVariableOrdering(variable_to_elimination_position_);
  }
}

bool T::CheckForZeroPivot(const std::vector<int>& parent,
                          std::vector<int>* index_of_zero_pivot) {
  index_of_zero_pivot->clear();
  FinalizeHelper(parent);
  Assemble();
  int i = 0;
  for (auto r : subsystems_) {
    if (r->supernodes().size() > 0) {
      Eigen::MatrixXd T = r->supernode_submatrix().selfadjointView<Eigen::Lower>();
      T = T.transpose() * T;
      bool zero_pivot = T.colwise().sum().minCoeff() == 0;
      if (zero_pivot) {
        index_of_zero_pivot->push_back(i);
      }
    }
    i++;
  }
  return index_of_zero_pivot->size() > 0;
}

void T::Finalize(const std::vector<int>& parent, bool check_for_zero_pivot) {
  FinalizeHelper(parent);
  if (check_for_zero_pivot) {
    std::vector<int> index_of_zero_pivot;
    if (CheckForZeroPivot(parent, &index_of_zero_pivot)) {
      throw std::runtime_error("Invalid tree: zero pivot detected.");
    }
  }
}

int T::number_of_variables() const {
  int max = 0;
  for (auto s : subsystems_) {
    const auto& sn = s->supernodes();
    if (sn.size() > 0) {
      double max_s = *std::max_element(sn.begin(), sn.end());
      if (max_s > max) {
        max = max_s;
      }
    }
  }
  return max + 1;
}

Eigen::MatrixXd T::DoKKTMatrix(bool permute_to_elimination_order) const {
  int num_vars = number_of_variables();
  Eigen::MatrixXd M(num_vars, num_vars);
  M.setZero();
  for (auto root : roots_) {
    root->MakeKKTMatrix(&M);
    M = M.selfadjointView<Eigen::Lower>();
  }
  if (permute_to_elimination_order) {
    return M;
  }
  Eigen::PermutationMatrix<-1> P(number_of_variables());
  P.indices() = Eigen::Map<const Eigen::VectorXi>(
      variable_to_elimination_position_.data(), number_of_variables());
  return P.transpose() * M * P;
}

void T::AddSubsystem(KKTSubsystem* system) { subsystems_.push_back(system); }

}  // namespace conex
