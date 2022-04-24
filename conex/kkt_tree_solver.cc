#include "conex/kkt_tree_solver.h"
#include "conex/tree_utils.h"

namespace conex {

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
      : 
        intersections_(intersections),
        subsystems_(cliques_sorted) {}
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

int PickCliqueOrderHelper(const std::vector<KKTSubsystem*>& subsystems,
                          int root_in,
                          bool validate_leaf_nodes,
                          SymmetricMatrix<vector<int>>* intersections_ptr,
                          RootedTree* tree_ptr) {
  auto& tree = *tree_ptr;
  auto& intersections = *intersections_ptr;
  size_t n = subsystems.size();
  Weight edge_weights(intersections, subsystems);
  CONEX_ASSERT(root_in < static_cast<int>(n), "Invalid root node.");

  vector<int> visited(n, 0);

  std::stack<size_t> node_stack;
  int root = root_in;
  if (root < 0) {
    root = 0;
  }

  node_stack.push(root);
  int num_visited = 0;
  while (num_visited < n) {
    size_t active = node_stack.top();
    if (visited.at(active) == 0) {
      visited.at(active) = 1;
      tree.parent.at(active) = -1;
    }

    // Find unvisited neighbor with maximum weight.
    size_t max_weight = 1;
    vector<int> argmax;
    for (size_t i = 0; i < n; i++) {
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
      tree.parent.at(e) = active;
    }

    // Process leaf node.
    if (argmax.size() == 0) {
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
        int final_leaf_position = static_cast<int>(active);
        while (!subsystems.at(final_leaf_position)->is_valid_leaf()) {
          final_leaf_position = tree.parent.at(final_leaf_position);
          if (final_leaf_position == -1) {
            throw std::runtime_error("System is not full rank.");
          }
        }
        if (active != static_cast<int>(final_leaf_position)) {
          tree.SwapPositions(active, final_leaf_position);
        }
      }

      node_stack.pop();
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
  return -1;
}

}


namespace conex {


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
    root->AssembleAndFactor();
  }
  return true;
}

void T::Finalize(const Options& options) {
  RootedTree tree(subsystems_.size());
  SymmetricMatrix<vector<int>> intersections(subsystems_.size());
  PickCliqueOrderHelper(subsystems_, options.root_node, 
                                      options.validate_leaf_nodes,
                                     &intersections, &tree);
  MakeTree(tree.parent, options.check_for_zero_pivots);
}

void T::MakeTreeHelper(const std::vector<int>& parent) {
  CONEX_DEMAND(parent.size() == subsystems_.size(), "Size of parent vector must equal number of subsystems.");
  roots_.clear();
  for (auto s : subsystems_) {
    s->Reset();
  }
  for (size_t i = 0; i < parent.size(); ++i) {
    if (parent[i] >= 0) {
      CONEX_DEMAND(parent[i] != static_cast<int>(i), "Tree is malformed: node cannot be own parent.");
      subsystems_.at(parent[i])->AddChild(subsystems_.at(i));
    } else {
      roots_.push_back(subsystems_.at(i));
    }
  }

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
    }
  }

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
  MakeTreeHelper(parent);
  Assemble();
  int i = 0; 
  for (auto r : subsystems_) {
    if (r->supernodes().size() > 0) {
      Eigen::MatrixXd T = r->supernode_submatrix();
      T = T.cwiseProduct(T);
      bool zero_pivot = T.colwise().sum().minCoeff() == 0;
      if (zero_pivot) {
        index_of_zero_pivot->push_back(i);
      }
    }
    i++;
  }
  return index_of_zero_pivot->size() > 0;
}

void T::MakeTree(const std::vector<int>& parent, bool check_for_zero_pivot) {
  MakeTreeHelper(parent);
  if (check_for_zero_pivot){
    std::vector<int> index_of_zero_pivot;
    if (CheckForZeroPivot(parent, &index_of_zero_pivot)) {
      throw std::runtime_error("Invalid tree: zero pivot detected.");
    }
  }
}

void T::RepairTreeInPlace(std::vector<int>* parent_ptr) {
  auto& parent = *parent_ptr;
  bool regenerate = true;
  std::vector<int> zero_pivot_indices;
  while (regenerate) {
    regenerate = false;
    bool zero_pivot = CheckForZeroPivot(parent, &zero_pivot_indices);
    if (zero_pivot) {
      for (auto i : zero_pivot_indices) {
        int parent_index = parent[i];
        int parent_of_parent = parent[parent_index];
        parent[i] = parent_of_parent;
        parent[parent_index] = i;
        regenerate = true;
      }
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
