#include "conex/clique_ordering.h"

#include <algorithm>
#include <stack>

#include "conex/clique_ordering_utils.h"
#include "conex/debug_macros.h"
#include "conex/error_checking_macros.h"

namespace conex {

using std::vector;
using Cliques = vector<vector<int>>;

namespace {

int GetRootNode(const std::vector<std::vector<int>>& vars,
                const std::vector<bool>& valid_leaf) {
  int arg_max = 0;
  size_t max = 0;

  if (valid_leaf.size() > 0) {
    for (size_t i = 0; i < vars.size(); i++) {
      if (!valid_leaf.at(i) && vars.at(i).size() > max) {
        arg_max = i;
        max = vars.at(i).size();
      }
    }
    if (max > 0) {
      return arg_max;
    }
  }

  arg_max = 0;
  max = vars.at(0).size();
  for (size_t i = 1; i < vars.size(); i++) {
    if (vars.at(i).size() > max) {
      arg_max = i;
      max = vars.at(i).size();
    }
  }
  return arg_max;
}

int GetMax(const std::vector<Clique>& cliques) {
  int max = cliques.at(0).at(0);
  for (const auto& c : cliques) {
    for (const auto ci : c) {
      if (ci > max) {
        max = ci;
      }
    }
  }
  return max;
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

using Edge = std::pair<int, int>;
vector<int> GetNodeDegrees(int n, const vector<Edge>& edges,
                           const SymmetricMatrix<vector<int>>& intersections) {
  vector<int> weights(n);
  for (auto& w : weights) {
    w = 0;
  }

  for (auto& e : edges) {
    weights[e.first] += intersections(e.first, e.second).size();
    weights[e.second] += intersections(e.first, e.second).size();
  }
  return weights;
}

class Weight {
 public:
  Weight(int n, SymmetricMatrix<vector<int>>& intersections,
         const vector<vector<int>>& cliques_sorted,
         const vector<bool>& valid_leaf)
      : num_nodes_(n),
        intersections_(intersections),
        cliques_sorted_(cliques_sorted),
        valid_leaf_(valid_leaf) {}

  int num_nodes_;
  SymmetricMatrix<vector<int>>& intersections_;
  const vector<vector<int>>& cliques_sorted_;
  const vector<bool>& valid_leaf_;

  size_t get_weight(int active, int i) {
    // Weight is the size of intersection.
    if (intersections_(active, i).size() == 0) {
      IntersectionOfSorted(cliques_sorted_.at(active), cliques_sorted_.at(i),
                           &intersections_(active, i));
    }
    size_t weight = intersections_(active, i).size();
    if (weight > 0) {
      if (valid_leaf_.size() > 0) {
        // Guarantee illegal leaf-nodes obtain largest weight so we
        // process them sooner.
        const int weight_offset = num_nodes_ + 1;
        if (!valid_leaf_.at(i)) {
          weight += weight_offset;
        }
      }
    }
    return weight;
  }
};

int PickCliqueOrderHelper(const std::vector<std::vector<int>>& cliques_sorted,
                          const std::vector<bool>& valid_leaf, int root_in,
                          SymmetricMatrix<vector<int>>* intersections_ptr,
                          vector<vector<int>>* separators,
                          std::vector<int>* order, RootedTree* tree_ptr) {
  auto& tree = *tree_ptr;
  auto& intersections = *intersections_ptr;
  size_t n = cliques_sorted.size();
  Weight edge_weights(n, intersections, cliques_sorted, valid_leaf);
  CONEX_CHECK(root_in < static_cast<int>(n));

  vector<int> visited(n, 0);

  std::stack<size_t> node_stack;
  int root = root_in;
  if (root < 0) {
    root = 0;
  }
  node_stack.push(root);

  using Path = vector<Edge>;
  vector<Path> paths;
  vector<Edge> edges;

  order->clear();
  order->reserve(n);

  while (order->size() < n) {
    size_t active = node_stack.top();

    if (visited.at(active) == 0) {
      order->push_back(active);
      visited.at(active) = 1;
      tree.parent.at(active) = -1;
      tree.height().at(active) = 0;
    }

    // Find unvisited neighbor with maximum weight.
    size_t max_weight = 1;
    vector<int> argmax;
    for (size_t i = 0; i < cliques_sorted.size(); i++) {
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
      separators->at(e) = intersections(active, e);
      node_stack.push(e);
      order->push_back(e);
      visited.at(e) = 1;
      edges.emplace_back(active, e);
      tree.parent.at(e) = active;
      tree.height().at(e) = tree.height().at(active) + 1;
      if (valid_leaf.size() > 0 && !valid_leaf.at(e)) {
        // Heuristic: quit now to increase chance node e
        // is not a leaf node.
        break;
      }
    }

    if (argmax.size() == 0) {
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

  auto weights = GetNodeDegrees(n, edges, intersections);
  int root_node = std::distance(
      weights.begin(),
      std::max_element(weights.begin(), weights.begin() + weights.size()));

  std::reverse(order->begin(), order->end());
  return root_node;
}

void GetCliqueEliminationOrder(const vector<vector<int>>& cliques_sorted,
                               const vector<bool>& valid_leaf, int root,
                               vector<int>* order,
                               vector<vector<int>>* supernodes,
                               vector<vector<int>>* separators,
                               RootedTree* tree) {
  size_t n = cliques_sorted.size();
  order->clear();
  order->resize(n);
  separators->clear();
  separators->resize(n);
  SymmetricMatrix<vector<int>> intersections(n);
  int better_root =
      PickCliqueOrderHelper(cliques_sorted, valid_leaf, root, &intersections,
                            separators, order, tree);

  if (root == -1) {
    order->clear();
    order->resize(n);
    separators->clear();
    separators->resize(n);
    RootedTree tree_i(n);
    PickCliqueOrderHelper(cliques_sorted, valid_leaf, better_root,
                          &intersections, separators, order, &tree_i);
    *tree = tree_i;
  }

  supernodes->resize(n);
  for (auto& e : *order) {
    supernodes->at(e).resize(cliques_sorted.at(e).size() -
                             separators->at(e).size());
    if (supernodes->at(e).size() > 0) {
      std::set_difference(cliques_sorted.at(e).begin(),
                          cliques_sorted.at(e).end(), separators->at(e).begin(),
                          separators->at(e).end(), supernodes->at(e).begin());
    }
  }
}

template <typename T>
auto FindSupernode(const std::vector<int>& separator, const T& b, const T& c,
                   vector<int>* intersection) {
  if (separator.size() == 0) {
    return c;
  }
  // TODO(FrankPermenter): Process separators in elimination order
  // so we do not search over all supernodes.
  for (auto i = b; i != c; ++i) {
    IntersectionOfSorted(separator, *i, intersection);
    if (intersection->size() == separator.size()) {
      return i;
    }
  }
  return c;
}

}  // namespace

void FillIn(const RootedTree& tree, int num_variables,
            const std::vector<int>& order, vector<std::vector<int>>* supernodes,
            vector<std::vector<int>>* separators) {
  std::vector<int> eliminated(num_variables, order.size() + 1);
  int num_cliques = order.size();

  // Detect if variable is a supernode of clique i and
  // clique j.  If so, apply running intersection property
  // to the path from clique i and to clique j:
  //  1) Make a supernode of the clique closest to the root.
  //  2) Make a separator of all other cliques.
  //
  for (size_t i = 0; i < order.size(); i++) {
    for (int v : supernodes->at(order.at(i))) {
      const bool variable_already_eliminated = eliminated.at(v) < num_cliques;
      if (variable_already_eliminated) {
        auto fill_in = PathInForest(order.at(i), eliminated.at(v), tree.parent,
                                    tree.height());
        for (size_t j = 0; j < fill_in.size() - 1; j++) {
          auto e = fill_in.at(j);
          separators->at(e) = UnionOfSorted(separators->at(e), {v});
        }
        eliminated.at(v) = fill_in.back();
      } else {
        eliminated.at(v) = order.at(i);
      }
    }
  }

  supernodes->clear();
  supernodes->resize(num_cliques);
  for (size_t i = 0; i < eliminated.size(); i++) {
    // TODO(FrankPermenter): Remove this check if we refactor
    // to require that require variable set = [0, ..., GetMax(Vars)].
    // As is, variables are only a subset and hence the "eliminated"
    // vector has spurious entries.
    if (eliminated.at(i) < num_cliques) {
      supernodes->at(eliminated.at(i)).push_back(i);
    }
  }
  Sort(separators);
  Sort(supernodes);
}

void PickCliqueOrder(const vector<vector<int>>& cliques_sorted,
                     const vector<bool>& valid_leaf, int root,
                     vector<int>* order, vector<int>* parent_in_tree,
                     vector<vector<int>>* supernodes,
                     vector<vector<int>>* separators) {
  size_t n = cliques_sorted.size();
  RootedTree tree(n);
  GetCliqueEliminationOrder(cliques_sorted, valid_leaf, root, order, supernodes,
                            separators, &tree);
  int num_vars = GetMax(cliques_sorted) + 1;
  FillIn(tree, num_vars, *order, supernodes, separators);

  *parent_in_tree = tree.parent;
}

void PickCliqueOrder(const vector<vector<int>>& cliques_sorted, int root,
                     vector<int>* order, vector<vector<int>>* supernodes,
                     vector<vector<int>>* separators) {
  const vector<bool> valid_leaf{};
  vector<int> parent_in_tree;
  PickCliqueOrder(cliques_sorted, valid_leaf, root, order, &parent_in_tree,
                  supernodes, separators);
}

void PickCliqueOrder(const vector<vector<int>>& cliques_sorted, int root,
                     vector<int>* order, vector<int>* parent_in_tree,
                     vector<vector<int>>* supernodes,
                     vector<vector<int>>* separators) {
  const vector<bool> valid_leaf{};
  PickCliqueOrder(cliques_sorted, valid_leaf, root, order, parent_in_tree,
                  supernodes, separators);
}

void PickCliqueOrder(const vector<vector<int>>& cliques_sorted,
                     const vector<bool>& valid_leaf, int root, RootedTree* tree,
                     vector<vector<int>>* supernodes,
                     vector<vector<int>>* separators) {
  std::vector<int> order;
  GetCliqueEliminationOrder(cliques_sorted, valid_leaf, root, &order,
                            supernodes, separators, tree);
  int num_vars = GetMax(cliques_sorted) + 1;
  FillIn(*tree, num_vars, order, supernodes, separators);
}

CliqueTree MakeCliqueTree(const vector<vector<int>>& cliques,
                          const std::vector<bool>& valid_leaf) {
  CliqueTree clique_tree;

  vector<std::vector<int>> cliques_sorted = cliques;
  Sort(&cliques_sorted);

  PickCliqueOrder(cliques_sorted, valid_leaf, GetRootNode(cliques, valid_leaf),
                  &clique_tree.clique_to_post_order_position,
                  &clique_tree.node_to_parent, &clique_tree.supernodes,
                  &clique_tree.separators);
  return clique_tree;
}

}  // namespace conex
