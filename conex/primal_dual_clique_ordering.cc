#include "conex/primal_dual_clique_ordering.h"

#include <algorithm>
#include <stack>
#include <map>

#include "conex/clique_ordering_utils.h"
#include "conex/clique_ordering.h"
#include "conex/debug_macros.h"
#include "conex/error_checking_macros.h"

namespace conex {

namespace {
int GetMax(const std::vector<Clique>& cliques) {
  int max = 0;
  for (const auto& c : cliques) {
    for (const auto ci : c) {
      if (ci > max) {
        max = ci;
      }
    }
  }
  return max;
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

int LinearIndex(int i, int j, int n) {
  if (i > j) {
    return j * n + i;
  } else {
    return i * n + j;
  }
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

class Edges {
 public:
  Edges(int n) : primal_intersection(n),  dual_intersection(n), number_of_sink_edges(n, 0),  weights_(n*n, -1), n_(n){}

  SymmetricMatrix<std::vector<int>> primal_intersection;
  SymmetricMatrix<std::vector<int>> dual_intersection;
  std::vector<int> number_of_sink_edges;

  int& operator()(int a, int b) {
    return weights_.at(a*n_ + b);
  }
  const int& operator()(int a, int b) const {
    return weights_.at(a*n_ + b);
  }
 private:
  std::vector<int> weights_;
  int n_ = 0;
};
void WeightedDepthFirstSearchTraversal(int root_in, int num_nodes, const Edges& edge_weights,  std::vector<int>* order_position_to_id, RootedTree* tree_ptr) {
  auto& tree = *tree_ptr;
  int n = num_nodes;
  CONEX_CHECK(root_in < static_cast<int>(n));

  vector<int> visited(n, 0);

  std::stack<int> node_stack;
  int root = root_in;
  if (root < 0) {
    root = 0;
  }
  node_stack.push(root);

  order_position_to_id->clear();
  order_position_to_id->reserve(n);

  while (order_position_to_id->size() < static_cast<size_t>(n)) {
    int active = node_stack.top();

    if (visited.at(active) == 0) {
      order_position_to_id->push_back(active);
      visited.at(active) = 1;
      tree.parent.at(active) = -1;
      tree.height().at(active) = 0;
    }

    // Find unvisited neighbor with maximum weight.
    int max_weight = 0;
    vector<int> argmax;
    for (int i = 0; i < num_nodes; i++) {
      if (i == active || visited.at(i) == 1) {
        continue;
      }

      auto current_weight = edge_weights(active, i);
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
      order_position_to_id->push_back(e);
      visited.at(e) = 1;
      tree.parent.at(e) = active;
      tree.height().at(e) = tree.height().at(active) + 1;
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

  std::reverse(order_position_to_id->begin(), order_position_to_id->end());
}

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

Edges ComputeEdges(const vector<vector<int>>& primal_variables,
                  const vector<vector<int>>& dual_variables) {
  Edges edges(primal_variables.size());
  for (size_t i = 0; i < primal_variables.size(); ++i) {
    for (size_t j = i + 1; j < primal_variables.size(); ++j) {
      IntersectionOfSorted(primal_variables.at(i), primal_variables.at(j), &edges.primal_intersection(i, j));
      IntersectionOfSorted(dual_variables.at(i), dual_variables.at(j), &edges.dual_intersection(i, j));
      size_t dual_intersection = edges.dual_intersection(i, j).size();
      size_t primal_intersection = edges.primal_intersection(i, j).size();

      if (dual_intersection + primal_intersection > 0) {
        int is_i_valid_sink = (primal_variables.at(i).size() + dual_intersection)
                            >= (dual_variables.at(i).size() + primal_intersection);

        int is_j_valid_sink = (primal_variables.at(j).size() + dual_intersection) 
                            >= (dual_variables.at(j).size() + primal_intersection); 
     
        if (dual_intersection + primal_intersection > 0) {
          edges(j, i) = 0;
          edges(i, j) = 0;
        }

        if (is_i_valid_sink) {
          edges(j, i) = dual_intersection + primal_intersection; 
          edges.number_of_sink_edges.at(i)++;
        } 

        if (is_j_valid_sink) {
          edges(i, j) = dual_intersection + primal_intersection; 
          edges.number_of_sink_edges.at(j)++;
        }
      }
    }
  }
  return edges;
}

void DualFillIn(const RootedTree& tree, int num_variables,
            const std::vector<int>& order_position_to_id, 
            const vector<vector<int>>& primal_supernodes,
            vector<std::vector<int>>* supernodes,
            vector<std::vector<int>>* separators) {

  size_t num_cliques = order_position_to_id.size();
  vector<int> weight(num_variables, 0);
  for (auto& id : order_position_to_id) {
    std::vector<int> supernodes_keep;

    for (auto& s : supernodes->at(id)) {
      weight.at(s) += primal_supernodes.at(id).size();
    }

    for (auto& s : supernodes->at(id)) {
      if (weight.at(s) > 0) {
        supernodes_keep.push_back(s);
      } else {
        separators->at(id).push_back(s);
        supernodes->at(tree.parent.at(id)).push_back(s);
      }
    }
    supernodes->at(id) = supernodes_keep;
  }
  for (auto& id : order_position_to_id) {
  DUMP(id);
  DUMP(primal_supernodes.at(id));
  DUMP(supernodes->at(id));
  DUMP(separators->at(id));
  }
}

} // namespace
PrimalDualCliqueTree MakePrimalDualCliqueTree(const vector<vector<int>>& primal_variables,
                                              const vector<vector<int>>& dual_variables) {
  CONEX_CHECK(primal_variables.size() == dual_variables.size());
  int n = primal_variables.size();

  PrimalDualCliqueTree c(n);

  vector<std::vector<int>> primal_sorted = primal_variables;
  Sort(&primal_sorted);
  vector<std::vector<int>> dual_sorted = dual_variables;
  Sort(&dual_sorted);
  Edges edges = ComputeEdges(primal_variables, dual_variables);
  int root = std::distance(edges.number_of_sink_edges.begin(), 
                           std::min_element(edges.number_of_sink_edges.begin(), edges.number_of_sink_edges.end()));

  RootedTree tree(n);
  std::vector<int> clique_id_to_post_order_position(n);
  WeightedDepthFirstSearchTraversal(root, n, edges, 
  &c.clique_id_to_post_order_position, &tree);
  c.clique_id_to_parent = tree.parent;

  vector<vector<int>>& primal_separators = c.primal_separators;
  vector<vector<int>>& primal_supernodes = c.primal_supernodes;
  vector<vector<int>>& dual_separators = c.dual_separators;
  vector<vector<int>>& dual_supernodes = c.dual_supernodes;
  for (int i = 0; i < n; i++) {
    if (c.clique_id_to_parent.at(i) != -1) {
      primal_separators.at(i) = edges.primal_intersection(i, c.clique_id_to_parent.at(i));
      dual_separators.at(i) = edges.dual_intersection(i, c.clique_id_to_parent.at(i));
      std::set_difference(primal_sorted.at(i).begin(),
                          primal_sorted.at(i).end(), primal_separators.at(i).begin(),
                          primal_separators.at(i).end(), std::back_inserter(primal_supernodes.at(i)));
      std::set_difference(dual_sorted.at(i).begin(),
                          dual_sorted.at(i).end(), dual_separators.at(i).begin(),
                          dual_separators.at(i).end(), std::back_inserter(dual_supernodes.at(i)));
    } else {
      primal_supernodes.at(i) = primal_variables.at(i);
      dual_supernodes.at(i) = dual_variables.at(i);
    }
  }
  FillIn(tree, GetMax(primal_variables) + 1, c.clique_id_to_post_order_position, &c.primal_supernodes, &c.primal_separators);
  FillIn(tree, GetMax(dual_variables) + 1, c.clique_id_to_post_order_position, &c.dual_supernodes, &c.dual_separators);
  return c;
}
}  // namespace conex
