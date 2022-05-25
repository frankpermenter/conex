#include "conex/clique_ordering.h"

#include <algorithm>
#include <stack>
#include <map>

#include "conex/clique_ordering_utils.h"
#include "conex/debug_macros.h"
#include "conex/error_checking_macros.h"

namespace conex {

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
  Edges(int n) : primal_intersection(n),  dual_intersection(n), number_of_sink_edges(n, 0),  weights_(n*n), n_(n){}

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

void WeightedDepthFirstSearchTraversal(int root_in, int num_nodes, const Edges& edge_weights,  std::vector<int>* order, RootedTree* tree_ptr) {
  auto& tree = *tree_ptr;
  size_t n = num_nodes;
  CONEX_CHECK(root_in < static_cast<int>(n));

  vector<int> visited(n, 0);

  std::stack<size_t> node_stack;
  int root = root_in;
  if (root < 0) {
    root = 0;
  }
  node_stack.push(root);

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
    for (size_t i = 0; i < num_nodes; i++) {
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
      order->push_back(e);
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

  std::reverse(order->begin(), order->end());
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
  int n = primal_variables.size();
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
  DUMP(edges.number_of_sink_edges);
  return edges;
}
PrimalDualCliqueTree MakePrimalDualCliqueTree(const vector<vector<int>>& primal_variables,
                                              const vector<vector<int>>& dual_variables) {
  CONEX_CHECK(primal_variables.size() == dual_variables.size());

  PrimalDualCliqueTree clique_tree;

  vector<std::vector<int>> primal_sorted = primal_variables;
  Sort(&primal_sorted);
  vector<std::vector<int>> dual_sorted = dual_variables;
  Sort(&dual_sorted);
  Edges edges = ComputeEdges(primal_variables, dual_variables);

  int n = primal_variables.size();
  int root = std::distance(edges.number_of_sink_edges.begin(), std::min(edges.number_of_sink_edges.begin(), edges.number_of_sink_edges.end()));
  RootedTree tree(n);
  std::vector<int> clique_id_to_post_order_position(n);
  WeightedDepthFirstSearchTraversal(root, n, edges, 
  &clique_tree.clique_id_to_post_order_position, &tree);
  clique_tree.clique_id_to_parent = tree.parent;

  return clique_tree;

}
}  // namespace conex
