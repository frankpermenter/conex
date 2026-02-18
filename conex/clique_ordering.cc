#include "conex/clique_ordering.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <stack>
#include <vector>

#include "conex/clique_ordering_utils.h"
#include "conex/error_checking_macros.h"
#include <Eigen/OrderingMethods>
#include <Eigen/Sparse>

namespace conex {

using std::vector;
using Cliques = vector<vector<int>>;

namespace {

vector<int> is_empty(const vector<std::vector<int>>& vect) {
  vector<int> y(vect.size());
  for (size_t i = 0; i < y.size(); i++) {
    y[i] = vect[i].size() == 0;
  }
  return y;
}

int GetRootNode(const std::vector<std::vector<int>>& vars,
                const std::vector<int>& valid_leaf) {
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

int GetRootNode(const std::vector<std::vector<int>>& vars,
                const std::vector<std::vector<int>>& dual_vars) {
  int arg_max = 0;

  size_t max = dual_vars.at(0).size();
  for (size_t i = 1; i < dual_vars.size(); i++) {
    if (dual_vars.at(i).size() > max) {
      arg_max = i;
      max = dual_vars.at(i).size();
    }
  }
  if (max > 0) {
    return arg_max;
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
         const vector<int>& valid_leaf)
      : num_nodes_(n),
        intersections_(intersections),
        cliques_sorted_(cliques_sorted),
        valid_leaf_(valid_leaf) {}

  int num_nodes_;
  SymmetricMatrix<vector<int>>& intersections_;
  const vector<vector<int>>& cliques_sorted_;
  const vector<int>& valid_leaf_;

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
                          const std::vector<int>& valid_leaf, int root_in,
                          SymmetricMatrix<vector<int>>* intersections_ptr,
                          vector<vector<int>>* separators,
                          std::vector<int>* order, RootedTree* tree_ptr) {
  auto& tree = *tree_ptr;
  auto& intersections = *intersections_ptr;
  size_t n = cliques_sorted.size();
  Weight edge_weights(n, intersections, cliques_sorted, valid_leaf);
  CONEX_ASSERT(root_in < static_cast<int>(n), "Invalid input.");

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
      tree.height.at(active) = 0;
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
      tree.height.at(e) = tree.height.at(active) + 1;
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
                               const vector<int>& valid_leaf, int root,
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

RootedTree BuildTreeWithHeights(const std::vector<int>& parent) {
  RootedTree tree(parent.size());
  tree.parent = parent;
  for (size_t i = 0; i < parent.size(); ++i) {
    int node = static_cast<int>(i);
    int depth = 0;
    while (tree.parent.at(node) != -1) {
      depth++;
      node = tree.parent.at(node);
    }
    tree.height.at(i) = depth;
  }
  return tree;
}

void GetCliqueEliminationOrderAmd(const vector<vector<int>>& cliques_sorted,
                                  vector<int>* order,
                                  vector<vector<int>>* supernodes,
                                  vector<vector<int>>* separators,
                                  RootedTree* tree) {
  const int n = static_cast<int>(cliques_sorted.size());
  order->assign(n, 0);
  separators->assign(n, {});

  SymmetricMatrix<vector<int>> intersections(n);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(n * 4);
  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, 1.0);
  }
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      IntersectionOfSorted(cliques_sorted.at(i), cliques_sorted.at(j),
                           &intersections(i, j));
      if (!intersections(i, j).empty()) {
        triplets.emplace_back(i, j, 1.0);
        triplets.emplace_back(j, i, 1.0);
      }
    }
  }

  Eigen::SparseMatrix<double> intersection_graph(n, n);
  intersection_graph.setFromTriplets(triplets.begin(), triplets.end());

  std::vector<int> remaining(n, 1);
  for (int step = 0; step < n; ++step) {
    int best = -1;
    int best_degree = n + 1;
    for (int i = 0; i < n; ++i) {
      if (!remaining.at(i)) {
        continue;
      }
      int degree = 0;
      for (Eigen::SparseMatrix<double>::InnerIterator it(intersection_graph, i);
           it; ++it) {
        const int j = static_cast<int>(it.row());
        if (j != i && remaining.at(j)) {
          degree++;
        }
      }
      if (degree < best_degree) {
        best_degree = degree;
        best = i;
      }
    }
    CONEX_DEMAND(best >= 0, "Min-degree ordering failed.");
    order->at(step) = best;
    remaining.at(best) = 0;
  }

  std::vector<int> position(n, -1);
  for (int i = 0; i < n; ++i) {
    position.at(order->at(i)) = i;
  }

  std::vector<std::vector<int>> adjacency(n);
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (!intersections(i, j).empty()) {
        adjacency.at(i).push_back(j);
        adjacency.at(j).push_back(i);
      }
    }
  }

  std::vector<int> parent(n, -1);
  for (const int active : *order) {
    std::vector<int> nplus;
    for (const int nei : adjacency.at(active)) {
      if (position.at(nei) > position.at(active)) {
        nplus.push_back(nei);
      }
    }
    if (nplus.empty()) {
      continue;
    }

    auto best_parent = nplus.front();
    size_t best_weight = intersections(active, best_parent).size();
    for (const int candidate : nplus) {
      const size_t w = intersections(active, candidate).size();
      if (w > best_weight) {
        best_parent = candidate;
        best_weight = w;
      }
    }
    parent.at(active) = best_parent;
    separators->at(active) = intersections(active, best_parent);

    for (size_t i = 0; i < nplus.size(); ++i) {
      for (size_t j = i + 1; j < nplus.size(); ++j) {
        const int a = nplus.at(i);
        const int b = nplus.at(j);
        if (std::find(adjacency.at(a).begin(), adjacency.at(a).end(), b) ==
            adjacency.at(a).end()) {
          adjacency.at(a).push_back(b);
          adjacency.at(b).push_back(a);
        }
      }
    }
  }

  *tree = BuildTreeWithHeights(parent);

  supernodes->assign(n, {});
  for (int e = 0; e < n; ++e) {
    supernodes->at(e).resize(cliques_sorted.at(e).size() -
                             separators->at(e).size());
    if (!supernodes->at(e).empty()) {
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

std::vector<std::vector<int>> FindMaximalCliquesImplicitFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    CliqueTree* implicit_tree = nullptr) {
  std::vector<int> unique_vars;
  for (const auto& row : row_supports) {
    for (int v : row) {
      if (v >= 0) {
        unique_vars.push_back(v);
      }
    }
  }
  std::sort(unique_vars.begin(), unique_vars.end());
  unique_vars.erase(std::unique(unique_vars.begin(), unique_vars.end()),
                    unique_vars.end());
  const int n = static_cast<int>(unique_vars.size());
  if (n == 0) {
    if (implicit_tree != nullptr) {
      implicit_tree->post_order_position_to_clique.clear();
      implicit_tree->node_to_parent.clear();
      implicit_tree->supernodes.clear();
      implicit_tree->separators.clear();
    }
    return {};
  }

  std::map<int, int> global_to_compact;
  for (int i = 0; i < n; ++i) {
    global_to_compact.emplace(unique_vars.at(i), i);
  }

  std::vector<std::vector<int>> supports_compact;
  supports_compact.reserve(row_supports.size());
  for (const auto& row : row_supports) {
    std::vector<int> support;
    support.reserve(row.size());
    for (int v : row) {
      const auto it = global_to_compact.find(v);
      if (it != global_to_compact.end()) {
        support.push_back(it->second);
      }
    }
    std::sort(support.begin(), support.end());
    support.erase(std::unique(support.begin(), support.end()), support.end());
    if (!support.empty()) {
      supports_compact.push_back(std::move(support));
    }
  }

  const int words = (n + 63) / 64;
  std::vector<std::uint64_t> adj_bits(static_cast<size_t>(n) * words, 0);
  auto row_bits = [&](int i) {
    return &adj_bits[static_cast<size_t>(i) * words];
  };
  auto has_edge = [&](int i, int j) {
    const auto* ri = row_bits(i);
    const std::uint64_t mask = std::uint64_t{1} << (j & 63);
    return (ri[j >> 6] & mask) != 0;
  };
  auto set_edge_symmetric = [&](int i, int j) {
    auto* ri = row_bits(i);
    auto* rj = row_bits(j);
    ri[j >> 6] |= (std::uint64_t{1} << (j & 63));
    rj[i >> 6] |= (std::uint64_t{1} << (i & 63));
  };
  auto set_diag = [&](int i) {
    auto* ri = row_bits(i);
    ri[i >> 6] |= (std::uint64_t{1} << (i & 63));
  };

  for (const auto& support : supports_compact) {
    for (size_t i = 0; i < support.size(); ++i) {
      const int u = support.at(i);
      set_diag(u);
      for (size_t j = i + 1; j < support.size(); ++j) {
        set_edge_symmetric(u, support.at(j));
      }
    }
  }
  for (int i = 0; i < n; ++i) {
    set_diag(i);
  }

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(n * 4));
  for (int i = 0; i < n; ++i) {
    const auto* ri = row_bits(i);
    for (int w = 0; w < words; ++w) {
      std::uint64_t bits = ri[w];
      while (bits) {
        const int b = __builtin_ctzll(bits);
        const int j = (w << 6) + b;
        if (j < n) {
          triplets.emplace_back(i, j, 1.0);
        }
        bits &= (bits - 1);
      }
    }
  }

  Eigen::SparseMatrix<double> graph(n, n);
  graph.setFromTriplets(triplets.begin(), triplets.end());
  graph.makeCompressed();

  std::vector<int> order(static_cast<size_t>(n), 0);
  bool valid_perm = true;
  {
    Eigen::COLAMDOrdering<int> ordering;
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> perm;
    ordering(graph, perm);
    if (perm.indices().size() != n) {
      valid_perm = false;
    } else {
      std::vector<char> seen(static_cast<size_t>(n), 0);
      for (int i = 0; i < n; ++i) {
        const int p = perm.indices()[i];
        if (p < 0 || p >= n || seen.at(static_cast<size_t>(p))) {
          valid_perm = false;
          break;
        }
        seen.at(static_cast<size_t>(p)) = 1;
        order.at(static_cast<size_t>(i)) = p;
      }
    }
  }
  if (!valid_perm) {
    for (int i = 0; i < n; ++i) {
      order.at(static_cast<size_t>(i)) = i;
    }
  }

  std::vector<int> pos(static_cast<size_t>(n), -1);
  for (int i = 0; i < n; ++i) {
    pos.at(static_cast<size_t>(order.at(static_cast<size_t>(i)))) = i;
  }

  std::vector<std::vector<int>> candidate_by_var(static_cast<size_t>(n));
  std::map<std::vector<int>, int> clique_to_best_rep;
  std::vector<std::vector<int>> candidates;
  candidates.reserve(static_cast<size_t>(n));
  for (int k = 0; k < n; ++k) {
    const int v = order.at(static_cast<size_t>(k));
    std::vector<int> later;
    const auto* rv = row_bits(v);
    for (int w = 0; w < words; ++w) {
      std::uint64_t bits = rv[w];
      while (bits) {
        const int b = __builtin_ctzll(bits);
        const int u = (w << 6) + b;
        if (u < n && u != v && pos.at(static_cast<size_t>(u)) > k) {
          later.push_back(u);
        }
        bits &= (bits - 1);
      }
    }

    for (size_t i = 0; i < later.size(); ++i) {
      for (size_t j = i + 1; j < later.size(); ++j) {
        const int a = later.at(i);
        const int b = later.at(j);
        if (!has_edge(a, b)) {
          set_edge_symmetric(a, b);
        }
      }
    }

    std::vector<int> clique;
    clique.reserve(later.size() + 1);
    clique.push_back(v);
    clique.insert(clique.end(), later.begin(), later.end());
    std::sort(clique.begin(), clique.end());
    clique.erase(std::unique(clique.begin(), clique.end()), clique.end());

    if (!clique.empty()) {
      candidate_by_var.at(static_cast<size_t>(v)) = clique;
      auto it = clique_to_best_rep.find(clique);
      if (it == clique_to_best_rep.end() ||
          pos.at(static_cast<size_t>(v)) <
              pos.at(static_cast<size_t>(it->second))) {
        clique_to_best_rep[clique] = v;
      }
      candidates.push_back(std::move(clique));
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const std::vector<int>& a, const std::vector<int>& b) {
              if (a.size() != b.size()) {
                return a.size() > b.size();
              }
              return a < b;
            });
  candidates.erase(std::unique(candidates.begin(), candidates.end()),
                   candidates.end());

  std::vector<std::vector<int>> cliques;
  cliques.reserve(candidates.size());
  for (const auto& c : candidates) {
    bool subset = false;
    for (const auto& mclq : cliques) {
      if (mclq.size() < c.size()) {
        continue;
      }
      if (std::includes(mclq.begin(), mclq.end(), c.begin(), c.end())) {
        subset = true;
        break;
      }
    }
    if (!subset) {
      cliques.push_back(c);
    }
  }
  if (cliques.empty()) {
    for (int i = 0; i < n; ++i) {
      cliques.push_back({i});
    }
  }

  auto remap_to_global = [&](std::vector<std::vector<int>>* sets) {
    for (auto& vals : *sets) {
      for (int& v : vals) {
        v = unique_vars.at(static_cast<size_t>(v));
      }
    }
  };
  remap_to_global(&cliques);

  if (implicit_tree != nullptr) {
    // Reuse the standard clique-intersection DFS tree construction.
    *implicit_tree =
        MakeCliqueTree(cliques, {}, CLIQUE_TREE_METHOD_WEIGHTED_DFS);
    if (!cliques.empty()) {
      auto supernodes_check = implicit_tree->supernodes;
      auto separators_check = implicit_tree->separators;
      RootedTree tree = BuildTreeWithHeights(implicit_tree->node_to_parent);
      const int num_vars = GetMax(cliques) + 1;
      const size_t fill_in =
          FillIn(tree, num_vars, implicit_tree->post_order_position_to_clique,
                 &supernodes_check, &separators_check);
      if (fill_in > 0) {
        throw std::runtime_error(
            "Implicit clique tree requires fill-in; refusing tree.");
      }
    }
  }
  return cliques;
}

}  // namespace

size_t FillIn(const RootedTree& tree, int num_variables,
              const std::vector<int>& order,
              vector<std::vector<int>>* supernodes,
              vector<std::vector<int>>* separators) {
  std::vector<int> eliminated(num_variables);
  int num_cliques = order.size();
  for (auto& e : eliminated) {
    e = num_cliques + 1;
  }

  // Detect if variable is a supernode of clique i and
  // clique j.  If so, apply running intersection property
  // to the path from clique i and to clique j:
  //  1) Make a supernode of the clique closest to the root.
  //  2) Make a separator of all other cliques.
  //
  size_t total_fill = 0;
  for (size_t i = 0; i < order.size(); i++) {
    for (int v : supernodes->at(order.at(i))) {
      const bool variable_already_eliminated = eliminated.at(v) < num_cliques;
      if (variable_already_eliminated) {
        auto fill_in = PathInForest(order.at(i), eliminated.at(v), tree.parent,
                                    tree.height);
        for (size_t j = 0; j < fill_in.size() - 1; j++) {
          auto e = fill_in.at(j);
          const size_t size_before = separators->at(e).size();
          separators->at(e) = UnionOfSorted(separators->at(e), {v});
          total_fill += separators->at(e).size() - size_before;
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
  return total_fill;
}

void PickCliqueOrder(const vector<vector<int>>& cliques_sorted,
                     const vector<int>& valid_leaf, int root,
                     vector<int>* order, vector<vector<int>>* supernodes,
                     vector<vector<int>>* separators,
                     vector<vector<vector<int>>>* post_order_pointer) {
  size_t n = cliques_sorted.size();
  RootedTree tree(n);
  GetCliqueEliminationOrder(cliques_sorted, valid_leaf, root, order, supernodes,
                            separators, &tree);
  int num_vars = GetMax(cliques_sorted) + 1;
  FillIn(tree, num_vars, *order, supernodes, separators);

  if (post_order_pointer) {
    int count = 0;
    auto& post_order = *post_order_pointer;
    for (auto& e : *separators) {
      std::vector<int> intersection;
      auto end = supernodes->end();
      auto ptr = FindSupernode(e, supernodes->begin(), end, &intersection);
      if (ptr != end) {
        int match_index = std::distance(supernodes->begin(), ptr);
        post_order.at(match_index).push_back(intersection);
      }
      count++;
    }
  }
}

void PickCliqueOrder(const vector<vector<int>>& cliques_sorted, int root,
                     vector<int>* order, vector<vector<int>>* supernodes,
                     vector<vector<int>>* separators,
                     vector<vector<vector<int>>>* post_order_pointer) {
  const vector<int> valid_leaf{};
  PickCliqueOrder(cliques_sorted, valid_leaf, root, order, supernodes,
                  separators, post_order_pointer);
}

namespace {

void PickCliqueOrder(const vector<vector<int>>& cliques_sorted,
                     const vector<int>& valid_leaf, int root,
                     vector<int>* post_order_position_to_clique,
                     vector<int>* parent_in_tree,
                     vector<vector<int>>* supernodes,
                     vector<vector<int>>* separators, int method) {
  size_t n = cliques_sorted.size();
  RootedTree tree(n);
  if (method == CLIQUE_TREE_METHOD_AMD) {
    GetCliqueEliminationOrderAmd(cliques_sorted, post_order_position_to_clique,
                                 supernodes, separators, &tree);
  } else {
    GetCliqueEliminationOrder(cliques_sorted, valid_leaf, root,
                              post_order_position_to_clique, supernodes,
                              separators, &tree);
  }
  int num_vars = GetMax(cliques_sorted) + 1;
  FillIn(tree, num_vars, *post_order_position_to_clique, supernodes,
         separators);

  *parent_in_tree = tree.parent;
}

}  // namespace
CliqueTree MakeCliqueTree(const vector<vector<int>>& cliques,
                          const std::vector<int>& valid_leaf, int method) {
  CliqueTree clique_tree;

  vector<std::vector<int>> cliques_sorted = cliques;
  Sort(&cliques_sorted);

  PickCliqueOrder(cliques_sorted, valid_leaf, GetRootNode(cliques, valid_leaf),
                  &clique_tree.post_order_position_to_clique,
                  &clique_tree.node_to_parent, &clique_tree.supernodes,
                  &clique_tree.separators, method);
  return clique_tree;
}

CliqueTree MakePrimalDualCliqueTree(
    const vector<vector<int>>& cliques,
    const std::vector<std::vector<int>>& dual_variables, int method) {
  CliqueTree clique_tree;

  vector<std::vector<int>> cliques_sorted = cliques;
  Sort(&cliques_sorted);

  PickCliqueOrder(cliques_sorted, is_empty(dual_variables),
                  GetRootNode(cliques, dual_variables),
                  &clique_tree.post_order_position_to_clique,
                  &clique_tree.node_to_parent, &clique_tree.supernodes,
                  &clique_tree.separators, method);
  return clique_tree;
}

size_t CountCliqueTreeFillIn(const vector<vector<int>>& cliques,
                             const std::vector<int>& valid_leaf, int method) {
  if (cliques.empty()) {
    return 0;
  }
  vector<std::vector<int>> cliques_sorted = cliques;
  Sort(&cliques_sorted);

  vector<int> order;
  vector<vector<int>> supernodes;
  vector<vector<int>> separators;
  RootedTree tree(cliques_sorted.size());
  if (method == CLIQUE_TREE_METHOD_AMD) {
    GetCliqueEliminationOrderAmd(cliques_sorted, &order, &supernodes,
                                 &separators, &tree);
  } else {
    GetCliqueEliminationOrder(cliques_sorted, valid_leaf,
                              GetRootNode(cliques_sorted, valid_leaf), &order,
                              &supernodes, &separators, &tree);
  }
  int num_vars = GetMax(cliques_sorted) + 1;
  return FillIn(tree, num_vars, order, &supernodes, &separators);
}

CliqueTree MakeCliqueTreeImplicitFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    std::vector<std::vector<int>>* maximal_cliques_out) {
  CliqueTree tree;
  auto cliques = FindMaximalCliquesImplicitFromRowSupports(row_supports, &tree);
  if (maximal_cliques_out != nullptr) {
    *maximal_cliques_out = std::move(cliques);
  }
  return tree;
}

}  // namespace conex
