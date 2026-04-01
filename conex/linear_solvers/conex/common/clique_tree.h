#pragma once
#include <set>
#include <vector>

namespace conex {

struct CliqueTree {
  std::vector<std::vector<int>> supernodes;
  std::vector<std::vector<int>> separators;
  std::vector<int> node_to_parent;
  std::vector<int> post_order_position_to_clique;

  // Check the running intersection property: for every variable v in
  // clique c, the set of cliques containing v forms a connected subtree.
  // Returns true if the property holds.
  bool CheckRunningIntersectionProperty() const {
    int nc = static_cast<int>(supernodes.size());
    for (int c = 0; c < nc; ++c) {
      std::set<int> vars(supernodes[c].begin(), supernodes[c].end());
      vars.insert(separators[c].begin(), separators[c].end());
      for (int v : vars) {
        bool found_gap = false;
        for (int a = node_to_parent[c]; a >= 0; a = node_to_parent[a]) {
          std::set<int> a_vars(supernodes[a].begin(), supernodes[a].end());
          a_vars.insert(separators[a].begin(), separators[a].end());
          if (a_vars.count(v)) {
            if (found_gap) return false;
            break;
          }
          found_gap = true;
        }
      }
    }
    return true;
  }
};

}  // namespace conex
