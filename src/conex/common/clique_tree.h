#pragma once
#include <map>
#include <set>
#include <vector>

namespace conex {

struct CliqueTree {
  std::vector<std::vector<int>> supernodes;
  std::vector<std::vector<int>> separators;
  std::vector<int> node_to_parent;
  std::vector<int> post_order_position_to_clique;

  // Check the running intersection property via two conditions:
  // 1) Each clique's separator is contained in the parent clique.
  // 2) Each variable is a supernode of exactly one clique.
  bool CheckRunningIntersectionProperty() const {
    int nc = static_cast<int>(supernodes.size());

    // Check 1: separator ⊆ parent clique.
    for (int c = 0; c < nc; ++c) {
      int p = node_to_parent[c];
      if (p < 0) continue;  // root has no parent
      std::set<int> parent_vars(supernodes[p].begin(), supernodes[p].end());
      parent_vars.insert(separators[p].begin(), separators[p].end());
      for (int v : separators[c]) {
        if (!parent_vars.count(v)) return false;
      }
    }

    // Check 2: each variable is a supernode of exactly one clique.
    std::map<int, int> supernode_count;
    for (int c = 0; c < nc; ++c) {
      for (int v : supernodes[c]) {
        supernode_count[v]++;
      }
    }
    for (const auto& [v, count] : supernode_count) {
      if (count != 1) return false;
    }

    return true;
  }
};

}  // namespace conex
