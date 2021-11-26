#include "conex/tree_utils.h"
#include "assert.h"
#include <algorithm>
#include <stack>
#include <vector>

using std::vector;
using Clique = vector<int>;

namespace conex {

vector<int> PathInTree(int x, int y, const vector<int>& tree,
                       const vector<int>& depth) {
  vector<int> path;
  while (x != y) {
    if (depth[x] < depth[y]) {
      path.push_back(y);
      y = tree.at(y);
    } else {
      path.push_back(x);
      x = tree.at(x);
    }
  }
  path.push_back(x);
  return path;
}

void IntersectionOfSorted(const vector<int>& v1,
                          const vector<int>& v2, vector<int>* v3) {
  v3->clear();
  std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                        back_inserter(*v3));
}

void Sort(vector<Clique>* path) {
  for (size_t i = 0; i < path->size(); i++) {
    std::sort(path->at(i).begin(), path->at(i).end());
  }
}

vector<int> UnionOfSorted(const vector<int>& x1,
                               const vector<int>& x2) {
  vector<int> y;
  set_union(x1.begin(), x1.end(), x2.begin(), x2.end(), inserter(y, y.end()));
  return y;
}

vector<int> NumberOfChildren(const RootedTree& tree) {
  vector<int> num_children(tree.NumberOfNodes(), 0);
  for (auto p : tree.parent) {
    if (p >= 0) {
      num_children.at(p)++;
    }
  }
  return num_children;
}

vector<int> GetUnvisitedRootNodes(const vector<int>& parent,
                                       const vector<int>& visited) {
  vector<int> nodes;
  for (size_t i = 0; i < parent.size(); i++) {
    if (visited.at(i) == 0 && parent.at(i) == -1) {
      nodes.push_back(i);
    }
  }
  return nodes;
}

vector<int> GetUnvisitedLeafNode(const vector<int>& num_children,
                                 const vector<int>& visited) {
  vector<int> leaf_nodes;
  for (size_t i = 0; i < num_children.size(); i++) {
    if (num_children[i] == 0 && visited[i] == 0) {
      leaf_nodes.push_back(i);
    }
  }
  return leaf_nodes;
}


vector<int> GetChildren(const RootedTree& d, int parent) {
  vector<int> y;
  for (size_t i = 0; i < d.parent.size(); i++) {
    if (d.parent[i] == parent) {
      y.push_back(i);
    }
  }
  return y;
}

vector<int> GetRootNodes(const RootedTree& d) {
  return GetChildren(d, -1);
}

// Visit nodes in a depth-first fashion
// Maintain a stack of paths.  
vector<vector<int>> PartitionIntoPaths(const RootedTree& tree) {
  vector<int> num_children = NumberOfChildren(tree);
  std::stack<vector<int>> path_stack;
  vector<vector<int>> paths;
  auto root = GetRootNodes(tree);
  for (auto& r : root) {

    vector<int> children = GetChildren(tree, r);
    for (auto c : children) {
      path_stack.push(vector<int>{r, c});
    }
  }
  while (path_stack.size() > 0) {
    vector<int> path = path_stack.top();

    vector<int> children = GetChildren(tree, path.back());
    // Add to current path
    while (children.size() == 1) {
      path.push_back(children.back());
      children = GetChildren(tree, path.back()); 
    } 
    // Terminate current path and save.
    path_stack.pop();
    paths.push_back(path);

    for (auto& c : children) {
      vector<int> new_path = {path.back()};
      new_path.push_back(c);
      path_stack.push(new_path);
    }
    
  }
  return paths;
}


}  // namespace conex
