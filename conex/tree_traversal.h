#pragma once
#include <omp.h>

#include "conex/tree_utils.h"

namespace conex {
void TraverseFromRoot(const RootedTree& d);
void TraverseFromLeafs(const RootedTree& d);

class TreeTraversalBase {
 public:
  TreeTraversalBase(const RootedTree* tree, int max_threads = 4) : tree_ptr_(tree) {}

  void TraverseFromRoot() {
    auto d = *tree_ptr_;
    bool parallelism_enabled = true;
    auto num_children = NumberOfChildren(d);
    std::vector<int> root_nodes;
    vector<int> visited(d.NumberOfNodes(), 0);
    omp_set_num_threads(num_threads_);
    do {
      root_nodes = GetUnvisitedRootNodes(d, visited);
      int N = root_nodes.size();
#pragma omp parallel for if (parallelism_enabled)
      for (int n = 0; n < N; ++n) {
        VisitDepthFirst(root_nodes[n], &visited);
      }
    } while (root_nodes.size() > 0);
  }

  void TraverseFromLeaves()  {
    auto d = *tree_ptr_;
    bool parallelism_enabled = true;
    auto num_children = NumberOfChildren(d);
    std::vector<int> leaf_nodes;
    vector<int> visited(d.NumberOfNodes(), 0);
    omp_set_num_threads(num_threads_);
    do {
      leaf_nodes = GetUnvisitedLeafNode(num_children, visited);
      int N = leaf_nodes.size();
#pragma omp parallel for if (parallelism_enabled)
      for (int n = 0; n < N; ++n) {
        VisitPostOrder(leaf_nodes[n], &visited, &num_children);
      }
    } while (leaf_nodes.size() > 0);
  }

  virtual ~TreeTraversalBase() = default;

 private:
  virtual int DoNodeOperation(int node) = 0;
  void VisitPostOrder(int starting_node, std::vector<int>* visited, 
                      std::vector<int>* num_children) {
    int node = starting_node;
    auto d = *tree_ptr_;
    while (1) {
      int parent_node = d.parent.at(node);
      (*visited)[node] = 1;
      DoNodeOperation(node);
      if (parent_node >= 0) {
        (*num_children)[parent_node]--;
        if ((*num_children)[parent_node] == 0) {
          node = parent_node;
        } else {
          return;
        }
      } else {
        return;
      }
    }
  }
  void VisitDepthFirst(int starting_node, vector<int>* visited) {
    auto d = *tree_ptr_;
    std::stack<size_t> node_stack;
    node_stack.push(starting_node);
    while (node_stack.size() > 0) {
      int node = node_stack.top();
      node_stack.pop();
      visited->at(node) = 1;
      DoNodeOperation(node);
      for (int i = 0; i < d.NumberOfNodes(); i++) {
        if (d.parent.at(i) == node && visited->at(i) == 0) {
          node_stack.push(i);
        }
      }
    }
  }
 protected:
  const RootedTree* tree_ptr_ = nullptr;
  const int num_threads_ = 1;
};


}  // namespace conex
