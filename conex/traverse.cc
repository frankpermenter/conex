#include "traverse.h"
#include <stack>

namespace conex {

namespace {
std::vector<int> NumberOfChildren(const RootedTree& tree) {
  std::vector<int> num_children(tree.NumberOfNodes(), 0);
  for (auto p : tree.parent) {
    if (p >= 0) {
      num_children.at(p)++;
    }
  }
  return num_children;
}

vector<int> GetUnvisitedLeafNode(const std::vector<int>& num_children, 
                         const std::vector<int>& visited) {
  std::vector<int> leaf_nodes;
  for (size_t i = 0; i < num_children.size(); i++) {
    if (num_children[i] == 0 && visited[i] == 0) {
      leaf_nodes.push_back(i);
    }
  }
  return leaf_nodes;
}

void VisitPostOrder(int starting_node, 
           const RootedTree& d, 
           vector<int>* visited,
           vector<int>* num_children) {
  int node = starting_node;
  while (1) {
    int parent_node = d.parent.at(node);
    (*visited)[node] = 1;
    printf("Node %d  Parent %d\n", node, parent_node);
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

void VisitDepthFirst(int starting_node, 
           const RootedTree& d, 
           vector<int>* visited) {
  std::stack<size_t> node_stack;
  node_stack.push(starting_node);
  while (node_stack.size() > 0) {
    int node = node_stack.top();
    node_stack.pop();
    visited->at(node) = 1;
    printf("Node %d", node);
    for (size_t i = 0; i < d.NumberOfNodes(); i++) {
      if (d.parent.at(i) == node && visited->at(i) == 0) {
        node_stack.push(i);
      }
    }
  } 
}

std::vector<int> GetUnvisitedRootNodes(const RootedTree& d, 
           const vector<int>& visited) {
  vector<int> nodes;
  for (size_t i = 0; i < d.NumberOfNodes(); i++) {
    if (visited.at(i) == 0 && d.parent.at(i) == -1) {
      nodes.push_back(i);
    }
  }
  return nodes;
}


} // namespace

void TraverseFromLeafs(const RootedTree& d) {
  bool parallelism_enabled = true;
  auto num_children = NumberOfChildren(d);
  std::vector<int> leaf_nodes;
  vector<int> visited(d.NumberOfNodes(), 0);
  do {
    leaf_nodes = GetUnvisitedLeafNode(num_children, visited);
    int N = leaf_nodes.size();
   #pragma omp parallel for if(parallelism_enabled)
    for(int n = 0; n < N; ++n) { 
      VisitPostOrder(leaf_nodes[n], d, &visited, &num_children);
    }
  } while (leaf_nodes.size() > 0);
}

void TraverseFromRoot(const RootedTree& d) {
  bool parallelism_enabled = true;
  auto num_children = NumberOfChildren(d);
  std::vector<int> root_nodes;
  vector<int> visited(d.NumberOfNodes(), 0);
  do {
    root_nodes = GetUnvisitedRootNodes(d, visited);
    int N = root_nodes.size();
   #pragma omp parallel for if(parallelism_enabled)
    for(int n = 0; n < N; ++n) { 
      VisitDepthFirst(root_nodes[n], d, &visited);
    }
  } while (root_nodes.size() > 0);
}






} // namespace conex
