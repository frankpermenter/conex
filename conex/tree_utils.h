#pragma once
#include <array>
#include <stack>
#include <vector>
#include "debug_macros.h"

using std::array;
using std::vector;
constexpr int N = 5;

namespace conex {

std::vector<int> UnionOfSorted(const std::vector<int>& x1,
                               const std::vector<int>& x2);
void Sort(std::vector<std::vector<int>>* path);

void IntersectionOfSorted(const std::vector<int>& v1,
                          const std::vector<int>& v2, std::vector<int>* v3);

struct RootedTree {
  RootedTree(int number_of_nodes)
      : parent(number_of_nodes), height(number_of_nodes) {}

  RootedTree() : parent({}), height({}) {}
  std::vector<int> parent;
  std::vector<int> height;
  int NumberOfNodes() const { return parent.size(); };
};

vector<int> PathInTree(int x, int y, const std::vector<int>& tree,
                       const std::vector<int>& height);

std::vector<int> NumberOfChildren(const RootedTree& tree);

std::vector<int> GetUnvisitedRootNodes(const RootedTree& d,
                                       const vector<int>& visited);

vector<int> GetUnvisitedLeafNode(const std::vector<int>& num_children,
                                 const std::vector<int>& visited);


//class ParseTreeData {
//  struct SimpleTriangularMatrixData {
//    std::vector<int> entering_block;
//    std::vector<int> exiting_block;
//    int num_blocks = 0;
//  };

//  struct DirectSumTriangularMatrixData {
//    vector<int> children;
//  };
//  // Root = DS
//  //
//  //
//  //     C
//  //  D  D  D
//  //  D  D  D
//  //  D  D  D
//
// public:
//  ParseTreeData(RootedTree* tree, std::vector<vector<int>>* cliques) : tree_(tree),
//    cliques_(cliques) {}
//
//  void BuildCompressedTree() {
//    std::vector<int> num_children = NumberOfChildren(*tree_);
//    std::vector<int> merged_tree_parent = GetRootNodes(*tree_);
//    std::stack<int> root_stack;
//    for (auto r: merged_tree_parent) {
//      root_stack.push(r);
//    }
//    vector<vector<int>> paths;
//    vector<int> path_parents;
//    while (root_stack.size() > 0) {
//      int root = root_stack.top();
//      root_stack.pop();
//
//      vector<int> children = GetChildren(*tree_, root);
//      for (auto& c : children) {
//        std::vector<int> path;
//        path.push_back(root);
//        path.push_back(c);
//        auto descendants = GetChildren(*tree_, c);
//        while (descendants.size() == 1) {
//          path.push_back(descendants.back());
//          descendants = GetChildren(*tree_, path.back());
//        }
//        if (descendants.size() > 1) {
//          for (auto s : descendants) {
//            root_stack.push(s);
//          }
//        }
//        paths.push_back(path);
//        path_parents.push_back(root);
//      }
//    }
//  }
//
// private:
//  RootedTree* tree_;
//  vector<vector<int>>* cliques_;
//  vector<SimpleTriangularMatrixData> simple_matrices_;
//  vector<DirectSumTriangularMatrixData> direct_sum_;
//};

}  // namespace conex

