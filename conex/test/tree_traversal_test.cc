#include <stdio.h>
#include <vector>
#include <Eigen/Dense>
#include <omp.h>

#include "conex/tree_traversal.h"

using conex::RootedTree;
using Eigen::MatrixXd;
using std::vector;

std::vector<Eigen::LLT<MatrixXd>> FactorInParallel(
    const std::vector<MatrixXd>& matrices) {
  bool parallelism_enabled = false;
  int N = matrices.size();
  std::vector<Eigen::LLT<MatrixXd>> llts(N);
#pragma omp parallel for if (parallelism_enabled)
  for (int n = 0; n < N; ++n) {
    llts.at(n).compute(matrices.at(n));
  }
  return llts;
}

MatrixXd RandomPSD(int n) {
  MatrixXd X = MatrixXd::Random(n, n);
  return X * X.transpose();
}

void DoFactorInParallel() {
  int N = 4;
  int order = 40;
  vector<MatrixXd> matrices(N);
  for (int i = 0; i < N; i++) {
    matrices.at(i) = RandomPSD(order);
  }
  FactorInParallel(matrices);
}

class NodePrinter final : public conex::TreeTraversalBase {
 public:
  NodePrinter(RootedTree* tree) : TreeTraversalBase(tree) {}
 private:
  int DoNodeOperation(int node) override {
    std::cout << "\nParent: " << tree_ptr_->parent.at(node) << "Node: " << node; 
    return node;
  }
};

int main(int argc, char** argv) {
  //  DoFactorInParallel();

  RootedTree tree(3);
  NodePrinter node_printer(&tree);

  tree.parent = vector<int>{-1, 0, 0};
  tree.height = vector<int>{0, 1, 1};
  node_printer.TraverseFromRoot();
  node_printer.TraverseFromLeaves();

  tree.parent = vector<int>{-1, 0, 1, -1};
  tree.height = vector<int>{0, 1, 2, 0};
  node_printer.TraverseFromRoot();
  node_printer.TraverseFromLeaves();

  tree.parent = vector<int>{-1, 0, 1, 0, 3, 4, 5, 6};
  tree.height = vector<int>{0, 1, 2, 1, 2, 3, 4, 5};
  node_printer.TraverseFromRoot();
  node_printer.TraverseFromLeaves();


  return 0;
}
