#include "conex/block_triangular_operations.h"
#include "conex/tree_traversal.h"

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::pair;
using std::vector;
using T = BlockTriangularOperations;

namespace {
void DoScalarOffDiagUpdate(TriangularMatrixWorkspace* X,
                           const MatrixXd& non_zero_rows,
                           int exiting_block_column) {
  int index = 0;
  const auto& temp = non_zero_rows;
  const auto& s_s = X->scatter_destination_pointers[exiting_block_column];
  for (int k = 0; k < temp.cols(); k++) {
    for (int j = k; j < temp.cols(); j++) {
      if (X->variable_to_diagonal_block_[X->non_zero_rows_[exiting_block_column]
                                                          [k]] !=
          X->variable_to_diagonal_block_[X->non_zero_rows_[exiting_block_column]
                                                          [j]]) {
        *s_s[index] -= temp.col(k).dot(temp.col(j));
      }
      index++;
    }
  }
}

void DoScalarDiagUpdate(TriangularMatrixWorkspace* X,
                        const MatrixXd& non_zero_rows,
                        int exiting_block_column) {
  int index = 0;
  const auto& temp = non_zero_rows;
  const auto& s_s = X->scatter_destination_pointers[exiting_block_column];
  for (int k = 0; k < temp.cols(); k++) {
    for (int j = k; j < temp.cols(); j++) {
      if (X->variable_to_diagonal_block_[X->non_zero_rows_[exiting_block_column]
                                                          [k]] ==
          X->variable_to_diagonal_block_[X->non_zero_rows_[exiting_block_column]
                                                          [j]]) {
        *s_s[index] -= temp.col(k).dot(temp.col(j));
      }
      index++;
    }
  }
}

} // namespace 

class ParallelCholesky : TreeTraversalBase {
 public:
  ParallelCholesky(TriangularMatrixWorkspace* matrix, int max_processors) : TreeTraversalBase(&matrix->clique_tree_, max_processors), 
    X(matrix) {
      auto& llts = X->llts;
      if (llts.size() > 0) {
        llts.clear();
      }
    }
  void Factor() {  TreeTraversalBase::TraverseFromLeaves(); }
 private:
  int DoNodeOperation(int node) override { 
    int i = node;
    if (X->diagonal[i].size() > 0) {
      X->llts.emplace_back(X->diagonal[i]);
      if (X->llts.back().info() != Eigen::Success) {
        return false;
      }
    } else {
      // Dummy decomposition. Needed to make Eigen's Lapack interface happy.
      MatrixXd x(1, 1);
      x(0) = 1;
      X->llts.emplace_back(x);
    }

    if (X->off_diagonal[i].size() > 0) {
      X->llts.back().matrixL().solveInPlace(X->off_diagonal[i]);
      auto& temp = X->off_diagonal[i];

      DoScalarDiagUpdate(X, temp, i);
      DoScalarOffDiagUpdate(X, temp, i);
    }
    return 0;
  }
  TriangularMatrixWorkspace* X;
};

bool T::ParallelBlockCholeskyInPlace(TriangularMatrixWorkspace* X, int max_threads) {
  ParallelCholesky cholesky(X, max_threads);
  cholesky.Factor();
  return true;
}

} // namespace conex
