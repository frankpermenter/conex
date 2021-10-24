#include "conex/sparsity_utils.h"
#include "conex/debug_macros.h"
namespace conex {

Eigen::MatrixXd Submatrix(const Eigen::MatrixXd& A,
                          const std::vector<int>& rows,
                          const std::vector<int>& cols) {
  Eigen::MatrixXd M(rows.size(), cols.size());
  for (size_t i = 0; i < rows.size(); i++) {
    for (size_t j = 0; j < cols.size(); j++) {
      M(i, j) = A(rows[i], cols[j]);
    }
  }
  return M;
}

Eigen::MatrixXd Subvector(const Eigen::VectorXd& A,
                          const std::vector<int>& rows) {
  Eigen::MatrixXd M(rows.size(), 1);
  for (int i = 0; i < rows.size(); i++) {
    M(i, 0) = A(rows[i], 0);
  }
  return M;
}

void PartitionRows(const Eigen::MatrixXd& A,
                   std::vector<std::vector<int>>* row_partition,
                   std::vector<std::vector<int>>* nonzero_columns) {
  nonzero_columns->reserve(A.rows());
  row_partition->reserve(A.rows());
  nonzero_columns->clear();
  row_partition->clear();
  for (int i = 0; i < A.rows(); i++) {
    row_partition->push_back({i});
  }

  int count = 0;
  for (auto& row : *row_partition) {
    nonzero_columns->push_back({});
    for (auto& i : row) {
      for (int j = 0; j < A.cols(); j++) {
        if (A(i, j) != 0) {
          nonzero_columns->back().push_back(j);
        }
      }
    }
  }
}

}  // namespace conex
