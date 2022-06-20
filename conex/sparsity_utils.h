#include <vector>
#include <Eigen/Sparse>

namespace conex {

void PartitionRows(const Eigen::MatrixXd& A,
                   std::vector<std::vector<int>>* row_partition,
                   std::vector<std::vector<int>>* nonzero_columns);

Eigen::MatrixXd Submatrix(const Eigen::MatrixXd& A,
                          const std::vector<int>& rows,
                          const std::vector<int>& cols);
Eigen::MatrixXd Subvector(const Eigen::VectorXd& A,
                          const std::vector<int>& rows);

}  // namespace conex
