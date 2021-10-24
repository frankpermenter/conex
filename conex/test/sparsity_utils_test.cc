#include "conex/sparsity_utils.h"
#include "conex/debug_macros.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {
using Eigen::MatrixXd;
using std::vector;

GTEST_TEST(SparsityUtils, PartitionRows) {
  MatrixXd A(4, 5);
  A << 1, 1, 0, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 3, 3, 4, 0, 0, 0, 4;

  vector<vector<int>> rows;
  vector<vector<int>> cols;
  PartitionRows(A, &rows, &cols);
  EXPECT_EQ(rows.size(), A.rows());
  EXPECT_EQ(cols.size(), A.rows());

  for (size_t i = 0; i < rows.size(); i++) {
    MatrixXd Ai = Submatrix(A, rows.at(i), cols.at(i));
    MatrixXd Ai_expected = MatrixXd::Ones(1, 2) * (i + 1);
    EXPECT_EQ(Ai.rows(), Ai_expected.rows());
    EXPECT_EQ(Ai.cols(), Ai_expected.cols());
    EXPECT_EQ((Ai - Ai_expected).norm(), 0);
  }
}

}  // namespace conex
