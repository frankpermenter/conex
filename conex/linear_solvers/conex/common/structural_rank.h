#pragma once
#include <Eigen/Sparse>

namespace conex {

// Compute the structural rank of a sparse matrix: the size of a maximum
// matching in the bipartite graph (rows × columns, edge for each nonzero).
// This is an upper bound on the numerical rank for any choice of nonzero values.
int StructuralRank(const Eigen::SparseMatrix<double>& A);

// Remove structurally redundant columns.  Returns a matrix with
// structural rank == number of columns, by keeping one representative
// column per matched column in a maximum matching.
// col_map[new_col] = original column index.
Eigen::SparseMatrix<double> DropStructurallyDependentColumns(
    const Eigen::SparseMatrix<double>& A,
    std::vector<int>* col_map = nullptr);

// Remove structurally redundant rows.  Returns a matrix with
// structural rank == number of rows, by keeping one representative
// row per matched row in a maximum matching.
// row_map[new_row] = original row index.
Eigen::SparseMatrix<double> DropStructurallyDependentRows(
    const Eigen::SparseMatrix<double>& A,
    std::vector<int>* row_map = nullptr);

}  // namespace conex
