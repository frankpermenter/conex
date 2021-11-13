#pragma once
#include <numeric>
#include <vector>

#include <Eigen/Dense>

#include "conex/debug_macros.h"
#include "conex/memory_utils.h"

namespace conex {

using Clique = std::vector<int>;

struct TriangularMatrixWorkspace {
  // Inputs: a list of cliques satisfying the running intersection property
  // given in elimination order. The first N_i elements are supernodes of
  // clique i, where N = supernode_size.at(i)
  TriangularMatrixWorkspace(const std::vector<std::vector<int>>& cliques,
                            const std::vector<int>& supernode_size);

  // We store a triangular matrix T using a collection of square matrices
  // D_i on the diagonal and matrices R_i below the diagonal.
  //
  //  T = D_1
  //      R_1  D_2
  //      R_1  R_2  D_3
  //      R_1  R_2  R_3  D_4
  //
  //
  // The inputs are the sizes of the matrices D_i and a list of
  // rows on which the i^{th} block column of T is nonzero.
  // This list satisfies:
  //
  //   T(non_zero_rows.at(i).at(k), columns of D_i) = R_i.row(k)
  //
  std::vector<std::vector<int>> GetCliques(
      const std::vector<int>& diagonal_size,
      const std::vector<std::vector<int>>& non_zero_rows) {
    int cnt = 0;
    std::vector<std::vector<int>> cliques(diagonal_size.size());
    for (size_t e = 0; e < diagonal_size.size(); e++) {
      for (size_t i = 0; i < diagonal_size.size(); i++) {
        cliques.at(e).push_back(cnt++);
      }
      for (auto s : non_zero_rows.at(e)) {
        cliques.at(e).push_back(s);
        if (s < cnt) {
          throw std::runtime_error(
              "Nonzero rows of R_i must be below the diagonal block D_i.");
        }
      }
    }
    return cliques;
  }
  TriangularMatrixWorkspace(const std::vector<int>& diagonal_size,
                            const std::vector<std::vector<int>>& non_zero_rows)
      : TriangularMatrixWorkspace(GetCliques(diagonal_size, non_zero_rows),
                                  diagonal_size) {}
  std::vector<Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>> diagonal;
  std::vector<Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>> off_diagonal;

  // For each off diagonal matrix R_i, scatter_destination_pointers.at(i)
  // contains the addresses of the non-zero elements of T that
  // are updated by the following operation:
  //
  //  T += lower_tri(  full(R_i) * full(R'_i)  )
  //
  // where full(R_i) denotes the matrix of size equal to T non-zero
  // only on the R_i block.
  std::vector<std::vector<double*>> scatter_destination_pointers;

  // TODO(FrankPermenter): Remove all of these members.
  std::vector<int> block_column_size_;
  std::vector<std::vector<int>> non_zero_rows_;

  friend int SizeOf(const TriangularMatrixWorkspace& o) {
    int size = 0;
    for (int j = 0; j < o.num_block_columns(); j++) {
      size += o.SizeOfSupernode(j);
    }
    for (int j = 0; j < o.num_block_columns(); j++) {
      size += o.SizeOfSeparator(j);
    }
    return size;
  }

  friend void Initialize(TriangularMatrixWorkspace* o, double* data_start);

  // A cache of IntersectionOfSupernodeAndSeparator. The first records
  // the j for which IntersectionOfSupernodeAndSeparator(i+1, j) is nonempty.
  // The second returns the output.
  std::vector<std::vector<int>> column_intersections;
  std::vector<std::vector<std::vector<std::pair<int, int>>>>
      intersection_position;

  std::vector<Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>>> llts;

  // Needed for solving linear systems.
  mutable std::vector<Eigen::VectorXd> temporaries;

  int num_columns() const { return num_columns_; }
  int num_block_columns() const { return num_block_columns_; }
  double coeff(int i, int j) const;

  Eigen::MatrixXd MakeDenseMatrix() const {
    Eigen::MatrixXd y(num_columns(), num_columns());
    for (int i = 0; i < num_columns(); i++) {
      for (int j = 0; j < num_columns(); j++) {
        y(i, j) = coeff(i, j);
      }
    }
    return y;
  }

  std::vector<int> variable_to_diagonal_block_;
  std::vector<int> variable_to_diagonal_block_position_;

 private:
  // TODO(FrankPermenter): Remove this method.
  void S_S(int clique, std::vector<double*>*);
  int SizeOfSupernode(int i) const {
    return get_size_aligned(block_column_size_.at(i) *
                            block_column_size_.at(i));
  }

  int SizeOfSeparator(int i) const {
    return get_size_aligned(block_column_size_.at(i) *
                            non_zero_rows_.at(i).size());
  }

  double* LookupAddress(int r, int c);

  int num_columns_;
  int num_block_columns_;
  std::vector<int> variable_to_entering_block_column_;
};

}  // namespace conex
