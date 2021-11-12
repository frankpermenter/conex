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
  // The inputs are the sizes of the matrices D and the rows of R_i
  // that are non-zero.  The columns of D_i and R_i are equal.
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
  std::vector<std::vector<double*>> seperator_diagonal;

  // TODO(FrankPermenter): Remove all of these members.
  std::vector<int> supernode_size;
  std::vector<std::vector<int>> separators;

  friend int SizeOf(const TriangularMatrixWorkspace& o) {
    int size = 0;
    for (size_t j = 0; j < o.snodes.size(); j++) {
      size += o.SizeOfSupernode(j);
    }
    for (size_t j = 0; j < o.separators.size(); j++) {
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

 private:
  // TODO(FrankPermenter): Remove this method.
  void S_S(int clique, std::vector<double*>*);
  int SizeOfSupernode(int i) const {
    return get_size_aligned(supernode_size.at(i) * supernode_size.at(i));
  }

  int SizeOfSeparator(int i) const {
    return get_size_aligned(supernode_size.at(i) * separators.at(i).size());
  }

  double* LookupAddress(int r, int c);

  int num_columns_;
  std::vector<std::vector<int>> snodes;
  std::vector<int> variable_to_diagonal_block_;
  std::vector<int> variable_to_diagonal_block_position_;
};

}  // namespace conex
