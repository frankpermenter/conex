#include <vector>

#include <Eigen/Dense>

#include "conex/debug_macros.h"

namespace conex {
// A square lower-triangular matrix whose rows and columns are partitioned into
// sets C_0, C_1, ..., C_N whose scalar entries satisfy the following
// property:
//
//  (P1) If (i, j) is non-zero, then (i, k) is nonzero for all j <= k <= i.
//  (P2) If (i, j) is non-zero and i \in C_m, then (k, j) is nonzero for all k
//  \in C_m satisfying k < i.
//
//
// Valid Examples:
//
//   * *                * *               *
//   * *                * *               * *
//       * *            * * * *               *
//       * *            * * * *               * *
//   * * * * * *            * * * *       * * * * * *
//   * * * * * *            * * * *           * * * *
//       (A)               (B)               (C)
//
//
// Invalid Examples:
//
//   * *                    *
//   * *                      *
//       * *                   *
//       * *                   * *
//   * *    * *                * * * *
//   * *    * *            * * * * * *
//
//   (Bottom two rows      (Bottom left corner
//    violates P1)          violates P2).
//
//
// To construct such a matrix, we take a list v of triplets
// {(i, j), s}, indicating that s new rows of block row i
// are non-zero starting at the beginning of block column j
//
// Inputs for the example matrices A, B, C are:
//
//  v_A = (0, 2), 2
//  v_B = (0, 1), 2;  (2, 1), 2
//  v_C = (0, 2), 1;  (2, 1), 1
//
// We assume the tripets are sorted by the block column index.
struct SimpleTriangularMatrixTriplet {
  SimpleTriangularMatrixTriplet(int row, int col, int size)
      : block_row(row), block_col(col), num_rows_entering(size) {}
  int block_row;
  int block_col;
  int num_rows_entering;
};

class SimpleTriangularMatrix {
 public:
  SimpleTriangularMatrix(const std::vector<int>& block_column_sizes,
                         const std::vector<SimpleTriangularMatrixTriplet>&
                             input_triplets_sorted_by_column);

  SimpleTriangularMatrix(const std::vector<int>& enter,
                         const std::vector<int>& exit);


  Eigen::MatrixXd MakeDenseMatrix() const;
  void SetConstant(double c) {
    for (auto& d : diagonal_blocks_) {
      d.setConstant(c);
    }
    for (auto& d : off_diagonal_blocks_) {
      d.setConstant(c);
    }
  }

  int cols() const { return num_cols_; }
  std::vector<int> block_sizes() const { return block_column_sizes_; }

  void AssembleFromCompressedColumns(const std::vector<Eigen::MatrixXd>& x) {
    if (x.size() != block_column_sizes_.size()) {
      throw std::runtime_error("Incorrect number of block columns provided.");
    }
    for (size_t i = 0; i < x.size() - 1; i++) {
      if (diagonal_blocks_.at(i).rows() + off_diagonal_blocks_.at(i).cols() !=
          x.at(i).rows()) {
        throw std::runtime_error("Incorrect number of block rows provided.");
      }
      if (diagonal_blocks_.at(i).cols() != x.at(i).cols()) {
        throw std::runtime_error("Size of block column is incorrect.");
      }
      diagonal_blocks_.at(i) = x.at(i).topRows(block_column_sizes_[i]);
      if (off_diagonal_blocks_.at(i).size() > 0) {
        off_diagonal_blocks_.at(i) =
            x.at(i).bottomRows(off_diagonal_blocks_.at(i).cols()).transpose();
      }
    }
    diagonal_blocks_.back() = x.back();
  }


  /* Increments a submatrix X of the full matrix T. The block X_{ij} is assigned
   * to T.block(partition.at(i).first, partition.at(j).first,
   * partition.at(i).first, partition.at(j).second).  
   * We assume that the partition is sorted by partition.at(:).first.
   * */  

  void IncrementSubmatrix(const Eigen::MatrixXd& X, 
                          const std::vector<std::pair<int, int>>& partition_sorted_by_block);

  void ComputeRootSchurComplement(Eigen::MatrixXd* X) {
    llt().compute(false);
    *X = diagonal_blocks().back();
  }

  void IncrementLeafSubmatrix(const Eigen::MatrixXd& submatrix) {
    std::vector<std::pair<int, int>> submatrix_partition;
    submatrix_partition.push_back(std::pair<int, int>(0,  diagonal_blocks_.at(0).rows()  ));
    //for (auto e : off_diagonal_partition_.at(0)) {
    //  submatrix_partition.push_back(e);
    //}
    IncrementSubmatrix(submatrix, submatrix_partition);
  }


  const std::vector<Eigen::MatrixXd>& diagonal_blocks() const { return diagonal_blocks_; }

  class LLT {
   public:
    bool compute(bool factor_last_block = true);
    Eigen::MatrixXd matrixL() { return matrix_.MakeDenseMatrix();  };

   private:
    LLT(SimpleTriangularMatrix* matrix) : matrix_(*matrix) {
      llt_of_diag_.reserve(matrix_.num_blocks_);
    }
    void SchurComplementInPlace(int i);
    bool ready() { return factorization_ready_; }
    SimpleTriangularMatrix& matrix_;
    std::vector<Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>>> llt_of_diag_;
    friend class SimpleTriangularMatrix;
    bool factorization_ready_ = false;
  };

  LLT llt() { return LLT(this); }

 private:

  std::vector<Eigen::MatrixXd> diagonal_blocks_;
  std::vector<Eigen::MatrixXd> off_diagonal_blocks_;
  std::vector<int> block_column_sizes_;
  std::vector<SimpleTriangularMatrixTriplet> off_diagonal_triplets_;
  int num_blocks_;
  int num_cols_ = 0;

  // Indicates that block column i contains off_diagonal_partition_.at(i).second
  // rows of block row off_diagonal_partition_.at(i).first.
  std::vector<std::vector<std::pair<int, int>>> off_diagonal_partition_;

  friend class LLT;
};


/* Builds a triangular matrix whose clique tree
 is a star.  We construct it from a collection of triangular
 matrices whose bottom right submatrix overlap.
 For instance, for example the star graph

       R
    /  |   \
  L1   L2  L3 

 we take as input matrices

 T1 = L1
      L1 R_1

 T2 = L2
      L2 R_2
 
 T3 = L3
      L3 R_3

 as construct:
  
  L1
     L2
        L3
  L1 L2 L3 (R_1 + R_2 + R_3)

 The matrices L_i and R_i are padded with zeros
 to have the same number of rows.
 */
class TriangularMatrixDirectSum {
 public:
  TriangularMatrixDirectSum(std::vector<SimpleTriangularMatrix>& matrices);
  Eigen::MatrixXd MakeDenseMatrix();

  class LLT {
   public:
    bool compute(bool factor_last_block = true);
    Eigen::MatrixXd matrixL();
   private:
    LLT(TriangularMatrixDirectSum* matrix) : matrix_(*matrix) { }
    bool ready() { return factorization_ready_; }
    const Eigen::MatrixXd& root_matrix() { return matrix_.common_block_; }
    TriangularMatrixDirectSum& matrix_;
    std::vector<Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>>> llt_of_diag_;
    bool factorization_ready_ = false;
    friend class TriangularMatrixDirectSum;
  };

  LLT llt() { return LLT(this); }
  Eigen::MatrixXd& root_matrix() { return common_block_; }
  const Eigen::MatrixXd& root_matrix() const { return common_block_; }
 private:
  std::vector<SimpleTriangularMatrix>& matrices_;
  Eigen::MatrixXd common_block_;
};

class BlockSparseSymmetricMatrix {
  BlockSparseSymmetricMatrix(
  const int num_blocks, 
  const std::vector<int>& start_block,
  const std::vector<int>& end_block);

  void SetFromDenseMatrix(const Eigen::MatrixXd&A ) {
    //auto y = GetCompressedBlockColumns(A, elimination_position_to_variable_, 
    //                                   cliques, block_sizes_);
    //lower_triangular_matrix_.AssembleFromCompressedColumns(y);
  }

  auto llt() { return lower_triangular_matrix_.llt(); }
  private:
   std::vector<int> block_sizes_;
   std::vector<int> elimination_position_to_variable_;
   SimpleTriangularMatrix lower_triangular_matrix_;
};




}  // namespace conex
