#pragma once
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

#define USE_SEPARATE_STORAGE 1
class SimpleTriangularMatrix {
 public:
  SimpleTriangularMatrix(const std::vector<int>& block_column_sizes,
                         const std::vector<SimpleTriangularMatrixTriplet>&
                             input_triplets_sorted_by_column);

  SimpleTriangularMatrix(const std::vector<int>& enter,
                         const std::vector<int>& exit);


  Eigen::MatrixXd MakeDenseMatrix() const;
#if USE_SEPARATE_STORAGE
  Eigen::Ref<const Eigen::MatrixXd> diagonal_blocks(int i) const { return diagonal_blocks_[i]; }
  Eigen::Ref<Eigen::MatrixXd> diagonal_blocks(int i) { return diagonal_blocks_[i]; }

  Eigen::Ref<const Eigen::MatrixXd> off_diagonal_blocks(int i) const { return off_diagonal_blocks_[i]; }
  Eigen::Ref<Eigen::MatrixXd> off_diagonal_blocks(int i) { return off_diagonal_blocks_[i]; }
#else

  Eigen::Ref<const Eigen::MatrixXd> diagonal_blocks(int i) const { return block_columns_[i].topRows(block_column_sizes_[i]); }
  Eigen::Ref<Eigen::MatrixXd> diagonal_blocks(int i) { return block_columns_[i].topRows(block_column_sizes_[i]); }

  auto off_diagonal_blocks(int i) const { return block_columns_[i].bottomRows(block_columns_[i].rows() -  block_column_sizes_[i]); }
  auto off_diagonal_blocks(int i) { return block_columns_[i].bottomRows(block_columns_[i].rows() -  block_column_sizes_[i]); }
#endif



  void SetConstant(double c) {
    for (int i = 0; i < num_blocks_; ++i) {
      diagonal_blocks(i).setConstant(c);
    }
    for (int i = 0; i < num_blocks_-1; ++i) {
      off_diagonal_blocks(i).setConstant(c);
    }
  }

  int cols() const { return num_cols_; }
  const std::vector<int>& block_sizes() const { return block_column_sizes_; }
  int num_blocks() const { return num_blocks_; }
#if USE_SEPARATE_STORAGE
  int num_off_diagonal_rows(int i) const { 
  return off_diagonal_blocks_.at(i).cols(); 
  } 
#else
  int num_off_diagonal_rows(int i) const { 
    return block_columns_.at(i).rows() - block_columns_.at(i).cols(); 
  } 
#endif

  void AssembleFromCompressedColumns(const std::vector<Eigen::MatrixXd>& x) {
    if (x.size() != block_column_sizes_.size()) {
      throw std::runtime_error("Incorrect number of block columns provided.");
    }
    for (size_t i = 0; i < x.size() - 1; i++) {
      if (block_column_sizes_.at(i) + num_off_diagonal_rows(i) 
          != x.at(i).rows()) {
        throw std::runtime_error("Incorrect number of block rows provided.");
      }
      if (diagonal_blocks(i).cols() != x.at(i).cols()) {
        throw std::runtime_error("Size of block column is incorrect.");
      }
      diagonal_blocks(i) = x.at(i).topRows(block_column_sizes_[i]);
      if (off_diagonal_blocks(i).size() > 0) {
        off_diagonal_blocks(i) =
            x.at(i).bottomRows(off_diagonal_blocks(i).cols()).transpose();
      }
    }
    diagonal_blocks(x.size() - 1) = x.back();
  }

  void AssembleFromDenseMatrix(const Eigen::MatrixXd& A);

  /* Increments a submatrix X of the full matrix T. The block X_{ij} is assigned
   * to T.block(partition.at(i).first, partition.at(j).first,
   * partition.at(i).first, partition.at(j).second).  
   * We assume that the partition is sorted by partition.at(:).first.
   * */  
  void IncrementSubmatrix(const Eigen::MatrixXd& X, 
                          const std::vector<std::pair<int, int>>& partition_sorted_by_block);

  void ComputeRootSchurComplement(Eigen::MatrixXd* X) {
    llt().compute(false);
    *X = diagonal_blocks(num_blocks_ - 1);
  }

  void IncrementLeafSubmatrix(const Eigen::MatrixXd& submatrix) {
    std::vector<std::pair<int, int>> submatrix_partition;
    submatrix_partition.push_back(std::pair<int, int>(0,  diagonal_blocks(0).rows()  ));
    //for (auto e : off_diagonal_partition_.at(0)) {
    //  submatrix_partition.push_back(e);
    //}
    IncrementSubmatrix(submatrix, submatrix_partition);
  }

  class LLT {
   public:
    bool compute(bool factor_last_block = true);
    Eigen::MatrixXd matrixL() { 
      if (factorization_ready_ != true) {
        throw std::runtime_error("Requested matrix not ready.");
      }
      Eigen::MatrixXd L = matrix_.MakeDenseMatrix();  
      if (vector_d_computed_) {
        //L.diagonal().array() = 1;
      }
      return L;
    };
    Eigen::VectorXd vectorD() { 
      if (vector_d_computed_ != true) {
        throw std::runtime_error("Requested matrix not ready.");
      }
      //return matrix_.MakeDenseMatrix().diagonal();  
    };
    void ApplyInverseOfL(Eigen::VectorXd* y);
    void ApplyInverseOfLt(Eigen::VectorXd* y);

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
    bool vector_d_computed_ = false;
  };

  LLT llt() { return LLT(this); }

 private:

#if USE_SEPARATE_STORAGE
  std::vector<Eigen::MatrixXd> diagonal_blocks_;
  std::vector<Eigen::MatrixXd> off_diagonal_blocks_;
#else
  std::vector<Eigen::MatrixXd> block_columns_;
#endif


  std::vector<int> block_column_sizes_;
  std::vector<SimpleTriangularMatrixTriplet> off_diagonal_triplets_;
  int num_blocks_;
  int num_cols_ = 0;

  // Indicates that block column i contains off_diagonal_partition_.at(i).second
  // rows of block row off_diagonal_partition_.at(i).first.
  std::vector<std::vector<std::pair<int, int>>> off_diagonal_partition_;

};



class BlockSparseSymmetricMatrix {
 public:
  BlockSparseSymmetricMatrix(
  const int num_blocks, 
  const std::vector<int>& start_block,
  const std::vector<int>& end_block);

  void SetFromDenseMatrix(const Eigen::MatrixXd& M) {
    int num_vars = elimination_position_to_variable_.size();
    Eigen::PermutationMatrix<-1> P(num_vars);
    P.indices() = Eigen::Map<const Eigen::VectorXi>(elimination_position_to_variable_.data(), num_vars);
    lower_triangular_matrix_.AssembleFromDenseMatrix(P.transpose()*M * P);
  }

  class LLT {
   public:
    bool compute();
    void ApplyInverseOfL(Eigen::VectorXd* y) { llt_.ApplyInverseOfL(y); }
    void ApplyInverseOfLt(Eigen::VectorXd* y) { llt_.ApplyInverseOfLt(y); }
    Eigen::PermutationMatrix<-1> matrixP() {
      Eigen::PermutationMatrix<-1> P(matrix_.elimination_position_to_variable_.size());
      P.indices() = Eigen::Map<const Eigen::VectorXi>(matrix_.elimination_position_to_variable_.data(), 
                                                      matrix_.elimination_position_to_variable_.size());
      return P;
    }
    Eigen::MatrixXd matrixL() { return llt_.matrixL().triangularView<Eigen::Lower>(); }
    Eigen::VectorXd vectorD() { return llt_.vectorD(); } 
   private:
    LLT(BlockSparseSymmetricMatrix* matrix) : matrix_(*matrix), 
    llt_(matrix_.lower_triangular_matrix_.llt()) { }
    friend class BlockSparseSymmetricMatrix;
    BlockSparseSymmetricMatrix& matrix_;
    SimpleTriangularMatrix::LLT llt_;
  };

  LLT llt() { return LLT(this); }
  private:
   std::vector<int> block_sizes_;
   std::vector<int> elimination_position_to_variable_;
   SimpleTriangularMatrix lower_triangular_matrix_;

  friend class LLT;
};

BlockSparseSymmetricMatrix MakeBlockSparseMatrix(const Eigen::MatrixXd& M, 
                                                        const std::vector<std::vector<int>>& cliques);



}  // namespace conex
