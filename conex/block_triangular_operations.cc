#include "conex/block_triangular_operations.h"
#include "conex/RLDLT.h"
#include "conex/debug_macros.h"

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::pair;
using std::vector;
using T = BlockTriangularOperations;
namespace {

class PartitionVectorForwardIterator {
 public:
  PartitionVectorForwardIterator(VectorXd& b, const std::vector<int>& sizes)
      : b_(b), sizes_(sizes) {
    Reset();
  }

  Eigen::Ref<VectorXd> b_i() { return b_.segment(start_i, size_i); }
  Eigen::Ref<VectorXd> b_i_minus_1() {
    return b_.segment(start_i_minus_1, size_i_minus_1);
  }

  void Reset() {
    i_ = 0;
    size_i = sizes_[i_];
    start_i = 0;
  }
  void Increment() {
    start_i_minus_1 = start_i;
    size_i_minus_1 = size_i;
    i_++;
    size_i = sizes_[i_];
    start_i = start_i_minus_1 + size_i_minus_1;
  }

  int i_ = 0;
  int start_i_minus_1;
  int start_i;
  int size_i_minus_1;
  int size_i;
  VectorXd& b_;
  const std::vector<int>& sizes_;
  void Set(int i) {
    if (i > 0) {
      assert(0);
    }
    if (i < i_) {
      assert(0);
    }
    while (i > i_) {
      Increment();
    }
  }
};

class PartitionVectorIterator {
 public:
  PartitionVectorIterator(VectorXd& b, int N, const std::vector<int>& sizes)
      : b_(b), N_(N), sizes_(sizes) {
    Reset();
  }

  Eigen::Ref<VectorXd> b_i() { return b_.segment(start_i, size_i); }
  Eigen::Ref<VectorXd> b_i_plus_1() {
    return b_.segment(start_i_plus_1, size_i_plus_1);
  }
  void Reset() {
    i_ = sizes_.size() - 1;
    size_i = sizes_[i_];
    start_i = N_ - size_i;
  }
  void Decrement() {
    start_i_plus_1 = start_i;
    size_i_plus_1 = size_i;
    i_--;
    size_i = sizes_[i_];
    start_i = start_i_plus_1 - size_i;
  }

  int i_ = 0;
  int start_i_plus_1;
  int start_i;
  int size_i_plus_1;
  int size_i;
  VectorXd& b_;
  const int N_;
  const std::vector<int>& sizes_;
  void Set(int i) {
    if (i < 0) {
      assert(0);
    }
    if (i > i_) {
      assert(0);
    }
    while (i < i_) {
      Decrement();
    }
  }
};
}  // namespace

//  Applies the recursion
//
// for k = N - 1, N-2, ..., 0:
//   y_{k} = inv(U_{i}) b_{i}
//   b_{0:k-1} = b_{0:k-1} -  c_{0:k-1, k} * y_k

// where c_i are the columns of an upper triangular
// matrix:
//
//    c_0 c_1 c_2 c_3
//    U_0 R_0 R_0 R_0
//        U_1 R_1 R_1
//            U_2 R_2
//                U_3
// y_{N} --> when does y enter?
void T::ApplyBlockInverseOfTransposeInPlace(
    const TriangularMatrixWorkspace& mat, VectorXd* y) {
  PartitionVectorIterator y_partitioned(*y, mat.num_columns(),
                                        mat.block_column_size_);
  PartitionVectorIterator b_partitioned(*y, mat.num_columns(),
                                        mat.block_column_size_);
  for (int k = static_cast<int>(mat.diagonal.size() - 1); k > 0; k--) {
    if (mat.diagonal.at(k).size() == 0) {
      y_partitioned.Decrement();
      continue;
    }
    mat.diagonal.at(k).triangularView<Eigen::Lower>().transpose().solveInPlace(
        y_partitioned.b_i());

    b_partitioned.Reset();

    int jcnt = 0;
    for (auto j : mat.column_intersections[k - 1]) {
      b_partitioned.Set(j);
      // Find columns of B_j that are nonzero on columns c_{i+1} of supernode
      // i+1. This corresponds to separators(i) that contain supernode(j) for j
      // > i.
      const auto& index_and_column_list =
          mat.intersection_position[k - 1][jcnt++];
      for (const auto& pair : index_and_column_list) {
        b_partitioned.b_i().noalias() -= mat.off_diagonal[j].col(pair.second) *
                                         y_partitioned.b_i()(pair.first);
      }
    }

    y_partitioned.Decrement();
  }
  if (mat.diagonal[0].size() > 0) {
    mat.diagonal[0].triangularView<Eigen::Lower>().transpose().solveInPlace(
        y_partitioned.b_i());
  }
}

//  c_1 c_2 c_3
//  L_1
//  B_1 L_2
//  B_1 B_2 L_3
//
//   y_{i} = inv(L_{i}) r_{i}
//   r = r -  c_i * y_i
void T::ApplyBlockInverseInPlace(const TriangularMatrixWorkspace& mat,
                                 VectorXd* y) {
  PartitionVectorForwardIterator ypart(*y, mat.block_column_size_);

  for (size_t i = 0; i < mat.diagonal.size() - 1; i++) {
    if (mat.diagonal[i].size() == 0) {
      ypart.Increment();
      continue;
    }
    mat.diagonal[i].triangularView<Eigen::Lower>().solveInPlace(ypart.b_i());
    if (mat.off_diagonal[i].size() > 0) {
      mat.temporaries[i].noalias() =
          mat.off_diagonal[i].transpose() * ypart.b_i();
      int cnt = 0;
      for (auto si : mat.non_zero_rows_[i]) {
        (*y)(si) -= mat.temporaries[i](cnt);
        cnt++;
      }
    }
    ypart.Increment();
  }
  mat.diagonal.back().triangularView<Eigen::Lower>().solveInPlace(ypart.b_i());
}

// For each element of rows, decide if consecutive elements are in same diagonal
// block.  If supernodes are sorted by their entering column, then the
// non_zero_rows in column C_i that exit in column C_j will be contiguous.
//
// Pf:
//
// Suppose (i, j) exit in the same block column J
// and appear in block column I and assume j > i.  Since they exit
// in the same column, all k \in [i, j]  also exit
// in this column. Further, enter(k) < enter(j).  Hence,
// all rows k between (i, j) appear in I.
//
// Note that non-zero rows that exist in different blocks
// J_1 and J_2 need not be contiguous as illustrated in
// the following example
//
//
//  *                      C1;R1         C2;R2      C3;R3
//  * *                 (1, 2; 3, 5),  (3, 4; 5),  (5, 6;)
//  * * *
//      * *
//  * * * * *
//          * *
//
//  Here rows 3, 5 \in R1 are not contig., but they exit
//  in different rows.
struct BatchUpdateBlocks {
  int exiting_column_block = 0;
  int offset = 0;
  int size = 0;
};
//
vector<BatchUpdateBlocks> GetBlocks(
    const vector<int>& rows, const vector<int>& variable_to_diagonal_block,
    const vector<int>& variable_to_diagonal_block_position) {
  vector<BatchUpdateBlocks> pairs;
  pairs.push_back({});

  pairs.back().exiting_column_block = variable_to_diagonal_block[rows[0]];
  pairs.back().offset = variable_to_diagonal_block_position[rows[0]];
  pairs.back().size = 1;

  for (size_t i = 1; i < rows.size(); i++) {
    int block = variable_to_diagonal_block[rows[i]];
    if (pairs.back().exiting_column_block == block) {
      pairs.back().size++;
    } else {
      pairs.push_back({});
      pairs.back().exiting_column_block = block;
      pairs.back().offset = variable_to_diagonal_block_position[rows[i]];
      pairs.back().size = 1;
    }
  }
  return pairs;
}

//  we want to update the sub-matrix R_j_exit_col(I) with  R_i_I
//  R_i_I.transpose() This matrix has structure:
//
//    exit_col(I) = [ I, ..., ],
//
// i.e., we know that I will be the first set of columns.
//
//  so to perform the block updates, we need the offsets of J and K inside
//  of non_zero_rows(exit_col(I)).  Letting E = exit_col(I),
//
//  R_E^T = [  ..., (R_J_E)^T, ... , (R_K_E)^T, ...  ].
//
//  So we need to know where exit_col(J) and exit_col(K)  start
//  inside of non_zero_rows_(E)).

// Helper function for computing the off-diagonal part of
//
//     C - B(LL^T)^{-1} B^T.
//
//  where C is bottom-right submatrix S_{i+1}. The inputs
//  are a lower triangular matrix X whose block column i
//  contains the factorization L and matrices L^{-1} B,
//  and columns j > i contain C.  Letting R^T denote L^{-1} B,
//  and I, J, K, L the  block columns of X, we will update
//  data in column J using
//  I       J     K      L
//  L       R_j   R_k    R_l
//  R^T_j   C_jj  C_jk   C_jl
//  R^T_k
//  R^T_l
//
//
//  subsets J and K that are non-zero in I.
//  Hence, to perform the update
//
//    C_{jk} -= R_j R^T_k where j, k denote
//
//  we need to extract the columns R_j and R_k from R and the submatrix C_jk
//  from C_{JK}. The column position of R_k in R is given by offsets(k, i) and
//  the column position of C_{jk} in C_J is given by offsets(k, j).
//
void GetRectangularBlocks(
    TriangularMatrixWorkspace* X, const int i,
    const vector<BatchUpdateBlocks> blocks,
    const Eigen::MatrixXd&
        offsets /* (i, j) entry: where row i starts in column j*/) {
  auto& R = X->off_diagonal.at(i);

  // Loop over the non-zero block rows of R^T.
  // The size records the number of rows and the
  // offset the offset in block column I.
  for (size_t j = 0; j < blocks.size(); j++) {
    int j_size = blocks.at(j).size;
    int j_offset = offsets(blocks.at(j).exiting_column_block, i);
    for (size_t k = j + 1; k < blocks.size(); k++) {
      if (blocks.at(k).exiting_column_block ==
          blocks.at(j).exiting_column_block) {
        throw std::runtime_error("Sparse matrix is malformed.");
      }
      int k_size = blocks.at(k).size;
      int k_offset = offsets(blocks.at(k).exiting_column_block, i);
      int destination_offset = offsets(blocks.at(k).exiting_column_block,
                                       blocks.at(j).exiting_column_block);
      if (destination_offset < 0) {
        throw std::runtime_error("Sparse matrix is malformed.");
      }

      X->off_diagonal.at(blocks.at(j).exiting_column_block)
          .topRows(j_size)
          .middleCols(destination_offset, k_size) -=
          R.middleCols(j_offset, j_size).transpose() *
          R.middleCols(k_offset, k_size);
    }
  }
}

// Recursively compute LL^T transform of input matrix X.
// We recursively update a principal submatrix S_i.
// Initialiing S_i = X, we partition S_i as
//
//  S_i = [A,  B^T
//         B,   C]
//
//  where A.cols() = X.diagonal.at(i).cols().
//
// Letting L = llt(A).matrixL, we update S_i with
//
//  S_i = [L,
//         (L^{-1} B)^T   C - B(LL^T)^{-1} B^T
//
//  We then set S_{i+1} = C - B(LL^T)^{-1} B^T and repeat.
bool T::BlockCholeskyInPlace(TriangularMatrixWorkspace* X,
                             bool use_batch_update) {
  if (use_batch_update && !X->sorted_by_entering_columns) {
    throw std::runtime_error(
        "Cannot do batch updates: supernodes are"
        "not sorted by entering block column.");
  }
  assert(X->diagonal.size() == X->off_diagonal.size());
  auto& llts = X->llts;
  if (llts.size() > 0) {
    llts.clear();
  }
  for (size_t i = 0; i < X->diagonal.size(); i++) {
    // In place LLT of [n, n] block
    if (X->diagonal[i].size() > 0) {
      llts.emplace_back(X->diagonal[i]);
      if (llts.back().info() != Eigen::Success) {
        return false;
      }
    } else {
      // Dummy decomposition.
      MatrixXd x(1, 1);
      x(0) = 1;
      llts.emplace_back(x);
    }

    if (X->off_diagonal[i].size() > 0) {
      llts.back().matrixL().solveInPlace(X->off_diagonal[i]);
      auto& temp = X->off_diagonal[i];

      if (use_batch_update) {
        auto blocks =
            GetBlocks(X->non_zero_rows_.at(i), X->variable_to_diagonal_block_,
                      X->variable_to_diagonal_block_position_);
        int offset = 0;
        for (auto b : blocks) {
          MatrixXd G = temp.middleCols(offset, b.size).transpose() *
                       temp.middleCols(offset, b.size);
          X->diagonal[b.exiting_column_block].block(b.offset, b.offset, b.size,
                                                    b.size) -= G;
          offset += b.size;
        }
        GetRectangularBlocks(X, i, blocks, X->nonzero_row_offsets_);
      }

      if (!use_batch_update) {
        int index = 0;
        const auto& s_s = X->scatter_destination_pointers[i];
        for (int k = 0; k < temp.cols(); k++) {
          for (int j = k; j < temp.cols(); j++) {
            *s_s[index++] -= temp.col(k).dot(temp.col(j));
          }
        }
      }
    }
  }
  return true;
}

// Apply inv(M^T)  = inv(L^T P) = P^T inv(L^T)
void T::ApplyBlockInverseOfMTranspose(
    const TriangularMatrixWorkspace& mat,
    const std::vector<Eigen::RLDLT<Eigen::Ref<MatrixXd>>> factorization,
    VectorXd* y) {
  PartitionVectorIterator ypart(*y, mat.num_columns(), mat.block_column_size_);
  factorization.back().matrixL().transpose().solveInPlace(ypart.b_i());
  Eigen::PermutationMatrix<-1> P0(factorization.back().transpositionsP());
  ypart.b_i() = P0.transpose() * ypart.b_i();

  for (int i = static_cast<int>(mat.diagonal.size() - 2); i >= 0; i--) {
    ypart.Decrement();

    // Loop over partition {B_j} of c_{i+1}
    PartitionVectorIterator residual(*y, mat.num_columns(),
                                     mat.block_column_size_);

    int jcnt = 0;
    for (auto j : mat.column_intersections[i]) {
      residual.Set(j);
      // Find columns of B_j that are nonzero on columns c_{i+1} of supernode
      // i+1. This corresponds to separators(i) that contain supernode(j) for j
      // > i.
      // auto index_and_column_list =
      //    IntersectionOfSupernodeAndSeparator(mat, i + 1, j);
      // for (const auto& pair : index_and_column_list) {
      //  residual.b_i() -= mat.off_diagonal[j].col(pair.second) *
      //                    ypart.b_i_plus_1()(pair.first);
      //}

      auto index_and_column_list = mat.intersection_position[i][jcnt++];
      for (const auto& pair : index_and_column_list) {
        residual.b_i() -= mat.off_diagonal[j].col(pair.second) *
                          ypart.b_i_plus_1()(pair.first);
      }
    }

    // mat.diagonal[i].triangularView<Eigen::Lower>().transpose().solveInPlace(ypart.b_i());
    factorization[i].matrixL().transpose().solveInPlace(ypart.b_i());
    Eigen::PermutationMatrix<-1> Pi(factorization[i].transpositionsP());
    ypart.b_i() = Pi.transpose() * ypart.b_i();
  }
}

void T::ApplyBlockInverseOfMD(
    const TriangularMatrixWorkspace& mat,
    const std::vector<Eigen::RLDLT<Eigen::Ref<MatrixXd>>> factorization,
    VectorXd* y) {
  // Apply inv(M) = inv(P^T L) = inv(L) P
  PartitionVectorForwardIterator ypart(*y, mat.block_column_size_);
  Eigen::PermutationMatrix<-1> P0(factorization[0].transpositionsP());
  ypart.b_i() = P0 * ypart.b_i();
  factorization[0].matrixL().solveInPlace(ypart.b_i());

  for (size_t i = 1; i < mat.diagonal.size(); i++) {
    ypart.Increment();
    if (mat.off_diagonal[i - 1].size() > 0) {
      VectorXd temp = mat.off_diagonal[i - 1].transpose() * ypart.b_i_minus_1();
      int cnt = 0;
      for (auto si : mat.non_zero_rows_[i - 1]) {
        (*y)(si) -= temp(cnt);
        cnt++;
      }
    }
    Eigen::PermutationMatrix<-1> Pi(factorization[i].transpositionsP());
    ypart.b_i() = Pi * ypart.b_i();
    factorization[i].matrixL().solveInPlace(ypart.b_i());
  }

  // Apply D inverse
  PartitionVectorForwardIterator ypart2(*y, mat.block_column_size_);
  ypart2.b_i() =
      factorization[0].vectorD().cwiseInverse().cwiseProduct(ypart2.b_i());
  for (size_t i = 1; i < mat.diagonal.size(); i++) {
    ypart2.Increment();
    ypart2.b_i() =
        factorization[i].vectorD().cwiseInverse().cwiseProduct(ypart2.b_i());
  }
}

// M D M^T
// M = P^T L
//
//   M    0   D_1      M^T   Q^T
//   Q    T       D_2        T^T
//
//  M D_1             M^T   Q^T
//  Q D_1     T D_2         T^T
//
//  [M D_1 M^T   M D_1 Q^T
//   Q D_1 M^T   Q D_1 Q^T + T D_2 T^T]
//
//  So, Q^T = inv(D_1) inv(M) off_diag
//          = inv(D_1) inv(L) P  * off_diag
bool T::BlockLDLTInPlace(
    TriangularMatrixWorkspace* C,
    std::vector<Eigen::RLDLT<Eigen::Ref<MatrixXd>>>* factorization) {
  auto& llts = *factorization;
  llts.clear();

  bool regularization_used = false;
  for (size_t i = 0; i < C->diagonal.size(); i++) {
    // In place LLT of [n, n] block
    llts.emplace_back(C->diagonal[i]);
    if (llts.back().regularization_used()) {
      regularization_used = true;
    }
    Eigen::PermutationMatrix<-1> P(llts[i].transpositionsP());

    if (C->off_diagonal[i].size() > 0) {
      C->off_diagonal[i] = P * C->off_diagonal[i];
      llts.back().matrixL().solveInPlace(C->off_diagonal[i]);
      C->off_diagonal[i].noalias() =
          llts.back().vectorD().asDiagonal().inverse() * (C->off_diagonal[i]);

      MatrixXd temp = llts.back().vectorD().asDiagonal() * C->off_diagonal[i];

      int index = 0;
      const auto& s_s = C->scatter_destination_pointers[i];
      for (int k = 0; k < temp.cols(); k++) {
        for (int j = k; j < temp.cols(); j++) {
          *s_s[index++] -= temp.col(k).dot(C->off_diagonal[i].col(j));
        }
      }
    }
  }
  return !regularization_used;
}

}  // namespace conex
