#include "conex/triangular_matrix_workspace.h"


#include <numeric>

#include "conex/tree_utils.h"
namespace conex {
using Eigen::MatrixXd;

namespace {

std::vector<int> GetSupernodeSize(const std::vector<Clique>& cliques,
                                  const std::vector<int>& tree_in = {}) {
  std::vector<int> y;
  std::vector<int> tree = tree_in;

  if (tree.size() == 0) {
    tree.resize(cliques.size());
    std::iota(tree.begin(), tree.end() - 1, 1);
    tree.back() = -1;
  }

  for (size_t j = 0; j < cliques.size(); j++) {
    std::vector<int> temp;
    if (tree.at(j) >= 0) {
      IntersectionOfSorted(cliques.at(j), cliques.at(tree.at(j)), &temp);
      y.push_back(cliques.at(j).size() - temp.size());
    } else {
      y.push_back(cliques.at(j).size());
    }
  }
  return y;
}


using BlockData = std::vector<std::pair<int, int>>;

class BlockMatrix {
 public:
  BlockMatrix(Eigen::MatrixXd& X, 
              const BlockData& blocks) : X_(X), blocks_(blocks) {}

  BlockMatrix(Eigen::MatrixXd& X, 
              const BlockData& blocks, int initial_index) : X_(X), blocks_(blocks) {
    for (int i = 0; i < initial_index; i++) {
      GotoNextBlock();
    }
  }

  //void CurrentBlock() { return X.middleCols(offset, size); }
  //void SetBlock(int i) { return X; }
  void GotoNextBlock() { 
    current_block_offset_ += blocks_.at(current_block_index_).second;
    current_block_index_++;
  }
  void GotoBlock(int i) {
    while (blocks_.at(current_block_offset_).first != i ) {
      GotoNextBlock();
    }
  }
  int CurrentBlockNumber() {
    return blocks_.at(current_block_index_).first;
  }

  int CurrentBlockSize() {
    return blocks_.at(current_block_index_).second;
  }

  Eigen::Ref<MatrixXd> CurrentBlock() {
    return X_.middleCols(current_block_offset_, CurrentBlockSize());
  }
  MatrixXd& X_;
  const BlockData& blocks_;
  int current_block_offset_ = 0;
  int current_block_index_ = 0;
};


}  // namespace

using T = TriangularMatrixWorkspace;
using S = SimpleTriangularMatrix;
  S::SimpleTriangularMatrix(const std::vector<int>& block_column_sizes,   
                            const std::vector<SimpleTriangularMatrixTriplet>& input_triplets) :
  block_column_sizes_(block_column_sizes), off_diagonal_triplets_(input_triplets)
{
    num_cols_ = std::accumulate(block_column_sizes.begin(), block_column_sizes.end(), 0);
    num_blocks_ = block_column_sizes.size();
    diagonal_blocks_.resize(num_blocks_);
    off_diagonal_blocks_.resize(num_blocks_ - 1);
    for (int i = 0; i < num_blocks_; i++) {
      diagonal_blocks_[i].resize(block_column_sizes[i], block_column_sizes[i]);
    }

    std::vector<int> off_diagonal_size(num_blocks_ - 1, 0);
    for (auto s : input_triplets) {
      for (int i = s.block_col; i < s.block_row; i++) {
        off_diagonal_size.at(i) += s.num_rows_entering;
      }
    }
    for (size_t i = 0; i < block_column_sizes.size() - 1; i++) {
      off_diagonal_blocks_[i].resize(block_column_sizes[i], off_diagonal_size[i]);
    }
  }


MatrixXd S::MakeDenseMatrix() const {
  MatrixXd M(num_cols_, num_cols_); M.setZero();
  int offset = 0;
  for (size_t i = 0; i < block_column_sizes_.size(); i++) {
    M.block(offset, offset, block_column_sizes_[i], block_column_sizes_[i]) = diagonal_blocks_.at(i);
    offset += block_column_sizes_[i];
  }

  std::vector<int> global_offsets(num_blocks_, 0);
  std::partial_sum(block_column_sizes_.begin(), block_column_sizes_.end() - 1,   global_offsets.begin() + 1);

  vector<int> internal_offsets(num_blocks_, 0);
  for (auto s : off_diagonal_triplets_) {
    for (int i = s.block_col; i < s.block_row; i++) {
      int size = s.num_rows_entering;
      int offset = internal_offsets.at(s.block_row);
      int r = global_offsets.at(s.block_row) + offset;
      M.block(r, global_offsets.at(i), size, block_column_sizes_.at(i)) = 
          off_diagonal_blocks_.at(i).middleCols(offset, size).transpose();
    }
    internal_offsets.at(s.block_row) += s.num_rows_entering;
  }
  return M;
}

//    BlockMatrix R(Rdata, input_block_info);
//    auto& off_diagonal_blocks = off_diagonal_blocks_;
//    auto& diagonal_blocks = diagonal_blocks_;
//    BlockMatrix input_i(Rdata, input_block_info);
//    for (size_t i = 0; i < input_block_info.size(); i++) {
//      int size_i = input_i.CurrentBlockSize();
//      BlockMatrix output(off_diagonal_blocks.at(input_i.CurrentBlockNumber()), offsets_.at(input_i.CurrentBlockNumber()));
//      BlockMatrix input_j(Rdata, input_block_info, i+1); 
//      for (size_t j = i+1; j < input_block_info.size(); j++) {
//        output.GotoBlock(input_j.CurrentBlockNumber());
//        output.CurrentBlock().topRows(size_i) 
//            -=  input_i.CurrentBlock().transpose() * input_j.CurrentBlock(); 
//        input_j.GotoNextBlock();
//      }
//
//      diagonal_blocks.at(input_i.CurrentBlockNumber()).topLeftCorner(size_i, size_i)
//            -=  input_i.CurrentBlock().transpose()  * input_i.CurrentBlock();
//    }
//    input_i.GotoNextBlock();


//  vector<int> internal_offsets(matrix_.num_blocks_, 0);
//  int i = 0;
//  for (i = triplet_offset; i < matrix_.off_diagonal_triplets_.size(); i++) {
//    const auto& si = matrix_.off_diagonal_triplets_.at(i);
//    if (si.block_col != block) {
//      break;
//    }
//    for (int j = triplet_offset + 1; j < matrix_.off_diagonal_triplets_.size(); j++) {
//      const auto& sj = matrix_.off_diagonal_triplets_.at(j);
//      if (sj.block_col != block) {
//        break;
//      }
//      int num_rows = internal_offsets.at(i);
//    }
//  }


// Replace bottom right corner C_22 with  C22 - C12' inv(C11) C12.
// We assume that (C11)^{-1/2} C12 has already been computed
// and stored in the block C12.  The full matrix C starts 
// at the diagonal block (i, i). 
void S::LLT::SchurComplementInPlace(int block) {
  for (size_t i = 0; i < active_blocks.size(): i++) {
    for (size_t j = i + 1; j < active_blocks.size(): j++) {
      off_diagonal.at(block).
    }
  }
}

T::TriangularMatrixWorkspace(const CliqueTree& tree)
    : TriangularMatrixWorkspace(
          tree.cliques, GetSupernodeSize(tree.cliques, tree.parent_in_tree.parent)) { }

//T::TriangularMatrixWorkspace(const JunctionTree& tree)
//    : TriangularMatrixWorkspace(
//          tree.cliques, GetSupernodeSize(tree.cliques, tree.parent_in_tree)) {}
//


double* TriangularMatrixWorkspace::LookupAddress(int r, int c) {
  int node = variable_to_diagonal_block_[c];
  int node_r = variable_to_diagonal_block_[r];

  int j = variable_to_diagonal_block_position_[c];
  if (node == node_r) {
    int i = variable_to_diagonal_block_position_[r];
    return &diagonal[node](i, j);
  }

  int cnt = 0;
  for (auto si : non_zero_rows_[node]) {
    if (si == r) {
      return &off_diagonal[node](j, cnt);
    }
    cnt++;
  }
  throw std::runtime_error(
      "Specified entry of sparse matrix is not accessible.");
}

RootedTree MakePath(int length) {
  RootedTree tree(length);
  for (int i = 0; i < length - 1; i++) {
    tree.parent[i] = i + 1;
  }
  tree.parent.back() = -1;
  return tree;
}

TriangularMatrixWorkspace::TriangularMatrixWorkspace(
    const std::vector<Clique>& cliques,
    const std::vector<int>& block_column_size,
    const RootedTree& parent_in_tree)
    : block_column_size_(block_column_size) {
  num_block_columns_ = cliques.size();
  num_columns_ =
      std::accumulate(block_column_size_.begin(), block_column_size_.end(), 0);
  variable_to_diagonal_block_.resize(num_columns_);
  variable_to_diagonal_block_position_.resize(num_columns_);
  variable_to_entering_block_column_.resize(num_columns_);


  clique_tree_ = parent_in_tree;
  if (clique_tree_.parent.size() == 0) {
    clique_tree_ = MakePath(cliques.size());
  }

  int cnt = 0;
  int var = 0;
  for (cnt = 0; cnt < num_block_columns_; cnt++) {
    for (int i = 0; i < block_column_size_.at(cnt); i++) {
      if (var >= num_columns_) {
        std::runtime_error("Invalid variable index.");
      }
      variable_to_diagonal_block_[var] = cnt;
      variable_to_entering_block_column_[var] = cnt;
      variable_to_diagonal_block_position_[var] = i;
      var++;
    }
  }

  non_zero_rows_.resize(cliques.size());
  column_intersections.resize(num_block_columns_ - 1);
  intersection_position.resize(num_block_columns_ - 1);

  // For each block column J, build list of non-zero
  // rows i not in J:
  //
  //  non_zero_rows_(J) = list of non_zero_rows_
  //
  // For each block column I, build list of block columns
  // J nonzero on row i \in I. Store this using two list of lists:.
  //
  //  column_intersection(I)(J) = list of (i, j) pairs, where
  //  i \in I,
  //  j \in non_zero_rows_(J)(j)
  int J = 0;
  for (auto& sep_i : non_zero_rows_) {
    int seperator_size = cliques.at(J).size() - block_column_size_.at(J);
    sep_i.resize(seperator_size);
    for (int i = 0; i < seperator_size; i++) {
      int var = cliques.at(J).at(i + block_column_size_.at(J));
      sep_i[i] = var;

      if (variable_to_entering_block_column_[var] > J) {
        variable_to_entering_block_column_[var] = J;
      }

      int I = variable_to_diagonal_block_[var] - 1;
      if (J > I) {
        throw std::runtime_error(
            "This variable has already been eliminated. The input cliques do "
            "not satisfy the running intersection property.");
      }

      // Create list for this separator if supernode doesn't have one.
      if (column_intersections[I].size() == 0 ||
          column_intersections[I].back() != J) {
        column_intersections[I].push_back(J);
        intersection_position[I].emplace_back(
            std::vector<std::pair<int, int>>());
      }
      std::pair<int, int> pair{variable_to_diagonal_block_position_[var], i};
      intersection_position[I].back().push_back(pair);
    }
    J++;
  }

  // Build M(i, j): the smallest element k in
  // non_zero_row_(j) satisfying exiting_column(k) = i.
  nonzero_row_offsets_.resize(num_block_columns_, num_block_columns_);
  nonzero_row_offsets_.setConstant(-1);

  for (size_t j = 0; j < non_zero_rows_.size(); j++) {
    for (size_t k = 0; k < non_zero_rows_.at(j).size(); k++) {
      int i = variable_to_diagonal_block_[non_zero_rows_.at(j).at(k)];
      if (nonzero_row_offsets_(i, j) == -1) {
        nonzero_row_offsets_(i, j) = k;
      }
    }
  }

  var = 0;
  sorted_by_entering_columns = true;
  for (cnt = 0; cnt < num_block_columns_; cnt++) {
    for (int i = 0; i < block_column_size_.at(cnt) - 1; i++) {
      // Within block column order nodes by when they enter
      if (variable_to_entering_block_column_[var] >
          variable_to_entering_block_column_[var + 1]) {
        sorted_by_entering_columns = false;
      }
      var++;
    }
    var++;
  }

  // TODO(FrankPermenter): Remove this.
  for (auto& l : column_intersections) {
    std::reverse(l.begin(), l.end());
  }
  for (auto& l : intersection_position) {
    std::reverse(l.begin(), l.end());
  }
}

double TriangularMatrixWorkspace::coeff(int r, int c) const {
  int node_r = variable_to_diagonal_block_[r];
  int node = variable_to_diagonal_block_[c];

  int j = variable_to_diagonal_block_position_[c];
  if (node == node_r) {
    int i = variable_to_diagonal_block_position_[r];
    return diagonal[node](i, j);
  }

  int cnt = 0;
  for (auto si : non_zero_rows_[node]) {
    if (si == r) {
      return off_diagonal[node](j, cnt);
    }
    cnt++;
  }
  return 0;
}

void Initialize(TriangularMatrixWorkspace* o, double* data_start) {
  double* data = data_start;
  for (int j = 0; j < o->num_block_columns_; j++) {
    o->diagonal.emplace_back(data, o->block_column_size_.at(j),
                             o->block_column_size_.at(j));

    data += o->SizeOfSupernode(j);
    o->off_diagonal.emplace_back(data, o->block_column_size_.at(j),
                                 o->non_zero_rows_.at(j).size());
    data += o->SizeOfSeparator(j);
  }

  o->scatter_destination_pointers.resize(o->num_block_columns_);
  for (int j = 0; j < o->num_block_columns_; j++) {
    o->S_S(j, &o->scatter_destination_pointers.at(j));
  }

  // Use reserve so that we can call default constructor of LLT objects.
  o->llts.reserve(o->num_block_columns_);

  o->temporaries.resize(o->num_block_columns_);
  for (int j = 0; j < o->num_block_columns_; j++) {
    o->temporaries.at(j).resize(o->non_zero_rows_.at(j).size());
  }
}

void TriangularMatrixWorkspace::S_S(int clique, std::vector<double*>* y) {
  auto& s = non_zero_rows_.at(clique);
  int size = .5 * (s.size() * s.size() + s.size());
  y->resize(size);
  int cnt = 0;
  for (size_t j = 0; j < s.size(); j++) {
    for (size_t i = j; i < s.size(); i++) {
      (*y)[cnt++] = LookupAddress(s[i], s[j]);
    }
  }
}

}  // namespace conex
