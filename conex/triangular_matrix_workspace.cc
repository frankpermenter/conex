#include "conex/triangular_matrix_workspace.h"

namespace conex {

using T = TriangularMatrixWorkspace;

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

TriangularMatrixWorkspace::TriangularMatrixWorkspace(
    const std::vector<Clique>& cliques,
    const std::vector<int>& block_column_size__)
    : block_column_size_(block_column_size__) {
  num_block_columns_ = cliques.size();
  num_columns_ =
      std::accumulate(block_column_size_.begin(), block_column_size_.end(), 0);
  variable_to_diagonal_block_.resize(num_columns_);
  variable_to_diagonal_block_position_.resize(num_columns_);
  variable_to_entering_block_column_.resize(num_columns_);

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
            "not "
            "satisfy the running intersection property.");
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
