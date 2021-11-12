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
  for (auto si : separators[node]) {
    if (si == r) {
      return &off_diagonal[node](j, cnt);
    }
    cnt++;
  }
  throw std::runtime_error(
      "Specified entry of sparse matrix is not accessible.");
}

TriangularMatrixWorkspace::TriangularMatrixWorkspace(
    const std::vector<Clique>& cliques, const std::vector<int>& supernode_size_)
    : supernode_size(supernode_size_) {
  num_block_columns_ = cliques.size();
  num_columns_ =
      std::accumulate(supernode_size.begin(), supernode_size.end(), 0);
  variable_to_diagonal_block_.resize(num_columns_);
  variable_to_diagonal_block_position_.resize(num_columns_);

  int cnt = 0;
  int var = 0;
  for (cnt = 0; cnt < num_block_columns_; cnt++) {
    for (int i = 0; i < supernode_size.at(cnt); i++) {
      if (var >= num_columns_) {
        std::runtime_error("Invalid variable index.");
      }
      variable_to_diagonal_block_[var] = cnt;
      variable_to_diagonal_block_position_[var] = i;
      var++;
    }
  }

  separators.resize(cliques.size());
  column_intersections.resize(num_block_columns_ - 1);
  intersection_position.resize(num_block_columns_ - 1);
  cnt = 0;

  // For each supernode [sn], find cliques that overlap. Store
  // this using two list of lists:.
  //  separator_list(supernode) = list of separators
  //  column_intersection(supernode) = list of (i, j) pairs, where
  //  supernode[i] = separator_list(supernode)[j]
  for (auto& sep_i : separators) {
    int seperator_size = cliques.at(cnt).size() - supernode_size.at(cnt);
    sep_i.resize(seperator_size);
    for (int i = 0; i < seperator_size; i++) {
      int var = cliques.at(cnt).at(i + supernode_size.at(cnt));
      sep_i[i] = var;

      int sn = variable_to_diagonal_block_[var] - 1;
      if (cnt > sn) {
        throw std::runtime_error(
            "Supernode has already been eliminated. The input cliques do not "
            "satisfy the running intersection property.");
      }

      // Create list for this separator if supernode doesn't have one.
      if (column_intersections[sn].size() == 0 ||
          column_intersections[sn].back() != cnt) {
        column_intersections[sn].push_back(cnt);
        intersection_position[sn].emplace_back(
            std::vector<std::pair<int, int>>());
      }
      std::pair<int, int> pair{variable_to_diagonal_block_position_[var], i};
      intersection_position[sn].back().push_back(pair);
    }
    cnt++;
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
  for (auto si : separators[node]) {
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
    o->diagonal.emplace_back(data, o->supernode_size.at(j),
                             o->supernode_size.at(j));

    data += o->SizeOfSupernode(j);
    o->off_diagonal.emplace_back(data, o->supernode_size.at(j),
                                 o->separators.at(j).size());
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
    o->temporaries.at(j).resize(o->separators.at(j).size());
  }
}

void TriangularMatrixWorkspace::S_S(int clique, std::vector<double*>* y) {
  auto& s = separators.at(clique);
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
