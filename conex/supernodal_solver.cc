#include "conex/supernodal_solver.h"
#include "conex/clique_ordering.h"

#include <iostream>
#include <map>

#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Permutation = Eigen::PermutationMatrix<-1>;
using std::vector;

namespace {

using Matrix = SparseTriangularMatrix;

vector<int> Relabel(const vector<int>& x, const vector<int>& labels) {
  vector<int> y(x.size());
  int i = 0;
  for (auto& xi : x) {
    y.at(i++) = labels.at(xi);
  }
  return y;
}

int GetMax(const std::vector<Clique>& cliques) {
  int max = cliques.at(0).at(0);
  for (const auto& c : cliques) {
    for (const auto ci : c) {
      if (ci > max) {
        max = ci;
      }
    }
  }
  return max;
}

}  // namespace

double SparseTriangularMatrix::coeff(int i, int j) const {
  return workspace_.coeff(i, j);
}

void IntersectionOfSorted(const std::vector<int>& v1,
                          const std::vector<int>& v2, std::vector<int>* v3) {
  v3->clear();
  std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                        back_inserter(*v3));
}

std::vector<Clique> Permute(std::vector<Clique>& path,
                            std::vector<int>& permutation) {
  auto y = path;
  for (size_t i = 0; i < path.size(); i++) {
    for (size_t j = 0; j < path.at(i).size(); j++) {
      y.at(i).at(j) = permutation.at(path.at(i).at(j));
    }
  }
  return y;
}

void Sort(std::vector<Clique>* path) {
  for (size_t i = 0; i < path->size(); i++) {
    std::sort(path->at(i).begin(), path->at(i).end());
  }
}

Eigen::MatrixXd Matrix::MakeDenseMatrix() const {
  auto mat = *this;
  MatrixXd y(mat.num_columns(), mat.num_columns());
  for (int i = 0; i < mat.num_columns(); i++) {
    for (int j = 0; j < mat.num_columns(); j++) {
      y(i, j) = mat.workspace_.coeff(i, j);
    }
  }
  return y;
}

void SparseTriangularMatrix::SetConstant(double val) {
  for (auto& n : supernodes_) {
    n.array() = val;
  }
  for (auto& n : separator_) {
    n.array() = val;
  }
}

std::vector<int> UnionOfSorted(const std::vector<int>& x1,
                               const std::vector<int>& x2) {
  std::vector<int> y;
  set_union(x1.begin(), x1.end(), x2.begin(), x2.end(), inserter(y, y.end()));
  return y;
}

MatrixData GetData(const vector<vector<int>>& cliques, int root_clique) {
  vector<vector<int>> separators;
  vector<vector<int>> supernodes;
  vector<std::vector<int>> cliques_sorted = cliques;
  Sort(&cliques_sorted);
  vector<int> order;

  PickCliqueOrder(cliques_sorted, root_clique, &order, &supernodes,
                  &separators);

  return SupernodesToData(GetMax(cliques) + 1, order, supernodes, separators);
}

MatrixData GetData(const vector<vector<int>>& cliques,
                   const std::vector<int>& valid_leaf, int root_clique) {
  vector<vector<int>> separators;
  vector<vector<int>> supernodes;
  vector<std::vector<int>> cliques_sorted = cliques;
  Sort(&cliques_sorted);
  vector<int> order;

  PickCliqueOrder(cliques_sorted, valid_leaf, root_clique, &order, &supernodes,
                  &separators);
  return SupernodesToData(GetMax(cliques) + 1, order, supernodes, separators);
}

MatrixData SupernodesToData(int num_vars, const std::vector<int>& order,
                            const std::vector<std::vector<int>>& supernodes,
                            const std::vector<std::vector<int>>& separators) {
  MatrixData d;
  d.clique_order = order;
  d.permutation.resize(num_vars);
  d.permutation_inverse.resize(num_vars);
  int i = 0;
  for (auto& e : order) {
    for (auto& sn_ii : supernodes.at(e)) {
      d.permutation_inverse.at(i) = sn_ii;
      d.permutation.at(sn_ii) = i;
      i++;
    }
  }

  auto& supernode_size = d.supernode_size;
  supernode_size.resize(order.size());
  d.supernodes_original_labels.resize(order.size());
  d.separators_original_labels.resize(order.size());
  d.cliques.resize(supernodes.size());
  i = 0;
  for (auto e : order) {
    d.cliques.at(i) = supernodes.at(e);

    auto temp = Relabel(separators.at(e), d.permutation);
    std::sort(temp.begin(), temp.end());
    auto sep = Relabel(temp, d.permutation_inverse);

    for (auto si : sep) {
      d.cliques.at(i).push_back(si);
    }
    d.cliques.at(i) = Relabel(d.cliques.at(i), d.permutation);
    supernode_size.at(i) = supernodes.at(e).size();

    d.supernodes_original_labels.at(i) = supernodes.at(e);
    d.separators_original_labels.at(i) = sep;

    i++;
  }
  d.N = std::accumulate(supernode_size.begin(), supernode_size.end(), 0);
  return d;
}

}  // namespace conex
