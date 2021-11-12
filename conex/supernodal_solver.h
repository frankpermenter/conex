#pragma once
#include <numeric>
#include <vector>

#include <Eigen/Dense>

#include "conex/supernodal_assembler.h"
#include "conex/triangular_matrix_workspace.h"

namespace conex {

std::vector<int> UnionOfSorted(const std::vector<int>& x1,
                               const std::vector<int>& x2);

// TODO(FrankPermenter): Deprecate this struct.
struct MatrixData {
  std::vector<std::vector<int>> cliques;
  std::vector<std::vector<int>> supernodes_original_labels;
  std::vector<std::vector<int>> separators_original_labels;
  std::vector<int> supernode_size;
  std::vector<int> permutation;
  // The map from supernode to original variable.
  std::vector<int> permutation_inverse;
  // The map from clique (with fill-in) to original clique.
  std::vector<int> clique_order;
  int N;
};

MatrixData GetData(const std::vector<std::vector<int>>& cliques, int init = 0);

MatrixData GetData(const std::vector<std::vector<int>>& cliques,
                   const std::vector<int>& valid_leafs, int init = 0);

template <typename T>
inline void DoBind(const MatrixData& data, TriangularMatrixWorkspace& workspace,
                   const std::vector<T*>& eqs) {
  auto& sn = data.supernodes_original_labels;
  auto& sep = data.separators_original_labels;

  for (int e = static_cast<int>(eqs.size()) - 1; e >= 0; e--) {
    int i = data.clique_order.at(e);

    if (sn.at(e).size() > 0) {
      auto blockD = BuildBlock(&sn.at(e), workspace.diagonal.at(e).data());
      eqs.at(i)->BindDiagonalBlock(&blockD);
    }

    if (sep.at(e).size() > 0 && sn.at(e).size() > 0) {
      auto block = BuildBlock(&sn.at(e), &sep.at(e),
                              workspace.off_diagonal.at(e).data());
      eqs.at(i)->BindOffDiagonalBlock(&block);
    }

    if (workspace.seperator_diagonal.at(e).size() > 0) {
      auto block = BuildBlock(&sep.at(e), &sep.at(e),
                              &workspace.seperator_diagonal.at(e));
      eqs.at(i)->BindOffDiagonalBlock(&block);
    }
  }
}

class SparseTriangularMatrix {
 public:
  SparseTriangularMatrix(int num_cols,
                         const std::vector<std::vector<int>>& cliques,
                         const std::vector<int>& supernode_sizes,
                         const Eigen::VectorXd& memory)
      : workspace_(cliques, supernode_sizes),
        memory_(memory),
        cliques_(cliques),
        supernode_size(workspace_.supernode_size),
        supernodes_(workspace_.diagonal),
        separator_(workspace_.off_diagonal) {
    if (memory_.size() >= SizeOf(workspace_)) {
      std::runtime_error("Invalid workspace size.");
    }
    Initialize(&workspace_, memory_.data());
  }

  SparseTriangularMatrix(int N_, const std::vector<std::vector<int>>& cliques,
                         const std::vector<int>& supernode_sizes)
      : SparseTriangularMatrix(
            N_, cliques, supernode_sizes,
            Eigen::VectorXd::Zero(
                SizeOf(TriangularMatrixWorkspace(cliques, supernode_sizes)))) {}

  SparseTriangularMatrix(const MatrixData& data)
      : SparseTriangularMatrix(data.N, data.cliques, data.supernode_size) {}

  SparseTriangularMatrix(const SparseTriangularMatrix& s)
      : SparseTriangularMatrix(s.num_columns(), s.cliques_, s.supernode_size,
                               s.memory_) {}

  SparseTriangularMatrix operator=(const SparseTriangularMatrix& s) {
    return SparseTriangularMatrix(s.num_columns(), s.cliques_, s.supernode_size,
                                  s.memory_);
  }

 public:
  int num_columns() const { return workspace_.num_columns(); }

  std::vector<Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>>& supernodes() {
    return supernodes_;
  }

  std::vector<Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>>& separator() {
    return separator_;
  }

  std::vector<std::vector<int>>& cliques() { return cliques_; }

  double coeff(int i, int j) const;
  Eigen::MatrixXd MakeDenseMatrix() const;
  void SetConstant(double val);

  TriangularMatrixWorkspace workspace_;

 private:
  Eigen::VectorXd memory_;
  std::vector<std::vector<int>> cliques_;
  std::vector<int>& supernode_size;
  std::vector<Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>>& supernodes_;
  std::vector<Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>>& separator_;

  friend class SupernodalCholeskyFactorization;
  friend class SupernodalKKTSolver;
};

std::vector<std::vector<int>> Permute(std::vector<std::vector<int>>& path,
                                      std::vector<int>& permutation);
void Sort(std::vector<std::vector<int>>* path);

void IntersectionOfSorted(const std::vector<int>& v1,
                          const std::vector<int>& v2, std::vector<int>* v3);

namespace TriangularMatrixOperations {
void CholeskyInPlace(SparseTriangularMatrix* mat);
Eigen::VectorXd ApplyInverse(SparseTriangularMatrix* L,
                             const Eigen::VectorXd& b);
Eigen::VectorXd ApplyInverseOfTranspose(SparseTriangularMatrix* L,
                                        const Eigen::VectorXd& b);
};  // namespace TriangularMatrixOperations

MatrixData SupernodesToData(int num_vars, const std::vector<int>& order,
                            const std::vector<std::vector<int>>& supernodes,
                            const std::vector<std::vector<int>>& separators);

}  // namespace conex
