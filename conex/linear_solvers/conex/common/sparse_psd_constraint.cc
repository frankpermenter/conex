#include "conex/common/sparse_psd_constraint.h"

#include <algorithm>
#include <numeric>
#include <set>
#include <unordered_map>

#include "conex/common/clique_ordering.h"
#include "conex/common/psd_constraint.h"
#include "conex/common/psd_cone_ops.h"

namespace conex {

namespace {

// Compute the aggregate sparsity pattern: union of B and all A_i,
// symmetrized.  Returns row supports (sorted column indices per row).
std::vector<std::vector<int>> AggregateRowSupports(
    const std::vector<Eigen::SparseMatrix<double>>& A_list,
    const Eigen::SparseMatrix<double>& B) {
  const int n = B.rows();
  std::vector<std::set<int>> adj(n);

  auto add_matrix = [&](const Eigen::SparseMatrix<double>& M) {
    for (int k = 0; k < M.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(M, k); it; ++it) {
        adj[it.row()].insert(it.col());
        adj[it.col()].insert(it.row());
      }
  };

  add_matrix(B);
  for (const auto& Ak : A_list) add_matrix(Ak);

  std::vector<std::vector<int>> supports(n);
  for (int i = 0; i < n; ++i)
    supports[i].assign(adj[i].begin(), adj[i].end());
  return supports;
}

// Check if sparse matrix A has any nonzero in the principal submatrix
// indexed by `indices`.
bool HasNonzeroInSubblock(const Eigen::SparseMatrix<double>& A,
                          const std::vector<int>& indices) {
  std::set<int> idx_set(indices.begin(), indices.end());
  for (int k = 0; k < A.outerSize(); ++k) {
    if (!idx_set.count(k)) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      if (idx_set.count(it.row())) return true;
    }
  }
  return false;
}

// Extract the principal submatrix A[S,S] as a dense matrix.
Eigen::SparseMatrix<double> ExtractPrincipalSubmatrixSparse(
    const Eigen::SparseMatrix<double>& A,
    const std::vector<int>& indices) {
  const int m = static_cast<int>(indices.size());
  std::unordered_map<int, int> idx_map;
  for (int i = 0; i < m; ++i) idx_map[indices[i]] = i;

  std::vector<Eigen::Triplet<double>> trips;
  for (int k = 0; k < A.outerSize(); ++k) {
    auto jt = idx_map.find(k);
    if (jt == idx_map.end()) continue;
    int lj = jt->second;
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      auto it2 = idx_map.find(it.row());
      if (it2 != idx_map.end())
        trips.emplace_back(it2->second, lj, it.value());
    }
  }
  Eigen::SparseMatrix<double> sub(m, m);
  sub.setFromTriplets(trips.begin(), trips.end());
  return sub;
}

Eigen::MatrixXd ExtractPrincipalSubmatrix(
    const Eigen::SparseMatrix<double>& A,
    const std::vector<int>& indices) {
  const int m = static_cast<int>(indices.size());
  std::unordered_map<int, int> idx_map;
  for (int i = 0; i < m; ++i) idx_map[indices[i]] = i;

  Eigen::MatrixXd sub = Eigen::MatrixXd::Zero(m, m);
  for (int k = 0; k < A.outerSize(); ++k) {
    auto jt = idx_map.find(k);
    if (jt == idx_map.end()) continue;
    int lj = jt->second;
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      auto it2 = idx_map.find(it.row());
      if (it2 != idx_map.end()) sub(it2->second, lj) = it.value();
    }
  }
  return sub;
}

// Vectorize: vec(M) in column-major order.
Eigen::VectorXd Vectorize(const Eigen::MatrixXd& M) {
  return Eigen::Map<const Eigen::VectorXd>(M.data(), M.size());
}

}  // namespace

SparsePSDConstraintAssembler::SparsePSDConstraintAssembler(
    const std::vector<Eigen::SparseMatrix<double>>& A_list,
    const Eigen::SparseMatrix<double>& B,
    const std::vector<int>& vars,
    bool use_chordal)
    : CliqueProvider(vars), B_(B), n_(B.rows()) {
  // Symmetrize A_i: the PSD constraint Σ A_i x_i + B ≽ 0 only sees
  // the symmetric part of each A_i (since X is symmetric).
  A_list_.reserve(A_list.size());
  for (const auto& Ak : A_list) {
    Eigen::SparseMatrix<double> sym = (Ak + Eigen::SparseMatrix<double>(Ak.transpose())) * 0.5;
    A_list_.push_back(sym);
  }
  // Also symmetrize B.
  B_ = (B + Eigen::SparseMatrix<double>(B.transpose())) * 0.5;
  std::vector<std::vector<int>> maximal_cliques;

  if (use_chordal) {
    // Compute aggregate sparsity and find matrix-space maximal cliques.
    auto supports = AggregateRowSupports(A_list_, B_);
    MakeCliqueTreeMinDegreeFromRowSupports(supports, &maximal_cliques);
  } else {
    // Single clique: all matrix indices.
    std::vector<int> all(n_);
    std::iota(all.begin(), all.end(), 0);
    maximal_cliques.push_back(all);
  }

  // For each matrix clique, determine which optimization variables are active.
  for (auto& clique : maximal_cliques) {
    std::sort(clique.begin(), clique.end());
    MatrixClique mc;
    mc.indices = clique;
    for (int k = 0; k < static_cast<int>(A_list_.size()); ++k) {
      if (HasNonzeroInSubblock(A_list_[k], clique)) {
        mc.var_indices.push_back(k);
        mc.opt_vars.push_back(vars[k]);
      }
    }
    matrix_cliques_.push_back(std::move(mc));
  }
}

std::vector<std::vector<int>>
SparsePSDConstraintAssembler::get_cliques() const {
  // Report optimization-variable cliques: one per matrix sub-block.
  // Like SparseQuadraticTermAssembler, this tells the tree builder
  // which optimization variables are coupled by each sub-block.
  std::vector<std::vector<int>> cliques;
  for (const auto& mc : matrix_cliques_) {
    if (!mc.opt_vars.empty()) cliques.push_back(mc.opt_vars);
  }
  return cliques;
}

std::vector<SupernodalAssemblerBase*>
SparsePSDConstraintAssembler::Decompose(
    const std::vector<std::vector<int>>& maximal_cliques) {
  const auto& vars = primal_variables();

  // Build variable-to-index map (global opt var → local index in vars).
  std::unordered_map<int, int> var_to_idx;
  for (int i = 0; i < static_cast<int>(vars.size()); ++i)
    var_to_idx[vars[i]] = i;

  // Build clique membership for assignment.
  int num_tree_cliques = static_cast<int>(maximal_cliques.size());
  std::vector<std::set<int>> tree_clique_sets(num_tree_cliques);
  for (int ci = 0; ci < num_tree_cliques; ++ci)
    tree_clique_sets[ci].insert(maximal_cliques[ci].begin(),
                                maximal_cliques[ci].end());

  // Assign each matrix sub-block to the smallest tree clique containing
  // all its optimization variables.
  std::vector<SupernodalAssemblerBase*> result;
  for (const auto& mc : matrix_cliques_) {
    // Find the smallest tree clique containing mc.opt_vars.
    int best = -1;
    size_t best_size = std::numeric_limits<size_t>::max();
    for (int ci = 0; ci < num_tree_cliques; ++ci) {
      if (maximal_cliques[ci].size() >= best_size) continue;
      bool contains_all = true;
      for (int v : mc.opt_vars) {
        if (!tree_clique_sets[ci].count(v)) { contains_all = false; break; }
      }
      if (contains_all) {
        best = ci;
        best_size = maximal_cliques[ci].size();
      }
    }
    if (best < 0) continue;  // shouldn't happen

    // Extract principal submatrices and vectorize.
    const int m = static_cast<int>(mc.indices.size());
    const int m2 = m * m;
    const int p = static_cast<int>(mc.var_indices.size());

    // Extract sparse principal submatrices.
    std::vector<Eigen::SparseMatrix<double>> A_sub(p);
    for (int k = 0; k < p; ++k) {
      A_sub[k] = ExtractPrincipalSubmatrixSparse(
          A_list_[mc.var_indices[k]], mc.indices);
    }
    Eigen::SparseMatrix<double> B_sub =
        ExtractPrincipalSubmatrixSparse(B_, mc.indices);

    auto constraint = std::make_unique<PSDConstraint>(m, A_sub, B_sub);
    constraint->SetPrimalVariables(mc.opt_vars);

    // Allocate workspace.
    size_t bytes = constraint->RequiredArenaBytes();
    owned_workspace_memory_.emplace_back(bytes / sizeof(double) + 1);
    constraint->BindArenaMemory(owned_workspace_memory_.back().data(), bytes);

    result.push_back(constraint.get());
    owned_constraints_.push_back(std::move(constraint));
  }
  return result;
}

}  // namespace conex
