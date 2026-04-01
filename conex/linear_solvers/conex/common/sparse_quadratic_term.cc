#include "conex/common/sparse_quadratic_term.h"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <set>
#include <unordered_map>

#include "conex/common/clique_ordering.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/sparse_linear_constraint.h"

namespace conex {

SparseQuadraticTermAssembler::SparseQuadraticTermAssembler(
    const Eigen::SparseMatrix<double>& Q,
    const std::vector<int>& variables)
    : CliqueProvider(variables),
      Q_sparse_(&Q) {}

SparseQuadraticTermAssembler::SparseQuadraticTermAssembler(
    const Eigen::MatrixXd& Q,
    const std::vector<int>& variables)
    : CliqueProvider(variables),
      Q_dense_(&Q), dense_(true) {}

std::vector<std::vector<int>> SparseQuadraticTermAssembler::get_cliques() const {
  // Return edges {i,j} as 2-element cliques.
  // For dense Q, return full row supports.
  std::vector<std::vector<int>> cliques;
  const auto& vars = primal_variables();

  if (dense_ && Q_dense_) {
    // Dense Q: each row is a clique (all variables connected).
    // Return one big clique.
    cliques.push_back(vars);
  } else if (Q_sparse_) {
    // Sparse Q: return individual edges for off-diagonal entries,
    // and singletons for diagonal-only variables.
    std::vector<bool> has_edge(vars.size(), false);
    for (int k = 0; k < Q_sparse_->outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(*Q_sparse_, k);
           it; ++it) {
        int r = it.row(), c = it.col();
        if (r > c && r < static_cast<int>(vars.size()) &&
            c < static_cast<int>(vars.size())) {
          cliques.push_back({vars[c], vars[r]});
          has_edge[r] = true;
          has_edge[c] = true;
        }
      }
    }
    // Include diagonal-only variables as singletons so they appear
    // in the clique tree.
    for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
      if (!has_edge[i] && Q_sparse_->coeff(i, i) != 0) {
        cliques.push_back({vars[i]});
      }
    }
  }
  return cliques;
}

std::vector<SupernodalAssemblerBase*> SparseQuadraticTermAssembler::Decompose(
    const std::vector<std::vector<int>>& maximal_cliques) {
  owned_sub_assemblers_.clear();
  const auto& vars = primal_variables();

  // Build variable-to-index map (original var -> Q index).
  std::unordered_map<int, int> var_to_idx;
  for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
    var_to_idx[vars[i]] = i;
  }

  // For each pair (i,j) in Q, assign it to the SMALLEST maximal clique
  // containing both i and j.  This avoids double-counting entries that
  // appear in separator variables shared between cliques.
  int num_cliques = static_cast<int>(maximal_cliques.size());

  // Build clique membership: var -> set of clique indices.
  std::unordered_map<int, std::vector<int>> var_to_cliques;
  for (int ci = 0; ci < num_cliques; ++ci) {
    for (int v : maximal_cliques[ci]) {
      var_to_cliques[v].push_back(ci);
    }
  }

  // Per-clique Q sub-blocks, initialized to zero.
  struct CliqueInfo {
    std::vector<int> q_indices;  // Q indices for vars in this clique
    std::vector<int> clique_vars;  // original variable indices
    std::unordered_map<int, int> var_to_local;  // var -> local index
  };
  std::vector<CliqueInfo> infos(num_cliques);
  std::vector<Eigen::MatrixXd> Q_blocks;

  for (int ci = 0; ci < num_cliques; ++ci) {
    auto& info = infos[ci];
    for (int v : maximal_cliques[ci]) {
      auto it = var_to_idx.find(v);
      if (it != var_to_idx.end()) {
        info.var_to_local[v] = static_cast<int>(info.q_indices.size());
        info.q_indices.push_back(it->second);
        info.clique_vars.push_back(v);
      }
    }
    int m = static_cast<int>(info.q_indices.size());
    Q_blocks.emplace_back(Eigen::MatrixXd::Zero(m, m));
  }

  // Assign each Q(i,j) to the smallest containing clique.
  auto get_Q = [&](int qi, int qj) -> double {
    if (dense_ && Q_dense_) return (*Q_dense_)(qi, qj);
    if (Q_sparse_) return Q_sparse_->coeff(qi, qj);
    return 0;
  };

  for (int qi = 0; qi < static_cast<int>(vars.size()); ++qi) {
    int vi = vars[qi];
    for (int qj = qi; qj < static_cast<int>(vars.size()); ++qj) {
      int vj = vars[qj];
      double val = get_Q(qi, qj);
      if (val == 0 && qi != qj) continue;
      if (val == 0 && qi == qj) continue;

      // Find smallest clique containing both vi and vj.
      int best_ci = -1;
      size_t best_size = std::numeric_limits<size_t>::max();
      auto it_i = var_to_cliques.find(vi);
      if (it_i == var_to_cliques.end()) continue;
      for (int ci : it_i->second) {
        if (maximal_cliques[ci].size() >= best_size) continue;
        if (infos[ci].var_to_local.count(vj)) {
          best_ci = ci;
          best_size = maximal_cliques[ci].size();
        }
      }
      if (best_ci < 0) continue;

      int li = infos[best_ci].var_to_local[vi];
      int lj = infos[best_ci].var_to_local[vj];
      Q_blocks[best_ci](li, lj) += val;
      if (li != lj) Q_blocks[best_ci](lj, li) += val;
    }
  }

  // Create sub-assemblers for non-zero blocks.
  std::vector<SupernodalAssemblerBase*> result;
  for (int ci = 0; ci < num_cliques; ++ci) {
    if (Q_blocks[ci].squaredNorm() < 1e-30) continue;
    owned_sub_assemblers_.emplace_back(
        std::move(Q_blocks[ci]), infos[ci].clique_vars);
    result.push_back(&owned_sub_assemblers_.back());
  }

  return result;
}

}  // namespace conex
