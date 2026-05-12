#include "conex/common/chordal_decomp.h"

#include <algorithm>
#include <map>
#include <numeric>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "conex/common/clique_ordering.h"
#include "conex/common/clique_tree.h"

namespace conex {
namespace {

using SpMat = Eigen::SparseMatrix<double>;

std::vector<std::vector<int>> AggregateRowSupports(
    const std::vector<SpMat>& A_list, const SpMat& B) {
  const int n = B.rows();
  std::vector<std::set<int>> adj(n);
  auto add = [&](const SpMat& M) {
    for (int k = 0; k < M.outerSize(); ++k)
      for (SpMat::InnerIterator it(M, k); it; ++it) {
        adj[it.row()].insert(it.col());
        adj[it.col()].insert(it.row());
      }
  };
  add(B);
  for (const auto& A : A_list) add(A);
  std::vector<std::vector<int>> supports(n);
  for (int i = 0; i < n; ++i)
    supports[i].assign(adj[i].begin(), adj[i].end());
  return supports;
}

// Extract principal submatrix M[idx, idx] as dense.
Eigen::MatrixXd ExtractSubmatrix(const Eigen::MatrixXd& M,
                                  const std::vector<int>& idx) {
  int s = idx.size();
  Eigen::MatrixXd sub(s, s);
  for (int i = 0; i < s; ++i)
    for (int j = 0; j < s; ++j)
      sub(i, j) = M(idx[i], idx[j]);
  return sub;
}

// Extract submatrix of sparse A at rows/cols in idx.  Returns |idx|×|idx|.
SpMat ExtractSubmatrixSparse(const SpMat& A, const std::vector<int>& idx) {
  int s = idx.size();
  std::map<int, int> inv;
  for (int i = 0; i < s; ++i) inv[idx[i]] = i;
  std::vector<Eigen::Triplet<double>> trips;
  for (int k = 0; k < A.outerSize(); ++k) {
    auto jt = inv.find(k);
    if (jt == inv.end()) continue;
    for (SpMat::InnerIterator it(A, k); it; ++it) {
      auto rt = inv.find(it.row());
      if (rt != inv.end())
        trips.emplace_back(rt->second, jt->second, it.value());
    }
  }
  SpMat sub(s, s);
  sub.setFromTriplets(trips.begin(), trips.end());
  return sub;
}

SpMat SpSymEntry(int n, int i, int j, double v) {
  SpMat M(n, n);
  M.insert(i, j) = v;
  if (i != j) M.insert(j, i) = v;
  M.makeCompressed();
  return M;
}

}  // namespace

bool HasChordalPSD(const Model& problem) {
  for (const auto& c : problem.constraints()) {
    if (auto* psd = std::get_if<Model::PSDConstraintData>(&c)) {
      if (psd->use_chordal) return true;
    }
  }
  return false;
}

std::pair<Model, ChordalExpansion> DecomposeChordalPSD(
    const Model& problem) {
  int original_n = problem.num_variables();
  Model result;
  int next_var = problem.num_variables();

  for (const auto& c : problem.constraints()) {
    if (auto* psd = std::get_if<Model::PSDConstraintData>(&c)) {
      if (!psd->use_chordal) {
        // Pass through unchanged.
        result.AddPSDConstraint(psd->A_list, psd->B, psd->vars, false);
        continue;
      }

      const int n = psd->B.rows();
      const auto& A_list = psd->A_list;
      const auto& vars = psd->vars;

      // Symmetrize.
      std::vector<SpMat> A_sym;
      for (const auto& A : A_list) {
        A_sym.push_back((A + SpMat(A.transpose())) * 0.5);
      }
      SpMat B_sym = (psd->B + SpMat(psd->B.transpose())) * 0.5;
      Eigen::MatrixXd B_dense(B_sym);

      // Compute clique tree.
      auto supports = AggregateRowSupports(A_sym, B_sym);
      std::vector<std::vector<int>> maximal_cliques;
      CliqueTree tree = MakeCliqueTreeMinDegreeFromRowSupports(
          supports, &maximal_cliques);

      int nc = tree.supernodes.size();

      // Build full clique = supernode ∪ separator for each node.
      std::vector<std::vector<int>> cliques(nc);
      for (int ci = 0; ci < nc; ++ci) {
        cliques[ci] = tree.supernodes[ci];
        cliques[ci].insert(cliques[ci].end(),
                           tree.separators[ci].begin(),
                           tree.separators[ci].end());
        std::sort(cliques[ci].begin(), cliques[ci].end());
      }

      // For each node, find the LEAF of its subtree (the lowest clique
      // in post-order that contains it).  B constants go to the leaf.
      std::vector<int> node_leaf(n, -1);
      for (int pi = 0; pi < nc; ++pi) {
        int ci = tree.post_order_position_to_clique[pi];
        for (int v : cliques[ci]) {
          if (node_leaf[v] < 0) node_leaf[v] = ci;
        }
      }

      // Allocate splitting variables.  For each non-root clique ci,
      // the separator S has |S|*(|S|+1)/2 splitting entries.
      // split_base[ci] = first global variable index for ci's split.
      std::vector<int> split_base(nc, -1);
      for (int ci = 0; ci < nc; ++ci) {
        if (tree.node_to_parent[ci] < 0) continue;  // root
        int sep = tree.separators[ci].size();
        if (sep == 0) continue;
        split_base[ci] = next_var;
        next_var += sep * (sep + 1) / 2;
      }

      // Helper: upper-triangle index for separator of clique ci.
      auto sep_var = [&](int ci, int sa, int sb) -> int {
        int a = std::min(sa, sb), b = std::max(sa, sb);
        int sep = tree.separators[ci].size();
        return split_base[ci] + a * sep - a * (a - 1) / 2 + (b - a);
      };

      // Map from matrix index to local position within a clique.
      auto local_pos = [](const std::vector<int>& clique, int v) -> int {
        auto it = std::lower_bound(clique.begin(), clique.end(), v);
        return static_cast<int>(it - clique.begin());
      };

      // Children of each clique.
      std::vector<std::vector<int>> children(nc);
      for (int ci = 0; ci < nc; ++ci) {
        int p = tree.node_to_parent[ci];
        if (p >= 0) children[p].push_back(ci);
      }

      // Process each clique.
      for (int ci = 0; ci < nc; ++ci) {
        const auto& clique = cliques[ci];
        int cs = clique.size();

        std::vector<SpMat> clique_A;
        std::vector<int> clique_vars;

        // Build B for this clique: extract B[clique, clique],
        // then zero out separator entries from each child
        // (those B entries live in the child, not here).
        Eigen::MatrixXd B_clique = ExtractSubmatrix(B_dense, clique);
        for (int child : children[ci]) {
          const auto& child_sep = tree.separators[child];
          for (int vi : child_sep) {
            for (int vj : child_sep) {
              int li = local_pos(clique, vi);
              int lj = local_pos(clique, vj);
              B_clique(li, lj) = 0;
            }
          }
        }

        // Original variables: A_k x_k enters this clique at supernode
        // positions.  For each A_k: check if it has support in this
        // clique.  If so, extract A_k[clique, clique].
        // BUT: only include contributions at positions where this
        // clique is the "owner" — i.e., positions NOT in any child's
        // separator.
        std::set<std::pair<int,int>> child_sep_positions;
        for (int child : children[ci]) {
          for (int vi : tree.separators[child])
            for (int vj : tree.separators[child])
              child_sep_positions.insert({vi, vj});
        }

        for (int k = 0; k < (int)A_sym.size(); ++k) {
          SpMat Ak_sub = ExtractSubmatrixSparse(A_sym[k], clique);
          // Zero out entries at child separator positions.
          Eigen::MatrixXd Ak_dense(Ak_sub);
          for (auto [vi, vj] : child_sep_positions) {
            int li = local_pos(clique, vi);
            int lj = local_pos(clique, vj);
            if (li < cs && lj < cs) Ak_dense(li, lj) = 0;
          }
          SpMat Ak_clean = Ak_dense.sparseView(1e-15);
          if (Ak_clean.nonZeros() > 0) {
            clique_A.push_back(std::move(Ak_clean));
            clique_vars.push_back(vars[k]);
          }
        }

        // Own separator: +T splitting vars.
        if (split_base[ci] >= 0) {
          const auto& sep = tree.separators[ci];
          int ss = sep.size();
          for (int sa = 0; sa < ss; ++sa) {
            for (int sb = sa; sb < ss; ++sb) {
              int la = local_pos(clique, sep[sa]);
              int lb = local_pos(clique, sep[sb]);
              clique_A.push_back(SpSymEntry(cs, la, lb, 1.0));
              clique_vars.push_back(sep_var(ci, sa, sb));
            }
          }
        }

        // Child separators: -T splitting vars from each child.
        for (int child : children[ci]) {
          if (split_base[child] < 0) continue;
          const auto& child_sep = tree.separators[child];
          int ss = child_sep.size();
          for (int sa = 0; sa < ss; ++sa) {
            for (int sb = sa; sb < ss; ++sb) {
              int la = local_pos(clique, child_sep[sa]);
              int lb = local_pos(clique, child_sep[sb]);
              clique_A.push_back(SpSymEntry(cs, la, lb, -1.0));
              clique_vars.push_back(sep_var(child, sa, sb));
            }
          }
        }

        result.AddPSDConstraint(
            clique_A, B_clique.sparseView(1e-15), clique_vars, false);
      }

    } else if (auto* lin = std::get_if<Model::LinearConstraintData>(&c)) {
      result.AddLinearConstraint(lin->A, lin->b, lin->vars);
    } else if (auto* quad = std::get_if<Model::QuadraticCostData>(&c)) {
      result.AddQuadraticCost(quad->Q_sparse, quad->vars);
    } else if (auto* soc = std::get_if<Model::SOCConstraintData>(&c)) {
      result.AddSOCConstraint(soc->A, soc->b, soc->vars);
    } else if (auto* eq = std::get_if<Model::EqualityConstraintData>(&c)) {
      result.AddEqualityConstraint(eq->C, eq->d, eq->primal_vars);
    }
  }

  // Extend cost with zeros for splitting variables.
  if (problem.has_linear_cost()) {
    Eigen::VectorXd cost = Eigen::VectorXd::Zero(next_var);
    cost.head(problem.linear_cost().size()) = problem.linear_cost();
    result.SetLinearCost(cost);
  }

  ChordalExpansion expansion;
  expansion.original_n = original_n;
  expansion.expanded_n = next_var;
  return {std::move(result), expansion};
}

PreprocessResult PreprocessProblem(const Model& problem) {
  PreprocessResult result;

  // Step 1: Chordal decomposition (if any PSD constraint requests it).
  Model after_chordal;
  if (HasChordalPSD(problem)) {
    auto [decomposed, chordal_exp] = DecomposeChordalPSD(problem);
    after_chordal = std::move(decomposed);
    result.chordal_expansion = chordal_exp;
  } else {
    after_chordal = problem;
    result.chordal_expansion = {problem.num_variables(),
                                 problem.num_variables()};
  }

  // Step 2: Remove structurally rank-deficient columns.
  auto [reduced, rank_exp] = RemoveStructuralRankDeficiency(after_chordal);
  result.problem = std::move(reduced);
  result.rank_expansion = rank_exp;

  return result;
}

}  // namespace conex
