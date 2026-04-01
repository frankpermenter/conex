#include "conex/tree_solver/kkt_solver_factory.h"

#include <set>
#include <unordered_map>
#include <unordered_set>

#include "conex/common/clique_ordering.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/kkt_tree_solver.h"
#include "conex/tree_solver/low_rank_diagonal_subsystem.h"
#include "conex/tree_solver/tree_utils.h"

namespace conex {
using std::vector;

template <typename T>
ContributionType ClassifyCliqueContribution(
    const T* assembler, int number_of_primal_variables) {
  for (auto v : assembler->variables()) {
    if (v >= number_of_primal_variables) {
      return ContributionType::kIndefinite;
    }
  }
  return ContributionType::kPositiveDefinite;
}

std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    const std::vector<CliqueProvider*>& clique_assemblers_ptrs_,
    int num_primal_vars,
    const SolverConfiguration& config) {
  vector<vector<int>> cliques;
  for (const auto& assembler : clique_assemblers_ptrs_) {
    auto c_cliques = assembler->get_cliques();
    cliques.insert(cliques.end(), c_cliques.begin(), c_cliques.end());
  }

  auto tree_solver_ =
      std::make_unique<::conex::SymmetricLinearSystemTreeSolver>();

  // Delayed variables: dual vars + primal vars that appear only in
  // indefinite (equality) assemblers and not in any PD assembler.
  std::unordered_set<int> pd_vars;
  for (auto* assembler : clique_assemblers_ptrs_) {
    if (assembler->is_positive_definite()) {
      for (int v : assembler->primal_variables())
        pd_vars.insert(v);
    }
  }

  vector<int> delayed_vars;
  for (auto* assembler : clique_assemblers_ptrs_) {
    // All dual variables are delayed.
    for (int v : assembler->dual_variables())
      delayed_vars.push_back(v);
    // Primal variables that appear only in indefinite assemblers.
    if (!assembler->is_positive_definite()) {
      for (int v : assembler->primal_variables()) {
        if (!pd_vars.count(v)) delayed_vars.push_back(v);
      }
    }
  }
  // Deduplicate.
  std::sort(delayed_vars.begin(), delayed_vars.end());
  delayed_vars.erase(std::unique(delayed_vars.begin(), delayed_vars.end()),
                     delayed_vars.end());

  vector<vector<int>> maximal_cliques;
  CliqueTree clique_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      cliques, &maximal_cliques, config.tree.max_merge_supernode_size,
      config.tree.supernode_reorder_method, delayed_vars);

  // --- Check if structured (low-rank) path is available ---
  // Requirements: all assemblers are PD, no delayed variables.
  bool can_use_structured = delayed_vars.empty();
  vector<SparseLinearConstraintAssembler*> slc_assemblers;
  for (auto* assembler : clique_assemblers_ptrs_) {
    auto* slc = dynamic_cast<SparseLinearConstraintAssembler*>(assembler);
    if (!slc) {
      can_use_structured = false;
      break;
    }
    slc_assemblers.push_back(slc);
  }

  // Collect raw blocks and compute per-clique ranks.
  const int num_cliques = static_cast<int>(clique_tree.supernodes.size());
  vector<bool> use_structured(num_cliques, false);

  struct RawBlock {
    Eigen::MatrixXd A;
    vector<int> variables;
    int clique_index;
  };
  vector<RawBlock> raw_blocks;

  if (can_use_structured) {
    for (auto* slc : slc_assemblers) {
      auto groups = slc->DecomposeRaw(maximal_cliques);
      for (auto& g : groups) {
        // Find smallest maximal clique containing this group.
        int best = -1;
        size_t best_size = std::numeric_limits<size_t>::max();
        for (size_t ci = 0; ci < maximal_cliques.size(); ++ci) {
          if (maximal_cliques[ci].size() >= best_size) continue;
          bool all_found = true;
          for (int v : g.variables) {
            if (std::find(maximal_cliques[ci].begin(),
                          maximal_cliques[ci].end(), v) ==
                maximal_cliques[ci].end()) {
              all_found = false;
              break;
            }
          }
          if (all_found) {
            best = static_cast<int>(ci);
            best_size = maximal_cliques[ci].size();
          }
        }
        if (best >= 0) {
          raw_blocks.push_back(
              {std::move(g.A), std::move(g.variables), best});
        }
      }
    }

    // Compute per-clique rank.
    vector<int> clique_rank(num_cliques, 0);
    for (const auto& rb : raw_blocks) {
      clique_rank[rb.clique_index] += rb.A.rows();
    }

    // Compute elimination ordering from clique tree (post-order traversal).
    // num_primal_vars passed as parameter.
    vector<int> var_to_elim(num_primal_vars, -1);
    {
      int epos = 0;
      for (int ci : clique_tree.post_order_position_to_clique) {
        for (int v : clique_tree.supernodes[ci]) {
          if (v >= 0 && v < num_primal_vars) var_to_elim[v] = epos++;
        }
      }
    }

    // Remap supernodes and separators to elimination order and sort.
    auto remapped_sn = clique_tree.supernodes;
    auto remapped_sep = clique_tree.separators;
    for (auto& sn : remapped_sn) {
      for (auto& v : sn) {
        if (v >= 0 && v < num_primal_vars) v = var_to_elim[v];
      }
      std::sort(sn.begin(), sn.end());
    }
    for (auto& sep : remapped_sep) {
      for (auto& v : sep) {
        if (v >= 0 && v < num_primal_vars) v = var_to_elim[v];
      }
      std::sort(sep.begin(), sep.end());
    }

    // Check that all children's scatter into the parent's supernode block
    // is diagonal.  A child scatters its separator Schur complement (dense)
    // into the parent.  The scatter is diagonal only when the child has at
    // most ONE separator variable overlapping the parent's supernodes
    // (a scalar 1×1 update).  Multiple overlapping variables produce
    // off-diagonal cross-terms from the dense Schur complement.
    auto has_diagonal_scatter = [&](int ci) -> bool {
      const auto& parent_sn = remapped_sn[ci];
      for (int j = 0; j < num_cliques; ++j) {
        if (clique_tree.node_to_parent[j] != ci) continue;
        const auto& child_sep = remapped_sep[j];
        int overlap_count = 0;
        for (int v : child_sep) {
          if (std::binary_search(parent_sn.begin(), parent_sn.end(), v)) {
            overlap_count++;
            if (overlap_count > 1) return false;
          }
        }
      }
      return true;
    };

    for (int ci = 0; ci < num_cliques; ++ci) {
      int sn_size = static_cast<int>(clique_tree.supernodes[ci].size());
      if (clique_rank[ci] > 0 && clique_rank[ci] < sn_size &&
          has_diagonal_scatter(ci)) {
        use_structured[ci] = true;
      }
    }
  }

  int num_structured = 0;
  for (bool s : use_structured) num_structured += s;

  // If no cliques use structured path, skip the mixed logic entirely.
  if (num_structured == 0) {
    vector<SupernodalAssemblerBase*> decomposed;
    for (auto* assembler : clique_assemblers_ptrs_) {
      auto subs = assembler->Decompose(maximal_cliques);
      decomposed.insert(decomposed.end(), subs.begin(), subs.end());
    }
    int num_primal_d = num_primal_vars;
    for (auto* assembler : decomposed) {
      auto adapter =
          std::make_unique<KKTAssemblerToSubsystemAdapter>(assembler);
      adapter->set_contribution_type(
          ClassifyCliqueContribution(assembler, num_primal_d));
      tree_solver_->push_back(std::move(adapter));
    }
    tree_solver_->SetUseGenericFactorization(
        config.tree.use_generic_factorization);
    tree_solver_->SetUseLUForIndefinite(config.tree.use_lu_for_indefinite);
    tree_solver_->Finalize(clique_tree, config.rhs_cols);
    tree_solver_->SetFactorizationMode(config.tree.left_looking);
    tree_solver_->EnableAutoUpdateAtAssemble(true);
    tree_solver_->SetNumThreads(config.num_threads);
    return tree_solver_;
  }

  // --- Build adapters and subsystems ---
  // For structured cliques: StackedLowRankAdapter + LowRankPlusDiagonalSubsystem
  // For dense cliques: standard LinearConstraint + KKTAssemblerToSubsystemAdapter

  // Group raw blocks by clique.
  vector<vector<int>> clique_block_indices(num_cliques);
  for (int bi = 0; bi < static_cast<int>(raw_blocks.size()); ++bi) {
    clique_block_indices[raw_blocks[bi].clique_index].push_back(bi);
  }

  // For dense cliques, we need the standard Decompose path.
  // Call Decompose on assemblrs to get LinearConstraints.
  vector<SupernodalAssemblerBase*> decomposed;
  for (auto* assembler : clique_assemblers_ptrs_) {
    auto subs = assembler->Decompose(maximal_cliques);
    decomposed.insert(decomposed.end(), subs.begin(), subs.end());
  }

  // Map each decomposed constraint to its clique (same logic as ClassifyCliques).
  int num_primal = num_primal_vars;
  vector<int> decomposed_to_clique(decomposed.size(), -1);
  {
    std::unordered_map<int, int> sn_to_node;
    for (int ci = 0; ci < num_cliques; ++ci) {
      for (int sn : clique_tree.supernodes[ci]) sn_to_node[sn] = ci;
    }
    for (size_t di = 0; di < decomposed.size(); ++di) {
      auto vars = decomposed[di]->variables();
      int best = -1;
      size_t best_size = std::numeric_limits<size_t>::max();
      std::set<int> candidates;
      for (int v : vars) {
        auto it = sn_to_node.find(v);
        if (it != sn_to_node.end()) candidates.insert(it->second);
      }
      for (int ci : candidates) {
        const auto& sn = clique_tree.supernodes[ci];
        const auto& sep = clique_tree.separators[ci];
        size_t total = sn.size() + sep.size();
        if (total >= best_size) continue;
        bool all_found = true;
        for (int v : vars) {
          if (std::find(sn.begin(), sn.end(), v) != sn.end()) continue;
          if (std::find(sep.begin(), sep.end(), v) != sep.end()) continue;
          all_found = false;
          break;
        }
        if (all_found) { best = ci; best_size = total; }
      }
      decomposed_to_clique[di] = best;
    }
  }

  // Add adapters: structured cliques get StackedLowRankAdapter,
  // dense cliques get standard adapters for their constraints.
  // Track which cliques have at least one adapter.
  vector<bool> clique_has_adapter(num_cliques, false);

  // First: structured cliques.
  for (int ci = 0; ci < num_cliques; ++ci) {
    if (!use_structured[ci]) continue;

    vector<StackedLowRankAdapter::Block> blocks;
    for (int bi : clique_block_indices[ci]) {
      blocks.push_back({std::move(raw_blocks[bi].A),
                         std::move(raw_blocks[bi].variables)});
    }
    vector<int> clique_vars = clique_tree.supernodes[ci];
    clique_vars.insert(clique_vars.end(),
                       clique_tree.separators[ci].begin(),
                       clique_tree.separators[ci].end());

    auto adapter = std::make_unique<StackedLowRankAdapter>(
        std::move(blocks), clique_vars);
    adapter->set_contribution_type(ContributionType::kPositiveDefinite);

    auto lr_subsystem = std::make_unique<LowRankPlusDiagonalSubsystem>();
    adapter->BindSubsystem(lr_subsystem.get());
    tree_solver_->InjectSubsystem(ci, std::move(lr_subsystem));
    tree_solver_->push_back(std::move(adapter));
    clique_has_adapter[ci] = true;
  }

  // Then: dense cliques get standard adapters.
  for (size_t di = 0; di < decomposed.size(); ++di) {
    int ci = decomposed_to_clique[di];
    if (ci >= 0 && use_structured[ci]) continue;  // skip structured

    auto adapter =
        std::make_unique<KKTAssemblerToSubsystemAdapter>(decomposed[di]);
    adapter->set_contribution_type(
        ClassifyCliqueContribution(decomposed[di], num_primal));
    tree_solver_->push_back(std::move(adapter));
    if (ci >= 0) clique_has_adapter[ci] = true;
  }

  tree_solver_->SetUseGenericFactorization(
      config.tree.use_generic_factorization);
  tree_solver_->SetUseLUForIndefinite(config.tree.use_lu_for_indefinite);
  tree_solver_->Finalize(clique_tree, config.rhs_cols);
  tree_solver_->SetFactorizationMode(config.tree.left_looking);
  tree_solver_->EnableAutoUpdateAtAssemble(true);
  tree_solver_->SetNumThreads(config.num_threads);

  return tree_solver_;
}

}  // namespace conex
