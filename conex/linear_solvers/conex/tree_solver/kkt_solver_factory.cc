#include "conex/tree_solver/kkt_solver_factory.h"

#include <set>
#include <unordered_map>

#include "conex/common/clique_ordering.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/kkt_tree_solver.h"
#include "conex/tree_solver/low_rank_diagonal_subsystem.h"
#include "conex/tree_solver/tree_utils.h"

namespace conex {
using std::vector;

ContributionType ClassifyCliqueContribution(
    const SupernodalAssemblerBase* assembler, int number_of_primal_variables) {
  for (auto v : assembler->variables()) {
    if (v >= number_of_primal_variables) {
      return ContributionType::kIndefinite;
    }
  }
  return ContributionType::kPositiveDefinite;
}

std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    ConstraintManager* c, const SolverConfiguration& config) {
  auto clique_assemblers_ptrs_ = c->clique_assemblers();
  vector<vector<int>> cliques;
  for (const auto& assembler : clique_assemblers_ptrs_) {
    auto c_cliques = assembler->get_cliques();
    cliques.insert(cliques.end(), c_cliques.begin(), c_cliques.end());
  }

  auto tree_solver_ =
      std::make_unique<::conex::SymmetricLinearSystemTreeSolver>();

  vector<int> dual_vars_flat;
  for (const auto& dv : c->equality_constraint_multipliers()) {
    dual_vars_flat.insert(dual_vars_flat.end(), dv.begin(), dv.end());
  }

  vector<vector<int>> maximal_cliques;
  CliqueTree clique_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      cliques, &maximal_cliques, config.tree.max_merge_supernode_size,
      config.tree.supernode_reorder_method, dual_vars_flat);

  // --- Check if structured (low-rank) path is available ---
  // Requirements: all assemblers are SLC, no dual variables.
  bool can_use_structured = dual_vars_flat.empty();
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
    for (int ci = 0; ci < num_cliques; ++ci) {
      int sn_size = static_cast<int>(clique_tree.supernodes[ci].size());
      // Only use structured when rank is meaningfully smaller than sn_size.
      // With ratio threshold 0.5, a 10-supernode clique needs rank < 5.
      if (clique_rank[ci] > 0 && clique_rank[ci] * 2 < sn_size) {
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
      if (config.tree.precompute_gram) {
        assembler->set_precompute_gram(true);
      }
    }
    int num_primal_d = c->GetNumberOfVariables();
    for (auto* assembler : decomposed) {
      auto adapter =
          std::make_unique<KKTAssemblerToSubsystemAdapter>(assembler);
      adapter->set_contribution_type(
          ClassifyCliqueContribution(assembler, num_primal_d));
      tree_solver_->push_back(std::move(adapter));
    }
    tree_solver_->SetUseGenericFactorization(
        config.tree.use_generic_factorization);
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
    if (config.tree.precompute_gram) {
      assembler->set_precompute_gram(true);
    }
  }

  // Map each decomposed constraint to its clique (same logic as ClassifyCliques).
  int num_primal = c->GetNumberOfVariables();
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
  tree_solver_->Finalize(clique_tree, config.rhs_cols);
  tree_solver_->SetFactorizationMode(config.tree.left_looking);
  tree_solver_->EnableAutoUpdateAtAssemble(true);
  tree_solver_->SetNumThreads(config.num_threads);

  return tree_solver_;
}

}  // namespace conex
