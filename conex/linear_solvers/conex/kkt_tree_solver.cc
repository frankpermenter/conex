#include "conex/kkt_tree_solver.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <limits>
#include <set>
#include <thread>
#include <unordered_map>

#include "conex/cholesky_solvers.h"
#include "conex/debug_macros.h"
#include "conex/static_subsystem.h"
#include "conex/tree_utils.h"

namespace conex {

void SupernodePartitionMatrix::SetPartition(
    const std::vector<KKTSubsystemBase*>& subsystems, int num_vars,
    const Eigen::VectorXi& perm_inv) {
  const int n = static_cast<int>(subsystems.size());
  blocks_.resize(n);
  for (int k = 0; k < n; ++k) {
    const auto& sn = subsystems[k]->supernodes();
    blocks_[k].sn_rows = static_cast<int>(sn.size());
    blocks_[k].sep_rows = static_cast<int>(subsystems[k]->separators().size());
    blocks_[k].sn_start = sn.empty() ? 0 : sn.front();
    blocks_[k].supernode_data = nullptr;
    blocks_[k].separator_data = nullptr;
  }

  var_mapping_.resize(num_vars);
  for (int k = 0; k < n; ++k) {
    const auto& sn = subsystems[k]->supernodes();
    for (int i = 0; i < static_cast<int>(sn.size()); ++i) {
      int orig_var = perm_inv(sn[i]);
      var_mapping_[orig_var] = {k, i};
    }
  }

  cols_ = 0;
  arena_.reset();
  arena_bytes_ = 0;
}

void SupernodePartitionMatrix::Resize(int cols) {
  if (blocks_.empty()) return;
  cols_ = cols;

  constexpr size_t kAlign = EIGEN_MAX_ALIGN_BYTES;
  size_t cursor = 0;
  for (const auto& blk : blocks_) {
    cursor = ((cursor + kAlign - 1) / kAlign) * kAlign;
    cursor += blk.sn_rows * cols * sizeof(double);
    cursor = ((cursor + kAlign - 1) / kAlign) * kAlign;
    cursor += blk.sep_rows * cols * sizeof(double);
  }

  if (cursor > arena_bytes_) {
    const size_t alloc_bytes = ((cursor + kAlign - 1) / kAlign) * kAlign;
    void* raw_ptr = nullptr;
    if (posix_memalign(&raw_ptr, kAlign, alloc_bytes) != 0) {
      throw std::bad_alloc();
    }
    arena_.reset(raw_ptr);
    arena_bytes_ = alloc_bytes;
  }

  // Recompute block pointers for current column count.
  char* base = static_cast<char*>(arena_.get());
  cursor = 0;
  for (auto& blk : blocks_) {
    cursor = ((cursor + kAlign - 1) / kAlign) * kAlign;
    blk.supernode_data = reinterpret_cast<double*>(base + cursor);
    cursor += blk.sn_rows * cols * sizeof(double);
    cursor = ((cursor + kAlign - 1) / kAlign) * kAlign;
    blk.separator_data = reinterpret_cast<double*>(base + cursor);
    cursor += blk.sep_rows * cols * sizeof(double);
  }
}

void SupernodePartitionMatrix::SetZero() {
  if (arena_bytes_ > 0) {
    std::memset(arena_.get(), 0, arena_bytes_);
  }
}

void SupernodePartitionMatrix::ScatterFrom(
    Eigen::Ref<const Eigen::MatrixXd> b) {
  const int n = static_cast<int>(var_mapping_.size());
  for (int v = 0; v < n; ++v) {
    const auto& m = var_mapping_[v];
    Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> sn(
        blocks_[m.block_index].supernode_data,
        blocks_[m.block_index].sn_rows, cols_);
    sn.row(m.row_in_block) = b.row(v);
  }
}

void SupernodePartitionMatrix::GatherInto(
    Eigen::Ref<Eigen::MatrixXd> b) const {
  const int n = static_cast<int>(var_mapping_.size());
  for (int v = 0; v < n; ++v) {
    const auto& m = var_mapping_[v];
    Eigen::Map<const Eigen::MatrixXd, Eigen::Aligned> sn(
        blocks_[m.block_index].supernode_data,
        blocks_[m.block_index].sn_rows, cols_);
    b.row(v) = sn.row(m.row_in_block);
  }
}

void SupernodePartitionMatrix::ScatterFromElimOrder(
    Eigen::Ref<const Eigen::MatrixXd> b) {
  for (const auto& blk : blocks_) {
    if (blk.sn_rows > 0) {
      Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> sn(
          blk.supernode_data, blk.sn_rows, cols_);
      sn = b.middleRows(blk.sn_start, blk.sn_rows);
    }
  }
}

void SupernodePartitionMatrix::GatherIntoElimOrder(
    Eigen::Ref<Eigen::MatrixXd> b) const {
  for (const auto& blk : blocks_) {
    if (blk.sn_rows > 0) {
      Eigen::Map<const Eigen::MatrixXd, Eigen::Aligned> sn(
          blk.supernode_data, blk.sn_rows, cols_);
      b.middleRows(blk.sn_start, blk.sn_rows) = sn;
    }
  }
}

using KKTSubsystemType = KKTSubsystemBase;
using Eigen::MatrixXd;
namespace {
template <typename Fn>
void ForEachTask(size_t num_tasks, int requested_threads, const Fn& fn) {
  if (num_tasks == 0) {
    return;
  }
  // Thread startup/join dominates for small task counts.
  if (requested_threads < 1 || num_tasks == 1 || num_tasks < 8) {
    for (size_t i = 0; i < num_tasks; ++i) {
      fn(i);
    }
    return;
  }
  const size_t num_threads = std::min<size_t>(requested_threads, num_tasks);
  std::atomic<size_t> next_task(0);
  std::vector<std::thread> workers;
  workers.reserve(num_threads);
  for (size_t t = 0; t < num_threads; ++t) {
    workers.emplace_back([&]() {
      while (true) {
        const size_t i = next_task.fetch_add(1, std::memory_order_relaxed);
        if (i >= num_tasks) {
          return;
        }
        fn(i);
      }
    });
  }
  for (auto& worker : workers) {
    worker.join();
  }
}

int EffectiveThreadCount(int requested_threads) {
  if (requested_threads > 0) {
    return requested_threads;
  }
  const unsigned int hardware_threads = std::thread::hardware_concurrency();
  if (hardware_threads == 0) {
    return 1;
  }
  return static_cast<int>(hardware_threads);
}

#if 0
class DistanceToRootRecursion {
 public:
  DistanceToRootRecursion(const std::vector<int>& parent)
      : parent_(parent), distance_(parent.size(), -1) {}
  int ComputeDistanceToRootHelper(int i) {
    if (distance_[i] >= 0) {
      return distance_[i];
    } else {
      int distance_of_new = 0;
      if (parent_.at(i) != -1) {
        distance_of_new = 1 + ComputeDistanceToRootHelper(parent_.at(i));
      }
      distance_[i] = distance_of_new;
      return distance_[i];
    }
  }
  std::vector<int> Compute() {
    for (size_t i = 0; i < parent_.size(); i++) {
      ComputeDistanceToRootHelper(i);
    }
    return distance_;
  }

 private:
  std::vector<int> parent_;
  std::vector<int> distance_;
};

std::vector<int> ComputeDistanceToRoot(const std::vector<int>& parent) {
  std::vector<int> distance_to_root;
  return DistanceToRootRecursion(parent).Compute();
}

void FillIn(std::vector<int> system_to_parent, int num_variables,
            std::vector<KKTSubsystemType*>* systems) {
  std::vector<int> system_to_distance_to_root =
      ComputeDistanceToRoot(system_to_parent);
  std::vector<int> eliminated(num_variables, -1);

  // Detect if variable is a supernode of clique i and
  // clique j.  If so, apply running intersection property
  // to the path from clique i and to clique j:
  //  1) Make a supernode of the clique closest to the root.
  //  2) Make a separator of all other cliques.
  for (size_t i = 0; i < systems->size(); ++i) {
    for (int v : systems->at(i)->supernodes()) {
      const bool variable_already_eliminated = eliminated.at(v) > -1;
      if (variable_already_eliminated) {
        auto path_in_tree = PathInForest(i, eliminated.at(v), system_to_parent,
                                         system_to_distance_to_root);
        for (size_t j = 0; j < path_in_tree.size() - 1; j++) {
          auto e = path_in_tree.at(j);
          systems->at(e)->AddSeparator(v);
        }
        eliminated.at(v) = path_in_tree.back();
      } else {
        eliminated.at(v) = i;
      }
    }
  }

  for (auto& s : *systems) {
    s->SetSupernodes({});
  }

  for (size_t i = 0; i < eliminated.size(); i++) {
    if (eliminated.at(i) != -1) {
      systems->at(eliminated.at(i))->AddSupernode(i);
    }
  }
}

void IntersectionOfSorted(const std::vector<int>& v1,
                          const std::vector<int>& v2, std::vector<int>* v3) {
  v3->clear();
  std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                        back_inserter(*v3));
}

int LinearIndex(int i, int j, int n) {
  if (i > j) {
    return j * n + i;
  } else {
    return i * n + j;
  }
}

int GetUnvisited(const std::vector<int>& x) {
  int cnt = 0;
  for (auto xi : x) {
    if (xi == 0) {
      return cnt;
    }
    cnt++;
  }
  return -1;
}

template <typename T>
class SymmetricMatrix {
 public:
  SymmetricMatrix(int n) : n_(n), data_(n * n) {}
  vector<int>& operator()(int a, int b) {
    return data_.at(LinearIndex(a, b, n_));
  }
  const vector<int>& operator()(int a, int b) const {
    return data_.at(LinearIndex(a, b, n_));
  }
  int n_;
  vector<T> data_;
};

class Weight {
 public:
  Weight(SymmetricMatrix<vector<int>>& intersections,
         const vector<KKTSubsystemType*>& cliques_sorted)
      : intersections_(intersections), subsystems_(cliques_sorted) {}
  int num_nodes_;
  SymmetricMatrix<vector<int>>& intersections_;
  const vector<KKTSubsystemType*>& subsystems_;

  size_t get_weight(int active, int i) {
    // Weight is the size of intersection.
    if (intersections_(active, i).size() == 0) {
      IntersectionOfSorted(subsystems_.at(active)->shared_variables(),
                           subsystems_.at(i)->shared_variables(),
                           &intersections_(active, i));
    }
    return intersections_(active, i).size();
  }
};

/*
Visit nodes of clique intersection graph using weighted DFS.
*/
int PickCliqueOrderHelper(const std::vector<KKTSubsystemType*>& subsystems,
                          int root_in, bool validate_leaf_nodes,
                          SymmetricMatrix<vector<int>>* intersections_ptr,
                          RootedTree* tree_ptr) {
  auto& tree = *tree_ptr;
  auto& intersections = *intersections_ptr;
  int n = subsystems.size();
  Weight edge_weights(intersections, subsystems);
  CONEX_ASSERT(root_in < n, "Invalid root node.");

  vector<int> visited(n, 0);
  std::stack<int> node_stack;
  int root = root_in;
  if (root < 0) {
    root = 0;
  }

  node_stack.push(root);
  int num_visited = 0;
  while (num_visited < n) {
    int active = node_stack.top();
    if (visited.at(active) == 0) {
      visited.at(active) = 1;
      num_visited++;
      tree.parent.at(active) = -1;
    }

    // Find unvisited neighbor with maximum weight.
    size_t max_weight = 1;
    vector<int> argmax;
    for (int i = 0; i < n; i++) {
      if (i == active || visited.at(i) == 1) {
        continue;
      }

      auto current_weight = edge_weights.get_weight(active, i);
      if (current_weight >= max_weight) {
        if (current_weight > max_weight) {
          argmax.clear();
          max_weight = current_weight;
        }
        argmax.push_back(i);
      }
    }

    for (auto e : argmax) {
      node_stack.push(e);
      visited.at(e) = 1;
      num_visited++;
      tree.parent.at(e) = active;
    }

    // Process leaf node.
    if (argmax.size() == 0) {
      node_stack.pop();
      // If node is invalid leaf node, move it up the
      // tree until a valid leaf is reached.  This
      // leads to the following transformation:
      //
      //     R            I*
      //    I  V          I
      //    I  V   =>     I
      //   *I  V          R
      //                  V
      //                  V
      //                  V
      if (validate_leaf_nodes) {
        int final_leaf_position = active;
        while (!subsystems.at(final_leaf_position)->is_valid_leaf()) {
          final_leaf_position = tree.parent.at(final_leaf_position);
          if (final_leaf_position == -1) {
            throw std::runtime_error("System is not full rank.");
          }
        }
        if (active != final_leaf_position) {
          tree.SwapPositions(active, final_leaf_position);
        }
      }

      if (node_stack.size() == 0) {
        auto node = GetUnvisited(visited);
        if (node == -1) {
          break;
        } else {
          node_stack.push(node);
        }
      }
    }
  }
  return -1;
}

#endif
}  // namespace
using T = SymmetricLinearSystemTreeSolver;
namespace {
int SubsystemThreadCount(int tree_threads, bool roots_only) {
  return roots_only ? 1 : tree_threads;
}
}  // namespace

void T::SetNumThreads(int num_threads) {
  CONEX_DEMAND(num_threads > 0, "num_threads must be positive.");
  num_threads_ = num_threads;
  const int subsystem_threads =
      SubsystemThreadCount(num_threads_, parallelize_roots_only_);
  for (auto* subsystem : subsystems_) {
    subsystem->SetNumThreads(subsystem_threads);
  }
}

void T::SetParallelizeRootsOnly(bool enable) {
  parallelize_roots_only_ = enable;
  const int subsystem_threads =
      SubsystemThreadCount(num_threads_, parallelize_roots_only_);
  for (auto* subsystem : subsystems_) {
    subsystem->SetNumThreads(subsystem_threads);
  }
}

void T::SetEliminationOrder(
    const std::vector<int>& variable_to_elimination_position) {
  variable_to_elimination_position_ = variable_to_elimination_position;
  for (auto s : subsystems_) {
    s->SetVariableOrdering(variable_to_elimination_position_);
  }
  for (auto& s : assembler_to_subsystem_adapter_) {
    s->SetEliminationPosition(variable_to_elimination_position);
  }
  // Cache permutation vectors and number_of_variables for fast solve.
  cached_num_vars_ = number_of_variables();
  const int n = cached_num_vars_;
  cached_perm_.resize(n);
  cached_perm_inv_.resize(n);
  for (int i = 0; i < n; ++i) {
    cached_perm_(i) = variable_to_elimination_position_[i];
    cached_perm_inv_(variable_to_elimination_position_[i]) = i;
  }
}
void T::DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                       bool in_original_order) const {
  const int n = cached_num_vars_ > 0 ? cached_num_vars_ : number_of_variables();
  CONEX_CHECK(b.rows() == n);
  if (b.cols() == 0) {
    return;
  }
  // Ensure per-subsystem temporary solve buffers are sized for this RHS width.
  if (b.cols() > reserved_solve_workspace_cols_) {
    ForEachTask(
        roots_.size(), EffectiveThreadCount(num_threads_),
        [&](size_t i) { roots_.at(i)->ReserveSolveWorkspace(b.cols()); });
    reserved_solve_workspace_cols_ = b.cols();
  }

  if (!solve_matrix_.empty()) {
    // Block-partitioned path: scatter into per-node blocks, solve, gather.
    if (solve_matrix_.cols() != b.cols()) {
      solve_matrix_.Resize(b.cols());
    }
    solve_matrix_.SetZero();
    if (in_original_order) {
      solve_matrix_.ScatterFrom(b);
    } else {
      solve_matrix_.ScatterFromElimOrder(b);
    }

    // Forward pass (post-order).
    const int num_solve = static_cast<int>(solve_order_.size());
    for (int idx = 0; idx < num_solve; ++idx) {
      const auto& info = solve_scatter_info_[idx];
      const int k = info.block_index;
      auto sn = solve_matrix_.supernode(k);
      auto sep = solve_matrix_.separator(k);

      // Scatter children's separator outputs into this node's blocks.
      for (const auto& cop : info.children) {
        auto child_sep = solve_matrix_.separator(cop.child_block_index);
        for (const auto& off : cop.sn_offsets) {
          sn.middleRows(off.first, off.size) -=
              child_sep.middleRows(off.second, off.size);
        }
        for (const auto& off : cop.sep_offsets) {
          sep.middleRows(off.first, off.size) +=
              child_sep.middleRows(off.second, off.size);
        }
      }

      solve_order_[idx]->ForwardSolveBlocked(sn, sep);
    }

    // Backward pass (reverse post-order).
    for (int idx = num_solve - 1; idx >= 0; --idx) {
      const auto& info = solve_scatter_info_[idx];
      const int k = info.block_index;
      auto sn = solve_matrix_.supernode(k);
      auto sep = solve_matrix_.separator(k);

      solve_order_[idx]->BackwardSolveBlocked(sn, sep);

      // Push solution values to children.
      for (const auto& cop : info.children) {
        auto child_sep = solve_matrix_.separator(cop.child_block_index);
        for (const auto& off : cop.sn_offsets) {
          child_sep.middleRows(off.second, off.size) =
              sn.middleRows(off.first, off.size);
        }
        for (const auto& off : cop.sep_offsets) {
          child_sep.middleRows(off.second, off.size) =
              sep.middleRows(off.first, off.size);
        }
      }
    }

    if (in_original_order) {
      solve_matrix_.GatherInto(b);
    } else {
      solve_matrix_.GatherIntoElimOrder(b);
    }
    return;
  }

  // Fallback: recursive traversal (only when solve_matrix_ not initialized).
  ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_),
              [&](size_t i) {
                auto* root = roots_.at(i);
                root->ApplyInverseOfLeftFactor(b);
                root->ApplyInverseOfRightFactor(b);
              });
}


void T::ComputeSeparatorOffsets() {
  ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_),
              [&](size_t i) { roots_.at(i)->ComputeSeparatorOffsets(); });
}

void T::UpdateAssemblerData() {
  // Zero all subsystem storage before contributors write additively.
  START_TIMER(Memset)
  if (arena_memory_) {
    std::memset(arena_memory_.get(), 0, arena_bytes_);
  }
  END_TIMER
  ResetUpdateDataTimers();
  START_TIMER(AdapterUpdateData)
  ForEachTask(assembler_to_subsystem_adapter_.size(),
              EffectiveThreadCount(num_threads_), [&](size_t i) {
                assembler_to_subsystem_adapter_.at(i)->UpdateData();
              });
  END_TIMER
  PrintUpdateDataTimers();
}

void T::DoAssemble() {
  if (auto_update_assemblers_) {
    UpdateAssemblerData();
  }
  ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_),
              [&](size_t i) { roots_.at(i)->Assemble(); });
}

bool T::DoAssembleAndFactor() {
  if (auto_update_assemblers_) {
    START_TIMER(UpdateAssemblerData)
    UpdateAssemblerData();
    END_TIMER
  }
  std::atomic<bool> success(true);
  START_TIMER(TreeFactorization)
  ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_), [&](size_t i) {
    if (!success.load(std::memory_order_relaxed)) {
      return;
    }
    if (!roots_.at(i)->AssembleAndFactor()) {
      success.store(false, std::memory_order_relaxed);
    }
  });
  END_TIMER
  return success.load(std::memory_order_relaxed);
}

bool T::DoFactor() {
  std::atomic<bool> success(true);
  ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_), [&](size_t i) {
    if (!success.load(std::memory_order_relaxed)) {
      return;
    }
    if (!roots_.at(i)->Factor()) {
      success.store(false, std::memory_order_relaxed);
    }
  });
  return success.load(std::memory_order_relaxed);
}

void T::Finalize(const CliqueTree& clique_tree) {
  // Auto-create subsystems when none were provided.  Use DynamicSubsystem
  // which supports both positive-definite (LLT) and indefinite (RLDLT)
  // factorization, since the general tree path may encounter equality
  // constraints that produce indefinite KKT systems.
  if (subsystems_.empty()) {
    owned_subsystems_.clear();
    for (size_t i = 0; i < clique_tree.supernodes.size(); ++i) {
      auto ds = std::make_unique<DynamicSubsystem>();
      subsystems_.push_back(ds.get());
      owned_subsystems_.push_back(std::move(ds));
    }
  }
  CONEX_CHECK(clique_tree.supernodes.size() == subsystems_.size());
  CONEX_CHECK(clique_tree.separators.size() == subsystems_.size());
  int i = 0;
  for (auto& s : subsystems_) {
    s->SetSupernodes(clique_tree.supernodes.at(i));
    s->SetSeparators(clique_tree.separators.at(i));
    ++i;
  }
  AllocateArenaAndBind();
  for (auto& s : subsystems_) {
    s->Initialize();
  }
  reserved_solve_workspace_cols_ = 0;
  SetEliminationTree(clique_tree.node_to_parent);
  SetEliminationOrder(ComputePostOrdering());
  ComputeSeparatorOffsets();
  // Build flat post-order traversal for non-recursive solve.
  solve_order_.clear();
  solve_order_.reserve(subsystems_.size());
  std::function<void(KKTSubsystemBase*)> visit = [&](KKTSubsystemBase* node) {
    for (auto* child : node->children()) {
      visit(child);
    }
    solve_order_.push_back(node);
  };
  for (auto* root : roots_) {
    visit(root);
  }
  AllocateSolveArena();

  // Build lookup: elimination position -> subsystem that owns it as a
  // supernode.  Each variable is a supernode of exactly one subsystem.
  std::unordered_map<int, KKTSubsystemBase*> elim_pos_to_subsystem;
  for (auto* subsystem : subsystems_) {
    for (int sn : subsystem->supernodes()) {
      elim_pos_to_subsystem[sn] = subsystem;
    }
  }

  // Bind contributors to adapters.
  for (auto& adapter : assembler_to_subsystem_adapter_) {
    auto c = std::make_unique<SubmatrixContributor>(
        MakeContributorFromLookup(adapter->elimination_positions(),
                                  elim_pos_to_subsystem));
    c->set_type(adapter->contribution_type());
    c->PrecomputeLazyOrder(adapter->elimination_positions());
    adapter->BindContributor(std::move(c));
  }
}

void T::SetEliminationTree(const std::vector<int>& parent) {
  reserved_solve_workspace_cols_ = 0;
  roots_.clear();
  for (auto s : subsystems_) {
    s->Reset();
  }
  for (size_t i = 0; i < parent.size(); ++i) {
    if (parent[i] >= 0) {
      CONEX_DEMAND(parent[i] != static_cast<int>(i),
                   "Tree is malformed: node cannot be own parent.");
      subsystems_.at(parent[i])->AddChild(subsystems_.at(i));
    } else {
      roots_.push_back(subsystems_.at(i));
    }
  }
}

std::vector<int> T::ComputePostOrdering() const {
  // Post-order
  std::vector<int> variable_to_elimination_position(number_of_variables());
  int first = 0;
  for (auto r : roots_) {
    first = r->ComputePostOrdering(first, &variable_to_elimination_position);
  }
  return variable_to_elimination_position;
}

void T::SetFactorizationMode(bool left_looking) {
  for (auto s : subsystems_) {
    s->SetFactorizationMode(left_looking);
  }
}

void T::AllocateArenaAndBind() {
  constexpr size_t kAlign = EIGEN_MAX_ALIGN_BYTES;
  size_t total_bytes = 0;
  for (const auto* subsystem : subsystems_) {
    total_bytes += subsystem->RequiredArenaBytes() + (kAlign - 1);
  }
  if (total_bytes == 0) {
    arena_memory_.reset();
    arena_bytes_ = 0;
    return;
  }
  const size_t alloc_bytes = ((total_bytes + (kAlign - 1)) / kAlign) * kAlign;
  void* raw_ptr = nullptr;
  if (posix_memalign(&raw_ptr, kAlign, alloc_bytes) != 0) {
    throw std::bad_alloc();
  }
  arena_memory_.reset(raw_ptr);
  arena_bytes_ = alloc_bytes;
  std::memset(raw_ptr, 0, alloc_bytes);

  std::uintptr_t cursor = reinterpret_cast<std::uintptr_t>(arena_memory_.get());
  for (auto* subsystem : subsystems_) {
    const size_t bytes = subsystem->RequiredArenaBytes();
    if (bytes == 0) {
      continue;
    }
    cursor =
        (cursor + (kAlign - 1)) & ~(static_cast<std::uintptr_t>(kAlign - 1));
    subsystem->BindArenaMemory(reinterpret_cast<double*>(cursor), bytes);
    cursor += bytes;
  }
}

void T::AllocateSolveArena() {
  solve_matrix_.SetPartition(subsystems_, cached_num_vars_, cached_perm_inv_);

  // Build subsystem pointer -> index map (local, used for setup only).
  const int num_subsystems = static_cast<int>(subsystems_.size());
  std::unordered_map<const KKTSubsystemBase*, int> subsystem_index;
  for (int k = 0; k < num_subsystems; ++k) {
    subsystem_index[subsystems_[k]] = k;
  }

  // Build precomputed scatter info for blocked solve hot path.
  const int num_solve = static_cast<int>(solve_order_.size());
  solve_scatter_info_.resize(num_solve);
  for (int i = 0; i < num_solve; ++i) {
    auto* node = solve_order_[i];
    int k = subsystem_index.at(node);
    solve_scatter_info_[i].block_index = k;
    solve_scatter_info_[i].children.clear();
    for (auto* child : node->children()) {
      int ck = subsystem_index.at(child);
      ChildScatterOp op;
      op.child_block_index = ck;
      op.sn_offsets = node->local_supernode_to_source_separator(child);
      op.sep_offsets = node->local_separator_to_source_separator(child);
      solve_scatter_info_[i].children.push_back(std::move(op));
    }
  }
}
void SubmatrixContributor::set_type(ContributionType type) {
  if (type == ContributionType::kIndefinite) {
    subsystem_->MarkIndefinite();
  }
}

void SubmatrixContributor::PrecomputeLazyOrder(
    const std::vector<int>& elim_positions) {
  const int n = static_cast<int>(elim_positions.size());

  // Classify each index: supernode or separator, with local offset.
  struct VarInfo {
    bool is_sn;
    int local;
  };
  std::vector<VarInfo> info(n);
  std::unordered_map<int, int> sep_to_local;
  sep_to_local.reserve(sep_indices_.size());
  for (int i = 0; i < static_cast<int>(sep_indices_.size()); ++i) {
    sep_to_local[sep_indices_[i]] = i;
  }
  for (int i = 0; i < n; ++i) {
    int ep = elim_positions[i];
    if (ep >= sn_start_ && ep < sn_start_ + sn_count_) {
      info[i] = {true, ep - sn_start_};
    } else {
      auto it = sep_to_local.find(ep);
      CONEX_DEMAND(
          it != sep_to_local.end(),
          "elim_positions entry not in contributor's sparsity pattern.");
      info[i] = {false, it->second};
    }
  }

  // Compute optimal permutation: supernodes sorted by local index first,
  // then separators sorted by local index.  This maximizes contiguous runs.
  cached_perm_.resize(n);
  for (int i = 0; i < n; ++i) cached_perm_[i] = i;
  std::sort(cached_perm_.begin(), cached_perm_.end(), [&](int a, int b) {
    if (info[a].is_sn != info[b].is_sn) return info[a].is_sn > info[b].is_sn;
    return info[a].local < info[b].local;
  });

  // Reorder info according to perm to find contiguous runs.
  std::vector<VarInfo> perm_info(n);
  for (int i = 0; i < n; ++i) {
    perm_info[i] = info[cached_perm_[i]];
  }

  // Find maximal contiguous runs in the permuted order.
  cached_runs_.clear();
  cached_runs_.reserve(n);
  int i = 0;
  while (i < n) {
    Run run{i, 1, perm_info[i].is_sn, perm_info[i].local};
    while (i + run.length < n &&
           perm_info[i + run.length].is_sn == run.is_sn &&
           perm_info[i + run.length].local == run.local_start + run.length) {
      run.length++;
    }
    cached_runs_.push_back(run);
    i += run.length;
  }

  lazy_order_cached_ = true;
}

void SubmatrixContributor::WriteSymmetric(
    const Eigen::MatrixXd& Q, const std::vector<int>& elim_positions) {
  const int n = static_cast<int>(elim_positions.size());
  CONEX_DEMAND(Q.rows() == n && Q.cols() == n,
               "Q dimensions must match elim_positions size.");

  auto sn_sub = supernode_submatrix();
  auto sep_r = separator_rows();
  auto sep_sc = separator_schur_complement();

  // Classify each Q index: is it supernode or separator, and what local index?
  struct VarInfo {
    bool is_sn;
    int local;
  };
  std::vector<VarInfo> info(n);

  std::unordered_map<int, int> sep_to_local;
  sep_to_local.reserve(sep_indices_.size());
  for (int i = 0; i < static_cast<int>(sep_indices_.size()); ++i) {
    sep_to_local[sep_indices_[i]] = i;
  }
  for (int i = 0; i < n; ++i) {
    int ep = elim_positions[i];
    if (ep >= sn_start_ && ep < sn_start_ + sn_count_) {
      info[i] = {true, ep - sn_start_};
    } else {
      auto it = sep_to_local.find(ep);
      CONEX_DEMAND(it != sep_to_local.end(),
                   "elim_positions entry not in contributor's sparsity pattern.");
      info[i] = {false, it->second};
    }
  }

  // Find maximal contiguous runs: consecutive Q indices that map to
  // consecutive local indices in the same block type.
  struct Run {
    int q_start;
    int length;
    bool is_sn;
    int local_start;
  };
  std::vector<Run> runs;
  runs.reserve(n);
  int i = 0;
  while (i < n) {
    Run run{i, 1, info[i].is_sn, info[i].local};
    while (i + run.length < n &&
           info[i + run.length].is_sn == run.is_sn &&
           info[i + run.length].local == run.local_start + run.length) {
      run.length++;
    }
    runs.push_back(run);
    i += run.length;
  }

  // Write blocks.  Only the lower triangle of Q is written, so we iterate
  // col_run <= row_run (in Q index order).  Runs partition [0,n) into
  // consecutive segments, so row_run.q_start >= col_run.q_start + col_run.length
  // when ri > ci, guaranteeing the Q sub-block is below the diagonal.
  const int nr = static_cast<int>(runs.size());
  for (int ci = 0; ci < nr; ++ci) {
    const auto& cr = runs[ci];
    for (int ri = ci; ri < nr; ++ri) {
      const auto& rr = runs[ri];
      auto Qblk = Q.block(rr.q_start, cr.q_start, rr.length, cr.length);

      if (ri == ci) {
        // Diagonal block — only lower triangle.
        if (rr.is_sn) {
          sn_sub.block(rr.local_start, cr.local_start, rr.length, cr.length)
              .triangularView<Eigen::Lower>() += Qblk;
        } else {
          sep_sc.block(rr.local_start, cr.local_start, rr.length, cr.length)
              .triangularView<Eigen::Lower>() += Qblk;
        }
      } else if (rr.is_sn && cr.is_sn) {
        // Both supernode.  Storage holds lower triangle only.
        if (rr.local_start > cr.local_start) {
          sn_sub.block(rr.local_start, cr.local_start, rr.length, cr.length)
              .noalias() += Qblk;
        } else {
          sn_sub.block(cr.local_start, rr.local_start, cr.length, rr.length)
              .noalias() += Qblk.transpose();
        }
      } else if (!rr.is_sn && cr.is_sn) {
        // row=separator, col=supernode → separator_rows.
        sep_r.block(rr.local_start, cr.local_start, rr.length, cr.length)
            .noalias() += Qblk;
      } else if (rr.is_sn && !cr.is_sn) {
        // row=supernode, col=separator → transpose into separator_rows.
        sep_r.block(cr.local_start, rr.local_start, cr.length, rr.length)
            .noalias() += Qblk.transpose();
      } else {
        // Both separator.  Storage holds lower triangle only.
        if (rr.local_start > cr.local_start) {
          sep_sc.block(rr.local_start, cr.local_start, rr.length, cr.length)
              .noalias() += Qblk;
        } else {
          sep_sc.block(cr.local_start, rr.local_start, cr.length, rr.length)
              .noalias() += Qblk.transpose();
        }
      }
    }
  }
}

SubmatrixContributor T::MakeContributor(
    const std::vector<int>& elim_indices) const {
  if (elim_indices.empty()) {
    return {};
  }

  // Build a set of requested indices for fast lookup.
  std::vector<int> sorted_indices = elim_indices;
  std::sort(sorted_indices.begin(), sorted_indices.end());

  // Find the smallest subsystem whose supernodes ∪ separators contain all
  // indices (tightest match).
  KKTSubsystemBase* match = nullptr;
  size_t match_size = std::numeric_limits<size_t>::max();
  for (auto* subsystem : subsystems_) {
    const auto& sn = subsystem->supernodes();
    const auto& sep = subsystem->separators();

    // Supernodes/separators may not be sorted (after ReorderSupernodes),
    // so use a set for containment checks.
    std::set<int> bag(sn.begin(), sn.end());
    bag.insert(sep.begin(), sep.end());

    bool all_found = true;
    for (int idx : sorted_indices) {
      if (bag.find(idx) == bag.end()) {
        all_found = false;
        break;
      }
    }
    if (all_found) {
      size_t total = sn.size() + sep.size();
      if (total < match_size) {
        match = subsystem;
        match_size = total;
      }
    }
  }

  if (!match) {
    throw std::runtime_error(
        "MakeContributor: requested elimination indices are not contained "
        "in any single subsystem's sparsity pattern.");
  }

  SubmatrixContributor contrib;
  contrib.subsystem_ = match;
  const auto& sn = match->supernodes();
  contrib.sn_start_ = sn.empty() ? 0 : sn.front();
  contrib.sn_count_ = static_cast<int>(sn.size());
  contrib.sep_indices_ = match->separators();
  return contrib;
}

SubmatrixContributor T::MakeContributorFromLookup(
    const std::vector<int>& elim_indices,
    const std::unordered_map<int, KKTSubsystemBase*>& elim_pos_to_subsystem)
    const {
  if (elim_indices.empty()) {
    return {};
  }

  // Each variable maps to the subsystem owning it as a supernode.
  // Collect candidate subsystems via the lookup, then find the smallest
  // whose supernodes ∪ separators contain all of the contributor's variables.
  std::set<KKTSubsystemBase*> candidates;
  for (int idx : elim_indices) {
    auto it = elim_pos_to_subsystem.find(idx);
    if (it != elim_pos_to_subsystem.end()) {
      candidates.insert(it->second);
    }
  }

  KKTSubsystemBase* match = nullptr;
  size_t match_size = std::numeric_limits<size_t>::max();
  for (auto* subsystem : candidates) {
    const auto& sn = subsystem->supernodes();
    const auto& sep = subsystem->separators();
    size_t total = sn.size() + sep.size();
    if (total >= match_size) {
      continue;
    }
    // Check containment: all elim_indices must be in supernodes ∪ separators.
    // Supernodes are contiguous; use range check. Separators: use sorted search.
    bool all_found = true;
    int sn_lo = sn.empty() ? 0 : sn.front();
    int sn_hi = sn.empty() ? -1 : sn.back();
    for (int idx : elim_indices) {
      if (idx >= sn_lo && idx <= sn_hi) {
        continue;  // In supernode range.
      }
      if (!std::binary_search(sep.begin(), sep.end(), idx)) {
        all_found = false;
        break;
      }
    }
    if (all_found) {
      match = subsystem;
      match_size = total;
    }
  }

  if (!match) {
    throw std::runtime_error(
        "MakeContributorFromLookup: no subsystem found for elimination "
        "indices.");
  }

  SubmatrixContributor contrib;
  contrib.subsystem_ = match;
  const auto& sn = match->supernodes();
  contrib.sn_start_ = sn.empty() ? 0 : sn.front();
  contrib.sn_count_ = static_cast<int>(sn.size());
  contrib.sep_indices_ = match->separators();
  return contrib;
}

void T::AllocateWorkspaceArena(int rhs_cols) {
  constexpr size_t kAlign = EIGEN_MAX_ALIGN_BYTES;
  auto align = [kAlign](size_t v) {
    return ((v + kAlign - 1) / kAlign) * kAlign;
  };

  // Compute total workspace bytes for all subsystems.
  size_t total = 0;
  for (auto* subsystem : subsystems_) {
    const auto& sn = subsystem->supernodes();
    int sn_rows = 0;
    if (!sn.empty()) {
      sn_rows = sn.back() - sn.front() + 1;
    }
    int sep_rows = static_cast<int>(subsystem->separators().size());
    // ws1: sn_rows × rhs_cols, ws2: sn_rows × rhs_cols, ws3: sep_rows × rhs_cols
    total += align(sn_rows * rhs_cols * sizeof(double));
    total += align(sn_rows * rhs_cols * sizeof(double));
    total += align(sep_rows * rhs_cols * sizeof(double));
  }

  if (total > workspace_arena_bytes_) {
    void* raw_ptr = nullptr;
    if (posix_memalign(&raw_ptr, kAlign, total + kAlign) != 0) {
      throw std::bad_alloc();
    }
    workspace_arena_.reset(raw_ptr);
    workspace_arena_bytes_ = total + kAlign;
  }
  workspace_arena_cols_ = rhs_cols;

  // Distribute pointers to subsystems.
  char* base = static_cast<char*>(workspace_arena_.get());
  size_t cursor = 0;
  for (auto* subsystem : subsystems_) {
    const auto& sn = subsystem->supernodes();
    int sn_rows = 0;
    if (!sn.empty()) {
      sn_rows = sn.back() - sn.front() + 1;
    }
    int sep_rows = static_cast<int>(subsystem->separators().size());

    cursor = align(cursor);
    double* ws1 = reinterpret_cast<double*>(base + cursor);
    cursor += sn_rows * rhs_cols * sizeof(double);

    cursor = align(cursor);
    double* ws2 = reinterpret_cast<double*>(base + cursor);
    cursor += sn_rows * rhs_cols * sizeof(double);

    cursor = align(cursor);
    double* ws3 = reinterpret_cast<double*>(base + cursor);
    cursor += sep_rows * rhs_cols * sizeof(double);

    subsystem->BindSolveWorkspace(ws1, sn_rows, rhs_cols,
                                  ws2, sn_rows, rhs_cols,
                                  ws3, sep_rows, rhs_cols);
  }
}

void T::ReserveSolveWorkspace(int rhs_cols) {
  CONEX_DEMAND(rhs_cols >= 0, "rhs_cols must be nonnegative.");
  if (rhs_cols <= reserved_solve_workspace_cols_) {
    return;
  }
  AllocateWorkspaceArena(rhs_cols);
  reserved_solve_workspace_cols_ = rhs_cols;
}

int T::number_of_variables() const {
  if (cached_num_vars_ > 0) return cached_num_vars_;
  int max = 0;
  for (auto s : subsystems_) {
    const auto& sn = s->supernodes();
    if (sn.size() > 0) {
      double max_s = *std::max_element(sn.begin(), sn.end());
      if (max_s > max) {
        max = max_s;
      }
    }
  }
  return max + 1;
}

Eigen::MatrixXd T::DoKKTMatrix(bool permute_to_elimination_order) const {
  int num_vars = number_of_variables();
  Eigen::MatrixXd M(num_vars, num_vars);
  M.setZero();
  for (auto root : roots_) {
    root->MakeKKTMatrix(&M);
    M = M.selfadjointView<Eigen::Lower>();
  }
  if (permute_to_elimination_order) {
    return M;
  } else {
    CONEX_CHECK(static_cast<int>(variable_to_elimination_position_.size()) ==
                number_of_variables());
    Eigen::PermutationMatrix<-1> P(number_of_variables());
    P.indices() = Eigen::Map<const Eigen::VectorXi>(
        variable_to_elimination_position_.data(), number_of_variables());
    return P.transpose() * M * P;
  }
}

void T::push_back(std::unique_ptr<KKTAssemblerToSubsystemAdapter>&& system) {
  assembler_to_subsystem_adapter_.emplace_back(std::move(system));
}

}  // namespace conex
