#include "conex/kkt_tree_solver.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <functional>
#include <thread>
#include <unordered_map>  // used in AllocateSolveArena setup

#include "conex/tree_utils.h"

namespace conex {

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
  // Cache the max requested width to avoid repeating full-tree recursion.
  if (b.cols() > reserved_solve_workspace_cols_) {
    ForEachTask(
        roots_.size(), EffectiveThreadCount(num_threads_),
        [&](size_t i) { roots_.at(i)->ReserveSolveWorkspace(b.cols()); });
    reserved_solve_workspace_cols_ = b.cols();
  }

  if (b.cols() == 1 && !solve_blocks_.empty() && in_original_order) {
    // Block-partitioned path: no global vector, direct block-to-block scatter.
    // Replaces both permutation and solve — var_to_sn_ptr_ maps original
    // variables directly into supernode blocks.
    std::memset(solve_arena_.get(), 0, solve_arena_bytes_);

    // Scatter b into supernode blocks via var_to_sn_ptr_ (replaces permutation).
    for (int v = 0; v < n; ++v) {
      *var_to_sn_ptr_[v] = b(v, 0);
    }

    // Forward pass (post-order).
    const int num_solve = static_cast<int>(solve_order_.size());
    for (int idx = 0; idx < num_solve; ++idx) {
      const auto& info = solve_scatter_info_[idx];
      auto& blk = solve_blocks_[info.block_index];

      // Scatter children's separator outputs into this node's blocks.
      for (const auto& cop : info.children) {
        const auto& child_blk = solve_blocks_[cop.child_block_index];
        for (const auto& off : cop.sn_offsets) {
          for (int i = 0; i < off.size; ++i) {
            blk.supernode_data[off.first + i] -=
                child_blk.separator_data[off.second + i];
          }
        }
        for (const auto& off : cop.sep_offsets) {
          for (int i = 0; i < off.size; ++i) {
            blk.separator_data[off.first + i] +=
                child_blk.separator_data[off.second + i];
          }
        }
      }

      // trsv + dgemv
      solve_order_[idx]->ForwardSolveBlocked(
          blk.supernode_data, blk.supernode_size,
          blk.separator_data, blk.separator_size);
    }

    // Backward pass (reverse post-order).
    for (int idx = num_solve - 1; idx >= 0; --idx) {
      const auto& info = solve_scatter_info_[idx];
      auto& blk = solve_blocks_[info.block_index];

      // trsv (with backward scatter from separator)
      solve_order_[idx]->BackwardSolveBlocked(
          blk.supernode_data, blk.supernode_size,
          blk.separator_data, blk.separator_size);

      // Push solution values to children.
      for (const auto& cop : info.children) {
        auto& child_blk = solve_blocks_[cop.child_block_index];
        for (const auto& off : cop.sn_offsets) {
          for (int i = 0; i < off.size; ++i) {
            child_blk.separator_data[off.second + i] =
                blk.supernode_data[off.first + i];
          }
        }
        for (const auto& off : cop.sep_offsets) {
          for (int i = 0; i < off.size; ++i) {
            child_blk.separator_data[off.second + i] =
                blk.separator_data[off.first + i];
          }
        }
      }
    }

    // Gather from blocks back into b via var_to_sn_ptr_ (replaces inv perm).
    for (int v = 0; v < n; ++v) {
      b(v, 0) = *var_to_sn_ptr_[v];
    }
    return;
  }

  if (in_original_order) {
    // Apply forward permutation using cached vectors and scratch buffer.
    if (solve_temp_.rows() < n || solve_temp_.cols() < b.cols()) {
      solve_temp_.resize(n, b.cols());
    }
    for (int i = 0; i < n; ++i) {
      solve_temp_.row(cached_perm_(i)) = b.row(i);
    }
    b = solve_temp_;
  }

  if (!solve_order_.empty()) {
    // Flat traversal (avoids recursive function calls).
    // Forward pass: post-order (leaves first).
    for (auto* node : solve_order_) {
      node->ForwardSolveLocal(b);
    }
    // Backward pass: reverse post-order (roots first).
    for (int i = static_cast<int>(solve_order_.size()) - 1; i >= 0; --i) {
      solve_order_[i]->BackwardSolveLocal(b);
    }
  } else {
    // Fallback: recursive traversal.
    ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_),
                [&](size_t i) {
                  auto* root = roots_.at(i);
                  root->ApplyInverseOfLeftFactor(b);
                  root->ApplyInverseOfRightFactor(b);
                });
  }

  if (in_original_order) {
    // Apply inverse permutation using cached vectors and scratch buffer.
    for (int i = 0; i < n; ++i) {
      solve_temp_.row(cached_perm_inv_(i)) = b.row(i);
    }
    b = solve_temp_;
  }
}


void T::ComputeSeparatorOffsets() {
  ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_),
              [&](size_t i) { roots_.at(i)->ComputeSeparatorOffsets(); });
}

void T::UpdateAssemblerData() {
  ForEachTask(assembler_to_subsystem_adapter_.size(),
              EffectiveThreadCount(num_threads_), [&](size_t i) {
                assembler_to_subsystem_adapter_.at(i)->UpdateData();
              });
}

void T::DoAssemble() {
  if (auto_update_assemblers_) {
    UpdateAssemblerData();
  }
  ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_),
              [&](size_t i) { roots_.at(i)->Assemble(); });
}

bool T::DoAssembleAndFactor() {
  std::atomic<bool> success(true);
  ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_), [&](size_t i) {
    if (!success.load(std::memory_order_relaxed)) {
      return;
    }
    if (!roots_.at(i)->AssembleAndFactor()) {
      success.store(false, std::memory_order_relaxed);
    }
  });
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
}

void T::Finalize(const Options& options) {
  throw std::runtime_error("Obsolete");
  // RootedTree tree(subsystems_.size());
  // SymmetricMatrix<vector<int>> intersections(subsystems_.size());
  // PickCliqueOrderHelper(subsystems_, options.root_node,
  //                       options.validate_leaf_nodes, &intersections, &tree);
  // Finalize(tree.parent, options.check_for_zero_pivots);
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

void T::FinalizeHelper(const std::vector<int>& parent) {
  throw std::runtime_error("Obsolete");
  /*
  CONEX_DEMAND(parent.size() == subsystems_.size(),
               "Size of parent vector must equal number of subsystems.");

  SetEliminationTree(parent);
  // Set supernodes from parent.
  for (size_t i = 0; i < parent.size(); ++i) {
    if (parent[i] >= 0) {
      std::vector<int> v1 = subsystems_.at(parent[i])->shared_variables();
      std::vector<int> v2 = subsystems_.at(i)->shared_variables();
      std::sort(v1.begin(), v1.end());
      std::sort(v2.begin(), v2.end());
      std::vector<int> separators;
      std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                            std::back_inserter(separators));
      subsystems_.at(i)->SetSeparators(separators);

      std::vector<int> supernodes;
      std::set_difference(v2.begin(), v2.end(), separators.begin(),
                          separators.end(), std::back_inserter(supernodes));
      subsystems_.at(i)->SetSupernodes(supernodes);
    } else {
      std::vector<int> v2 = subsystems_.at(i)->shared_variables();
      std::sort(v2.begin(), v2.end());
      subsystems_.at(i)->SetSupernodes(v2);
      subsystems_.at(i)->SetSeparators({});
    }
  }
  FillIn(parent, number_of_variables(), &subsystems_);

  // Post-order
  variable_to_elimination_position_.resize(number_of_variables());
  int first = 0;
  for (auto r : roots_) {
    first = r->ComputePostOrdering(first, &variable_to_elimination_position_);
  }
  for (auto s : subsystems_) {
    s->SetVariableOrdering(variable_to_elimination_position_);
  }*/
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
  constexpr size_t kAlign = EIGEN_MAX_ALIGN_BYTES;
  const int n = cached_num_vars_;
  const int num_subsystems = static_cast<int>(subsystems_.size());

  // Compute total arena size: for each subsystem, we need supernode_size +
  // separator_size doubles, each block SIMD-aligned.
  size_t total_doubles = 0;
  for (int k = 0; k < num_subsystems; ++k) {
    const auto* sub = subsystems_[k];
    const int sn_size = static_cast<int>(sub->supernodes().size());
    const int sep_size = static_cast<int>(sub->separators().size());
    // Each block needs alignment padding.
    size_t sn_bytes = sn_size * sizeof(double);
    size_t sep_bytes = sep_size * sizeof(double);
    total_doubles += (sn_bytes + kAlign - 1) / sizeof(double);
    total_doubles += (sep_bytes + kAlign - 1) / sizeof(double);
  }

  // Compute byte-level layout with alignment.
  size_t arena_bytes = 0;
  // First pass: compute total size.
  struct BlockLayout {
    size_t sn_offset;
    size_t sep_offset;
  };
  std::vector<BlockLayout> layouts(num_subsystems);
  size_t cursor = 0;
  for (int k = 0; k < num_subsystems; ++k) {
    const auto* sub = subsystems_[k];
    const int sn_size = static_cast<int>(sub->supernodes().size());
    const int sep_size = static_cast<int>(sub->separators().size());

    // Supernode block.
    cursor = ((cursor + kAlign - 1) / kAlign) * kAlign;
    layouts[k].sn_offset = cursor;
    cursor += sn_size * sizeof(double);

    // Separator block.
    cursor = ((cursor + kAlign - 1) / kAlign) * kAlign;
    layouts[k].sep_offset = cursor;
    cursor += sep_size * sizeof(double);
  }
  arena_bytes = cursor;

  // Build subsystem pointer -> index map (local, used for setup only).
  std::unordered_map<const KKTSubsystemBase*, int> subsystem_index;
  for (int k = 0; k < num_subsystems; ++k) {
    subsystem_index[subsystems_[k]] = k;
  }

  if (arena_bytes == 0) {
    solve_blocks_.clear();
    var_to_sn_ptr_.clear();
    solve_arena_.reset();
    solve_arena_bytes_ = 0;
    return;
  }

  // Allocate aligned.
  const size_t alloc_bytes = ((arena_bytes + kAlign - 1) / kAlign) * kAlign;
  void* raw_ptr = nullptr;
  if (posix_memalign(&raw_ptr, kAlign, alloc_bytes) != 0) {
    throw std::bad_alloc();
  }
  solve_arena_.reset(raw_ptr);
  solve_arena_bytes_ = alloc_bytes;

  // Build solve_blocks_.
  char* base = static_cast<char*>(raw_ptr);
  solve_blocks_.resize(num_subsystems);
  for (int k = 0; k < num_subsystems; ++k) {
    const auto* sub = subsystems_[k];
    solve_blocks_[k].supernode_size =
        static_cast<int>(sub->supernodes().size());
    solve_blocks_[k].separator_size =
        static_cast<int>(sub->separators().size());
    solve_blocks_[k].supernode_data =
        reinterpret_cast<double*>(base + layouts[k].sn_offset);
    solve_blocks_[k].separator_data =
        reinterpret_cast<double*>(base + layouts[k].sep_offset);
  }

  // Build var_to_sn_ptr_: for each original variable v, a pointer into
  // the owning node's supernode block.
  var_to_sn_ptr_.resize(n);
  for (int k = 0; k < num_subsystems; ++k) {
    const auto& sn = subsystems_[k]->supernodes();
    double* sn_data = solve_blocks_[k].supernode_data;
    for (int i = 0; i < static_cast<int>(sn.size()); ++i) {
      // sn[i] is the elimination-order index. We need the original variable.
      // cached_perm_inv_ maps elimination position -> original variable.
      int orig_var = cached_perm_inv_(sn[i]);
      var_to_sn_ptr_[orig_var] = sn_data + i;
    }
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
void T::ReserveSolveWorkspace(int rhs_cols) {
  CONEX_DEMAND(rhs_cols >= 0, "rhs_cols must be nonnegative.");
  if (rhs_cols <= reserved_solve_workspace_cols_) {
    return;
  }
  ForEachTask(roots_.size(), EffectiveThreadCount(num_threads_),
              [&](size_t i) { roots_.at(i)->ReserveSolveWorkspace(rhs_cols); });
  reserved_solve_workspace_cols_ = rhs_cols;
}

bool T::CheckForZeroPivot(const std::vector<int>& parent,
                          std::vector<int>* index_of_zero_pivot) {
  index_of_zero_pivot->clear();
  FinalizeHelper(parent);
  Assemble();
  int i = 0;
  for (auto r : subsystems_) {
    if (r->supernodes().size() > 0) {
      Eigen::MatrixXd T =
          r->supernode_submatrix().selfadjointView<Eigen::Lower>();
      T = T.transpose() * T;
      bool zero_pivot = T.colwise().sum().minCoeff() == 0;
      if (zero_pivot) {
        index_of_zero_pivot->push_back(i);
      }
    }
    i++;
  }
  return index_of_zero_pivot->size() > 0;
}

void T::Finalize(const std::vector<int>& parent, bool check_for_zero_pivot) {
  CONEX_CHECK(subsystems_.size() == parent.size());
  FinalizeHelper(parent);
  if (check_for_zero_pivot) {
    std::vector<int> index_of_zero_pivot;
    if (CheckForZeroPivot(parent, &index_of_zero_pivot)) {
      throw std::runtime_error("Invalid tree: zero pivot detected.");
    }
  }
}

int T::number_of_variables() const {
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
  DUMP(variable_to_elimination_position_);
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

Eigen::SparseMatrix<double> T::MakeSparseKKTMatrix(
    bool permute_to_elimination_order) const {
  std::vector<Eigen::Triplet<double>> triplets;
  for (auto s : subsystems_) {
    s->AddSparseMatrixTriplets(&triplets);
  }
  Eigen::SparseMatrix<double> matrix(number_of_variables(),
                                     number_of_variables());
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  Eigen::SparseMatrix<double> matrix_sym =
      matrix.selfadjointView<Eigen::Lower>();
  if (permute_to_elimination_order) {
    return matrix;
  } else {
    CONEX_CHECK(static_cast<int>(variable_to_elimination_position_.size()) ==
                number_of_variables());
    Eigen::PermutationMatrix<-1> P(number_of_variables());
    P.indices() = Eigen::Map<const Eigen::VectorXi>(
        variable_to_elimination_position_.data(), number_of_variables());
    return P.transpose() * matrix_sym * P;
  }
}

void T::AddSubsystem(KKTSubsystemType* system) {
  CONEX_CHECK(system != nullptr);
  system->SetNumThreads(
      SubsystemThreadCount(num_threads_, parallelize_roots_only_));
  subsystems_.push_back(system);
  reserved_solve_workspace_cols_ = 0;
}

void T::push_back(std::unique_ptr<KKTAssemblerToSubsystemAdapter>&& system) {
  assembler_to_subsystem_adapter_.emplace_back(std::move(system));
}

}  // namespace conex
