#include "conex/gpu_tree_solver/gpu_tree_solver.h"

#include <algorithm>
#include <map>
#include <numeric>
#include <stdexcept>

#include <cublas_v2.h>
#include <cuda_runtime.h>
#include <cusolverDn.h>

namespace conex {

namespace {
void Check(cudaError_t e, const char* m) {
  if (e != cudaSuccess)
    throw std::runtime_error(std::string(m) + ": " + cudaGetErrorString(e));
}
void Check(cusolverStatus_t s, const char* m) {
  if (s != CUSOLVER_STATUS_SUCCESS)
    throw std::runtime_error(std::string(m) + ": cuSOLVER error " +
                             std::to_string(static_cast<int>(s)));
}
void Check(cublasStatus_t s, const char* m) {
  if (s != CUBLAS_STATUS_SUCCESS)
    throw std::runtime_error(std::string(m) + ": cuBLAS error " +
                             std::to_string(static_cast<int>(s)));
}
}  // namespace

// --------------------------------------------------------------------------
// Construction / destruction
// --------------------------------------------------------------------------

GpuTreeSolver::GpuTreeSolver() {
  Check(cudaStreamCreate(&stream_), "cudaStreamCreate");
  Check(cusolverDnCreate(&cusolver_), "cusolverDnCreate");
  Check(cusolverDnSetStream(cusolver_, stream_), "cusolverDnSetStream");
  Check(cublasCreate(&cublas_), "cublasCreate");
  Check(cublasSetStream(cublas_, stream_), "cublasSetStream");
}

GpuTreeSolver::~GpuTreeSolver() {
  if (d_rhs_) cudaFree(d_rhs_);
  if (d_info_) cudaFree(d_info_);
  if (d_scatter_ops_) cudaFree(d_scatter_ops_);
  if (d_sep_indices_) cudaFree(d_sep_indices_);
  if (d_gather_buf_) cudaFree(d_gather_buf_);
  if (d_potrf_work_) cudaFree(d_potrf_work_);
  if (d_batch_ptrs_) cudaFree(d_batch_ptrs_);
  if (d_batch_sep_offsets_) cudaFree(d_batch_sep_offsets_);
  if (cublas_) cublasDestroy(cublas_);
  if (cusolver_) cusolverDnDestroy(cusolver_);
  if (stream_) cudaStreamDestroy(stream_);
}

GpuTreeSolver::GpuTreeSolver(GpuTreeSolver&& o) noexcept
    : clique_tree_(std::move(o.clique_tree_)),
      num_vars_(o.num_vars_),
      rhs_cols_(o.rhs_cols_),
      perm_(std::move(o.perm_)),
      perm_inv_(std::move(o.perm_inv_)),
      descriptors_(std::move(o.descriptors_)),
      levels_(std::move(o.levels_)),
      arena_(std::move(o.arena_)),
      d_rhs_(o.d_rhs_),
      d_info_(o.d_info_),
      d_scatter_ops_(o.d_scatter_ops_),
      scatter_op_offsets_(std::move(o.scatter_op_offsets_)),
      scatter_op_counts_(std::move(o.scatter_op_counts_)),
      host_data_(std::move(o.host_data_)),
      d_sep_indices_(o.d_sep_indices_),
      sep_indices_offsets_(std::move(o.sep_indices_offsets_)),
      d_gather_buf_(o.d_gather_buf_),
      max_sep_size_(o.max_sep_size_),
      d_potrf_work_(o.d_potrf_work_),
      potrf_work_size_(o.potrf_work_size_),
      level_groups_(std::move(o.level_groups_)),
      d_batch_ptrs_(o.d_batch_ptrs_),
      d_batch_sep_offsets_(o.d_batch_sep_offsets_),
      max_batch_size_(o.max_batch_size_),
      max_batch_gather_(o.max_batch_gather_),
      cusolver_(o.cusolver_),
      cublas_(o.cublas_),
      stream_(o.stream_),
      host_scatter_ops_(std::move(o.host_scatter_ops_)) {
  o.d_rhs_ = nullptr;
  o.d_info_ = nullptr;
  o.d_scatter_ops_ = nullptr;
  o.d_sep_indices_ = nullptr;
  o.d_gather_buf_ = nullptr;
  o.d_potrf_work_ = nullptr;
  o.d_batch_ptrs_ = nullptr;
  o.d_batch_sep_offsets_ = nullptr;
  o.cusolver_ = nullptr;
  o.cublas_ = nullptr;
  o.stream_ = nullptr;
}

GpuTreeSolver& GpuTreeSolver::operator=(GpuTreeSolver&& o) noexcept {
  if (this != &o) {
    this->~GpuTreeSolver();
    new (this) GpuTreeSolver(std::move(o));
  }
  return *this;
}

// --------------------------------------------------------------------------
// FinalizeStructure: symbolic setup
// --------------------------------------------------------------------------

void GpuTreeSolver::FinalizeStructure(const CliqueTree& clique_tree, int rhs_cols) {
  clique_tree_ = clique_tree;
  rhs_cols_ = rhs_cols;
  const int num_cliques = static_cast<int>(clique_tree_.supernodes.size());

  // --- Compute elimination ordering (post-order) ---
  int total_vars = 0;
  for (const auto& sn : clique_tree_.supernodes)
    for (int v : sn) total_vars = std::max(total_vars, v + 1);
  num_vars_ = total_vars;

  perm_.resize(num_vars_);
  perm_.setConstant(-1);
  perm_inv_.resize(num_vars_);
  perm_inv_.setConstant(-1);

  int epos = 0;
  for (int ci : clique_tree_.post_order_position_to_clique) {
    for (int v : clique_tree_.supernodes[ci]) {
      if (v >= 0 && v < num_vars_) {
        perm_(v) = epos;
        perm_inv_(epos) = v;
        epos++;
      }
    }
  }
  if (epos != num_vars_) {
    throw std::runtime_error(
        "Elimination ordering incomplete: " + std::to_string(epos) +
        " of " + std::to_string(num_vars_) + " variables assigned.");
  }

  // --- Build descriptors ---
  descriptors_.resize(num_cliques);
  std::vector<std::vector<int>> children(num_cliques);
  for (int i = 0; i < num_cliques; ++i) {
    int p = clique_tree_.node_to_parent[i];
    if (p >= 0 && p != i) children[p].push_back(i);
  }

  for (int i = 0; i < num_cliques; ++i) {
    auto& d = descriptors_[i];
    d.sn_size = static_cast<int>(clique_tree_.supernodes[i].size());
    d.sep_size = static_cast<int>(clique_tree_.separators[i].size());
    d.total_size = d.sn_size + d.sep_size;
    d.parent_index = clique_tree_.node_to_parent[i];
    if (d.parent_index == i) d.parent_index = -1;  // root
  }

  // --- Level assignment (BFS from leaves) ---
  std::vector<int> level(num_cliques, 0);
  // Post-order guarantees children come before parents.
  for (int ci : clique_tree_.post_order_position_to_clique) {
    for (int ch : children[ci]) {
      level[ci] = std::max(level[ci], level[ch] + 1);
    }
  }

  int max_level = *std::max_element(level.begin(), level.end());
  levels_.resize(max_level + 1);
  for (int i = 0; i < num_cliques; ++i) {
    descriptors_[i].level = level[i];
    levels_[level[i]].push_back(i);
  }

  // --- Precompute supernode start positions in elimination order ---
  sn_starts_.resize(num_cliques);
  for (int ci = 0; ci < num_cliques; ++ci) {
    sn_starts_[ci] = clique_tree_.supernodes[ci].empty()
                         ? 0
                         : perm_(clique_tree_.supernodes[ci][0]);
  }

  // --- Allocate device arena ---
  arena_.Allocate(descriptors_);

  // --- Allocate RHS buffer ---
  if (d_rhs_) cudaFree(d_rhs_);
  Check(cudaMalloc(&d_rhs_, num_vars_ * rhs_cols_ * sizeof(double)),
        "cudaMalloc d_rhs");

  // --- Allocate info buffer ---
  if (d_info_) cudaFree(d_info_);
  Check(cudaMalloc(&d_info_, num_cliques * sizeof(int)), "cudaMalloc d_info");

  // --- Build extend-add scatter ops ---
  // For each child-parent edge, create ScatterOps for the three regions:
  // child sep_schur -> parent sn (supernode block)
  // child sep_schur -> parent sep_rows (off-diagonal)
  // child sep_schur -> parent sep_schur (separator complement)
  //
  // The scatter pattern mirrors AccumulateUpdate in kkt_subsystem.cc.
  // We use the elimination ordering to compute which child separator
  // variables overlap with the parent's supernode vs separator ranges.
  host_scatter_ops_.clear();
  scatter_op_offsets_.resize(max_level + 1);
  scatter_op_counts_.resize(max_level + 1, 0);

  // For each child, compute scatter info into parent.
  // A child at level L scatters into its parent (at level L+1).
  // The scatter ops are grouped by the child's level.
  for (int lev = 0; lev <= max_level; ++lev) {
    scatter_op_offsets_[lev] = static_cast<int>(host_scatter_ops_.size());

    for (int ci : levels_[lev]) {
      int pi = descriptors_[ci].parent_index;
      if (pi < 0) continue;

      const auto& child_sep = clique_tree_.separators[ci];
      const auto& parent_sn = clique_tree_.supernodes[pi];
      const auto& parent_sep = clique_tree_.separators[pi];
      int child_sep_size = descriptors_[ci].sep_size;

      if (child_sep_size == 0) continue;

      // Map child separator variables to positions in parent's
      // supernode and separator blocks (in elimination order).
      // Parent supernode vars occupy elim positions
      // [parent_sn_start, parent_sn_start + parent_sn_size).
      // Parent separator vars are at known elim positions.

      // Build index: for each child sep var, find its position in
      // parent's supernode block or separator block.
      auto find_pos_in_parent = [&](int var) -> std::pair<int, int> {
        // Returns (block_type, local_index) where block_type:
        // 0 = supernode, 1 = separator, -1 = not found
        for (int j = 0; j < static_cast<int>(parent_sn.size()); ++j) {
          if (parent_sn[j] == var) return {0, j};
        }
        for (int j = 0; j < static_cast<int>(parent_sep.size()); ++j) {
          if (parent_sep[j] == var) return {1, j};
        }
        return {-1, -1};
      };

      // For each pair of child separator variables (i, j), determine
      // which parent block receives the scatter.
      // child_sep_schur(i, j) -> parent block(pos_i, pos_j)
      //
      // Group contiguous ranges into ScatterOps (matching Offset logic).
      // For simplicity, emit one ScatterOp per entry pair. A production
      // implementation would merge contiguous ranges.
      for (int i = 0; i < child_sep_size; ++i) {
        auto pos_i = find_pos_in_parent(child_sep[i]);
        int type_i = pos_i.first, idx_i = pos_i.second;
        if (type_i < 0) continue;

        for (int j = 0; j <= i; ++j) {
          auto pos_j = find_pos_in_parent(child_sep[j]);
          int type_j = pos_j.first, idx_j = pos_j.second;
          if (type_j < 0) continue;

          ScatterOp op;
          op.src_ptr = arena_.sep_schur_ptr(descriptors_[ci]);
          // Read from lower triangle (row >= col, i.e., i >= j).
          op.src_row = i;
          op.src_col = j;
          op.src_ld = child_sep_size;
          op.block_size = 1;

          // Determine destination based on parent block types.
          if (type_i == 0 && type_j == 0) {
            // Both in parent supernode (symmetric, write lower triangle).
            op.dst_ptr = arena_.sn_ptr(descriptors_[pi]);
            op.dst_row = std::max(idx_i, idx_j);
            op.dst_col = std::min(idx_i, idx_j);
            op.dst_ld = descriptors_[pi].sn_size;
          } else if (type_i == 0 && type_j == 1) {
            // i in supernode, j in separator -> separator_rows(sep_idx, sn_idx).
            op.dst_ptr = arena_.sep_rows_ptr(descriptors_[pi]);
            op.dst_row = idx_j;
            op.dst_col = idx_i;
            op.dst_ld = descriptors_[pi].sep_size;
          } else if (type_i == 1 && type_j == 0) {
            // i in separator, j in supernode -> separator_rows(sep_idx, sn_idx).
            op.dst_ptr = arena_.sep_rows_ptr(descriptors_[pi]);
            op.dst_row = idx_i;
            op.dst_col = idx_j;
            op.dst_ld = descriptors_[pi].sep_size;
          } else {
            // Both in parent separator (symmetric, write lower triangle).
            op.dst_ptr = arena_.sep_schur_ptr(descriptors_[pi]);
            op.dst_row = std::max(idx_i, idx_j);
            op.dst_col = std::min(idx_i, idx_j);
            op.dst_ld = descriptors_[pi].sep_size;
          }

          host_scatter_ops_.push_back(op);
        }
      }
    }

    scatter_op_counts_[lev] =
        static_cast<int>(host_scatter_ops_.size()) - scatter_op_offsets_[lev];
  }

  // Upload scatter ops to device.
  if (d_scatter_ops_) cudaFree(d_scatter_ops_);
  if (!host_scatter_ops_.empty()) {
    Check(cudaMalloc(&d_scatter_ops_,
                     host_scatter_ops_.size() * sizeof(ScatterOp)),
          "cudaMalloc scatter_ops");
    Check(cudaMemcpy(d_scatter_ops_, host_scatter_ops_.data(),
                     host_scatter_ops_.size() * sizeof(ScatterOp),
                     cudaMemcpyHostToDevice),
          "cudaMemcpy scatter_ops");
  }

  // --- Host staging ---
  host_data_.resize(num_cliques);

  // --- Separator indices for gather/scatter in solve ---
  // For each supernode, store the elimination positions of its separator
  // variables so the solve kernels can gather/scatter non-contiguous entries.
  std::vector<int> all_sep_indices;
  sep_indices_offsets_.resize(num_cliques);
  max_sep_size_ = 0;

  for (int ci = 0; ci < num_cliques; ++ci) {
    sep_indices_offsets_[ci] = static_cast<int>(all_sep_indices.size());
    for (int v : clique_tree_.separators[ci]) {
      if (perm_(v) < 0 || perm_(v) >= num_vars_) {
        throw std::runtime_error(
            "Separator variable " + std::to_string(v) +
            " of supernode " + std::to_string(ci) +
            " has invalid elimination position " + std::to_string(perm_(v)));
      }
      all_sep_indices.push_back(perm_(v));
    }
    max_sep_size_ = std::max(max_sep_size_, descriptors_[ci].sep_size);
  }

  if (d_sep_indices_) cudaFree(d_sep_indices_);
  d_sep_indices_ = nullptr;
  if (!all_sep_indices.empty()) {
    Check(cudaMalloc(&d_sep_indices_,
                     all_sep_indices.size() * sizeof(int)),
          "cudaMalloc sep_indices");
    Check(cudaMemcpy(d_sep_indices_, all_sep_indices.data(),
                     all_sep_indices.size() * sizeof(int),
                     cudaMemcpyHostToDevice),
          "cudaMemcpy sep_indices");
  }

  // Gather buffer for solve: sized for batched gather (max_batch_gather entries).
  int gather_buf_size = std::max(max_sep_size_, max_batch_gather_);
  if (d_gather_buf_) cudaFree(d_gather_buf_);
  d_gather_buf_ = nullptr;
  if (gather_buf_size > 0) {
    Check(cudaMalloc(&d_gather_buf_,
                     gather_buf_size * rhs_cols_ * sizeof(double)),
          "cudaMalloc gather_buf");
  }

  // --- Persistent cuSOLVER workspace (sized for largest supernode) ---
  int max_sn_size = 0;
  for (const auto& d : descriptors_)
    max_sn_size = std::max(max_sn_size, d.sn_size);

  if (d_potrf_work_) cudaFree(d_potrf_work_);
  d_potrf_work_ = nullptr;
  potrf_work_size_ = 0;
  if (max_sn_size > 0) {
    // Query workspace for the largest supernode.  cuSOLVER workspace
    // size is monotone in matrix dimension, so this covers all supernodes.
    int ws = 0;
    Check(cusolverDnDpotrf_bufferSize(cusolver_, CUBLAS_FILL_MODE_LOWER,
                                       max_sn_size, nullptr, max_sn_size, &ws),
          "potrf bufferSize (max)");
    potrf_work_size_ = ws;
    Check(cudaMalloc(&d_potrf_work_, ws * sizeof(double)),
          "cudaMalloc potrf_work");
  }

  // --- Batch groups per level (group same-size supernodes for batched APIs) ---
  level_groups_.resize(levels_.size());
  max_batch_size_ = 0;
  for (int lev = 0; lev < static_cast<int>(levels_.size()); ++lev) {
    // Sort supernodes at this level by (sn_size, sep_size).
    std::map<std::pair<int, int>, std::vector<int>> groups;
    for (int ci : levels_[lev]) {
      const auto& d = descriptors_[ci];
      if (d.sn_size == 0) continue;
      groups[{d.sn_size, d.sep_size}].push_back(ci);
    }
    for (auto& [key, indices] : groups) {
      BatchGroup g;
      g.sn_size = key.first;
      g.sep_size = key.second;
      g.indices = std::move(indices);
      max_batch_size_ = std::max(max_batch_size_,
                                  static_cast<int>(g.indices.size()));
      max_batch_gather_ = std::max(max_batch_gather_,
          g.sep_size * static_cast<int>(g.indices.size()));
      level_groups_[lev].push_back(std::move(g));
    }
  }

  // Device buffer for batched pointer arrays.
  // Need up to 4 pointer arrays simultaneously (sn, sep, schur, temp).
  if (d_batch_ptrs_) cudaFree(d_batch_ptrs_);
  d_batch_ptrs_ = nullptr;
  if (max_batch_size_ > 0) {
    Check(cudaMalloc(&d_batch_ptrs_,
                     4 * max_batch_size_ * sizeof(double*)),
          "cudaMalloc batch_ptrs");
  }

  // Device buffer for batched separator index offsets (for batched gather/scatter).
  if (d_batch_sep_offsets_) cudaFree(d_batch_sep_offsets_);
  d_batch_sep_offsets_ = nullptr;
  if (max_batch_size_ > 0) {
    Check(cudaMalloc(&d_batch_sep_offsets_,
                     max_batch_size_ * sizeof(int)),
          "cudaMalloc batch_sep_offsets");
  }

}

// --------------------------------------------------------------------------
// Assembly
// --------------------------------------------------------------------------

void GpuTreeSolver::SetSupernodeData(int sn_idx, const Eigen::MatrixXd& data) {
  host_data_[sn_idx] = data;
}

void GpuTreeSolver::DoAssemble() {
  arena_.ZeroAsync(stream_);

  const int num_cliques = static_cast<int>(descriptors_.size());
  for (int i = 0; i < num_cliques; ++i) {
    const auto& d = descriptors_[i];
    if (host_data_[i].size() == 0) continue;

    const auto& M = host_data_[i];

    // Copy supernode block (top-left sn x sn).
    if (d.sn_size > 0) {
      // Host data is column-major total x total. Extract sn x sn.
      Eigen::MatrixXd sn_block = M.topLeftCorner(d.sn_size, d.sn_size);
      Check(cudaMemcpyAsync(arena_.sn_ptr(d), sn_block.data(),
                            d.sn_size * d.sn_size * sizeof(double),
                            cudaMemcpyHostToDevice, stream_),
            "memcpy sn");
    }

    // Copy separator rows (bottom-left sep x sn).
    if (d.sep_size > 0 && d.sn_size > 0) {
      Eigen::MatrixXd sep_block =
          M.bottomLeftCorner(d.sep_size, d.sn_size);
      Check(cudaMemcpyAsync(arena_.sep_rows_ptr(d), sep_block.data(),
                            d.sep_size * d.sn_size * sizeof(double),
                            cudaMemcpyHostToDevice, stream_),
            "memcpy sep_rows");
    }

    // Copy separator Schur complement (bottom-right sep x sep).
    if (d.sep_size > 0) {
      Eigen::MatrixXd schur_block =
          M.bottomRightCorner(d.sep_size, d.sep_size);
      Check(cudaMemcpyAsync(arena_.sep_schur_ptr(d), schur_block.data(),
                            d.sep_size * d.sep_size * sizeof(double),
                            cudaMemcpyHostToDevice, stream_),
            "memcpy sep_schur");
    }
  }

  Check(cudaStreamSynchronize(stream_), "sync after assemble");
}

// --------------------------------------------------------------------------
// Factorization
// --------------------------------------------------------------------------

bool GpuTreeSolver::FactorLevel(int lev) {
  const double one = 1.0;
  const double neg_one = -1.0;

  // d_batch_ptrs_ layout: 4 arrays of max_batch_size_ each.
  // [0..bs): sn_ptrs, [bs..2bs): sep_ptrs, [2bs..3bs): schur_ptrs, [3bs..4bs): temp_ptrs
  double** d_sn_ptrs = d_batch_ptrs_;
  double** d_sep_ptrs = d_batch_ptrs_ + max_batch_size_;
  double** d_schur_ptrs = d_batch_ptrs_ + 2 * max_batch_size_;
  double** d_temp_ptrs = d_batch_ptrs_ + 3 * max_batch_size_;

  for (const auto& group : level_groups_[lev]) {
    const int bs = static_cast<int>(group.indices.size());
    const int sn = group.sn_size;
    const int sep = group.sep_size;

    // Build host pointer arrays.
    std::vector<double*> h_sn(bs), h_sep(bs), h_schur(bs), h_temp(bs);
    std::vector<int*> h_info(bs);
    for (int i = 0; i < bs; ++i) {
      const auto& d = descriptors_[group.indices[i]];
      h_sn[i] = arena_.sn_ptr(d);
      h_sep[i] = arena_.sep_rows_ptr(d);
      h_schur[i] = arena_.sep_schur_ptr(d);
      h_temp[i] = arena_.temp_ptr(d);
    }

    // Upload pointer arrays to device.
    Check(cudaMemcpyAsync(d_sn_ptrs, h_sn.data(), bs * sizeof(double*),
                          cudaMemcpyHostToDevice, stream_), "cp sn_ptrs");
    // 1. Batched Cholesky.
    Check(cusolverDnDpotrfBatched(cusolver_, CUBLAS_FILL_MODE_LOWER,
                                   sn, d_sn_ptrs, sn, d_info_, bs),
          "potrfBatched");

    if (sep > 0) {
      // Upload remaining pointer arrays.
      Check(cudaMemcpyAsync(d_sep_ptrs, h_sep.data(), bs * sizeof(double*),
                            cudaMemcpyHostToDevice, stream_), "cp sep_ptrs");
      Check(cudaMemcpyAsync(d_schur_ptrs, h_schur.data(), bs * sizeof(double*),
                            cudaMemcpyHostToDevice, stream_), "cp schur_ptrs");

      // 2. Batched triangular solve: sep = sep * L^{-T}.
      Check(cublasDtrsmBatched(cublas_, CUBLAS_SIDE_RIGHT,
                                CUBLAS_FILL_MODE_LOWER,
                                CUBLAS_OP_T, CUBLAS_DIAG_NON_UNIT,
                                sep, sn, &one,
                                d_sn_ptrs, sn,
                                d_sep_ptrs, sep, bs),
            "trsmBatched");

      // 3. Cache sep for backward solve (async copies, pipelined on stream).
      for (int i = 0; i < bs; ++i) {
        Check(cudaMemcpyAsync(h_temp[i], h_sep[i],
                              sep * sn * sizeof(double),
                              cudaMemcpyDeviceToDevice, stream_),
              "copy temp");
      }

      // 4. Batched Schur complement: schur -= sep * sep^T.
      //    Using gemmBatched (writes both triangles; dpotrf only reads lower).
      Check(cublasDgemmBatched(cublas_, CUBLAS_OP_N, CUBLAS_OP_T,
                                sep, sep, sn, &neg_one,
                                (const double**)d_sep_ptrs, sep,
                                (const double**)d_sep_ptrs, sep, &one,
                                d_schur_ptrs, sep, bs),
            "gemmBatched");
    }
  }

  // 5. Extend-add: scatter children's Schur complements into parents.
  if (scatter_op_counts_[lev] > 0) {
    LaunchExtendAdd(d_scatter_ops_ + scatter_op_offsets_[lev],
                    scatter_op_counts_[lev], stream_);
  }

  // Sync and batch-check factorization info.
  Check(cudaStreamSynchronize(stream_), "sync level");
  // cusolverDnDpotrfBatched writes info to d_info_[0..bs-1] (not per-supernode index).
  // Check all entries.
  for (const auto& group : level_groups_[lev]) {
    int bs = static_cast<int>(group.indices.size());
    std::vector<int> h_info(bs);
    Check(cudaMemcpy(h_info.data(), d_info_, bs * sizeof(int),
                     cudaMemcpyDeviceToHost), "memcpy info");
    for (int i = 0; i < bs; ++i) {
      if (h_info[i] != 0) return false;
    }
  }
  return true;
}

bool GpuTreeSolver::DoFactor() {
  for (int lev = 0; lev < num_levels(); ++lev) {
    if (!FactorLevel(lev)) return false;
  }
  return true;
}

bool GpuTreeSolver::DoAssembleAndFactor() {
  DoAssemble();
  return DoFactor();
}

// --------------------------------------------------------------------------
// Solve
// --------------------------------------------------------------------------

void GpuTreeSolver::ForwardSolve(double* d_x, int cols) const {
  const double one = 1.0;
  const double neg_one = -1.0;

  // d_batch_ptrs_ layout: [0..bs) = A ptrs, [bs..2bs) = B ptrs
  double** d_A_ptrs = d_batch_ptrs_;
  double** d_B_ptrs = d_batch_ptrs_ + max_batch_size_;

  // Process levels leaf-to-root.
  for (int lev = 0; lev < num_levels(); ++lev) {
    for (const auto& group : level_groups_[lev]) {
      const int bs = static_cast<int>(group.indices.size());
      const int sn = group.sn_size;
      const int sep = group.sep_size;

      // Batched forward trsm: x_sn = L^{-1} x_sn.
      if (bs == 1) {
        // Single supernode: unbatched trsm avoids batched API overhead.
        Check(cublasDtrsm(cublas_,
                          CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER,
                          CUBLAS_OP_N, CUBLAS_DIAG_NON_UNIT,
                          sn, cols, &one,
                          arena_.sn_ptr(descriptors_[group.indices[0]]), sn,
                          d_x + sn_starts_[group.indices[0]], num_vars_),
              "forward trsm");
      } else {
        std::vector<double*> h_A(bs), h_B(bs);
        for (int i = 0; i < bs; ++i) {
          h_A[i] = arena_.sn_ptr(descriptors_[group.indices[i]]);
          h_B[i] = d_x + sn_starts_[group.indices[i]];
        }
        Check(cudaMemcpyAsync(d_A_ptrs, h_A.data(), bs * sizeof(double*),
                              cudaMemcpyHostToDevice, stream_), "cp A");
        Check(cudaMemcpyAsync(d_B_ptrs, h_B.data(), bs * sizeof(double*),
                              cudaMemcpyHostToDevice, stream_), "cp B");
        Check(cublasDtrsmBatched(cublas_,
                                  CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER,
                                  CUBLAS_OP_N, CUBLAS_DIAG_NON_UNIT,
                                  sn, cols, &one,
                                  d_A_ptrs, sn,
                                  d_B_ptrs, num_vars_, bs),
              "forward trsmBatched");
      }

      if (sep == 0) continue;

      // Separator update: sequential gather-gemm-scatter per supernode.
      // Separator positions may overlap between supernodes (e.g., block-arrow),
      // so batching requires atomicAdd which loses precision. Keep sequential.
      for (int i = 0; i < bs; ++i) {
        int ci = group.indices[i];
        const auto& d = descriptors_[ci];
        double* x_sn = d_x + sn_starts_[ci];
        const int* sep_idx = d_sep_indices_ + sep_indices_offsets_[ci];

        LaunchGather(d_gather_buf_, d_x, sep_idx,
                     sep, cols, num_vars_, stream_);
        Check(cublasDgemm(cublas_, CUBLAS_OP_N, CUBLAS_OP_N,
                          sep, cols, sn, &neg_one,
                          arena_.sep_rows_ptr(d), sep,
                          x_sn, num_vars_, &one,
                          d_gather_buf_, sep),
              "forward gemm");
        LaunchScatter(d_gather_buf_, d_x, sep_idx,
                      sep, cols, num_vars_, stream_);
      }
    }

    Check(cudaStreamSynchronize(stream_), "sync forward level");
  }
}

void GpuTreeSolver::BackwardSolve(double* d_x, int cols) const {
  const double one = 1.0;
  const double neg_one = -1.0;

  double** d_A_ptrs = d_batch_ptrs_;
  double** d_B_ptrs = d_batch_ptrs_ + max_batch_size_;

  // Process levels root-to-leaf.
  for (int lev = num_levels() - 1; lev >= 0; --lev) {
    for (const auto& group : level_groups_[lev]) {
      const int bs = static_cast<int>(group.indices.size());
      const int sn = group.sn_size;
      const int sep = group.sep_size;

      // Separator update (before trsm): sequential gather-gemm.
      if (sep > 0) {
        for (int i = 0; i < bs; ++i) {
          int ci = group.indices[i];
          const auto& d = descriptors_[ci];
          double* x_sn = d_x + sn_starts_[ci];
          const int* sep_idx = d_sep_indices_ + sep_indices_offsets_[ci];

          LaunchGather(d_gather_buf_, d_x, sep_idx,
                       sep, cols, num_vars_, stream_);
          Check(cublasDgemm(cublas_, CUBLAS_OP_T, CUBLAS_OP_N,
                            sn, cols, sep, &neg_one,
                            arena_.sep_rows_ptr(d), sep,
                            d_gather_buf_, sep, &one,
                            x_sn, num_vars_),
                "backward gemm");
        }
      }

      // Batched backward trsm: x_sn = L^{-T} x_sn.
      if (bs == 1) {
        Check(cublasDtrsm(cublas_,
                          CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER,
                          CUBLAS_OP_T, CUBLAS_DIAG_NON_UNIT,
                          sn, cols, &one,
                          arena_.sn_ptr(descriptors_[group.indices[0]]), sn,
                          d_x + sn_starts_[group.indices[0]], num_vars_),
              "backward trsm");
      } else {
        std::vector<double*> h_A(bs), h_B(bs);
        for (int i = 0; i < bs; ++i) {
          h_A[i] = arena_.sn_ptr(descriptors_[group.indices[i]]);
          h_B[i] = d_x + sn_starts_[group.indices[i]];
        }
        Check(cudaMemcpyAsync(d_A_ptrs, h_A.data(), bs * sizeof(double*),
                              cudaMemcpyHostToDevice, stream_), "cp A");
        Check(cudaMemcpyAsync(d_B_ptrs, h_B.data(), bs * sizeof(double*),
                              cudaMemcpyHostToDevice, stream_), "cp B");
        Check(cublasDtrsmBatched(cublas_,
                                  CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER,
                                  CUBLAS_OP_T, CUBLAS_DIAG_NON_UNIT,
                                  sn, cols, &one,
                                  d_A_ptrs, sn,
                                  d_B_ptrs, num_vars_, bs),
              "backward trsmBatched");
      }
    }

    Check(cudaStreamSynchronize(stream_), "sync backward level");
  }
}

void GpuTreeSolver::DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                                    bool permute) const {
  const int n = num_vars_;
  const int cols = static_cast<int>(b.cols());

  // Permute to elimination order on host.
  Eigen::MatrixXd b_perm(n, cols);
  if (permute) {
    for (int i = 0; i < n; ++i) b_perm.row(perm_(i)) = b.row(i);
  } else {
    b_perm = b;
  }

  // Copy to device.
  Check(cudaMemcpyAsync(d_rhs_, b_perm.data(), n * cols * sizeof(double),
                        cudaMemcpyHostToDevice,
                        stream_),
        "memcpy rhs H2D");
  Check(cudaStreamSynchronize(stream_), "sync H2D");

  // Forward then backward.
  ForwardSolve(d_rhs_, cols);
  BackwardSolve(d_rhs_, cols);

  // Copy back to host.
  Check(cudaMemcpyAsync(b_perm.data(), d_rhs_, n * cols * sizeof(double),
                        cudaMemcpyDeviceToHost,
                        stream_),
        "memcpy rhs D2H");
  Check(cudaStreamSynchronize(stream_), "sync D2H");

  // Un-permute.
  if (permute) {
    for (int i = 0; i < n; ++i) b.row(i) = b_perm.row(perm_(i));
  } else {
    b = b_perm;
  }

}

Eigen::MatrixXd GpuTreeSolver::DoKKTMatrix(bool /*permute*/) const {
  // Not performance-critical. Download assembled matrix from device.
  const int n = num_vars_;
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);

  // TODO: reconstruct from per-supernode blocks on device.
  // For now, return zeros (assembly verification uses host data).
  return M;
}

}  // namespace conex
