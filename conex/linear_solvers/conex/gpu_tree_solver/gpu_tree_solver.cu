#include "conex/gpu_tree_solver/gpu_tree_solver.h"

#include <algorithm>
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
      cusolver_(o.cusolver_),
      cublas_(o.cublas_),
      stream_(o.stream_),
      partition_(std::move(o.partition_)),
      host_scatter_ops_(std::move(o.host_scatter_ops_)) {
  o.d_rhs_ = nullptr;
  o.d_info_ = nullptr;
  o.d_scatter_ops_ = nullptr;
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
// Finalize: symbolic setup
// --------------------------------------------------------------------------

void GpuTreeSolver::Finalize(const CliqueTree& clique_tree, int rhs_cols) {
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
        auto [type_i, idx_i] = find_pos_in_parent(child_sep[i]);
        if (type_i < 0) continue;

        for (int j = 0; j <= i; ++j) {
          auto [type_j, idx_j] = find_pos_in_parent(child_sep[j]);
          if (type_j < 0) continue;

          ScatterOp op;
          op.src_ptr = arena_.sep_schur_ptr(descriptors_[ci]);
          op.src_row = j;  // lower triangle: row >= col
          op.src_col = i;
          op.src_ld = child_sep_size;
          op.block_size = 1;

          // Determine destination based on parent block types.
          if (type_i == 0 && type_j == 0) {
            // Both in parent supernode.
            op.dst_ptr = arena_.sn_ptr(descriptors_[pi]);
            op.dst_row = idx_j;
            op.dst_col = idx_i;
            op.dst_ld = descriptors_[pi].sn_size;
          } else if (type_i == 0 && type_j == 1) {
            // i in supernode, j in separator -> separator_rows block.
            op.dst_ptr = arena_.sep_rows_ptr(descriptors_[pi]);
            op.dst_row = idx_j;
            op.dst_col = idx_i;
            op.dst_ld = descriptors_[pi].sep_size;
          } else if (type_i == 1 && type_j == 0) {
            // i in separator, j in supernode -> separator_rows^T block.
            op.dst_ptr = arena_.sep_rows_ptr(descriptors_[pi]);
            op.dst_row = idx_i;
            op.dst_col = idx_j;
            op.dst_ld = descriptors_[pi].sep_size;
          } else {
            // Both in parent separator.
            op.dst_ptr = arena_.sep_schur_ptr(descriptors_[pi]);
            op.dst_row = idx_j;
            op.dst_col = idx_i;
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

  // --- Partition ---
  partition_ = DenseBlockPartition(num_vars_);

  // Allocate cuSOLVER workspace for the largest supernode.
  // (cuSOLVER allocates internally, so we just need d_info.)
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

    int total = d.total_size;
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
  const double zero = 0.0;

  for (int ci : levels_[lev]) {
    const auto& d = descriptors_[ci];
    if (d.sn_size == 0) continue;

    double* sn = arena_.sn_ptr(d);
    double* sep = arena_.sep_rows_ptr(d);
    double* schur = arena_.sep_schur_ptr(d);
    double* temp = arena_.temp_ptr(d);

    // 1. Cholesky factorization of supernode block: sn = L * L^T.
    int work_size = 0;
    Check(cusolverDnDpotrf_bufferSize(cusolver_, CUBLAS_FILL_MODE_LOWER,
                                      d.sn_size, sn, d.sn_size, &work_size),
          "potrf bufferSize");

    double* d_work = nullptr;
    Check(cudaMalloc(&d_work, work_size * sizeof(double)), "malloc work");

    Check(cusolverDnDpotrf(cusolver_, CUBLAS_FILL_MODE_LOWER,
                           d.sn_size, sn, d.sn_size,
                           d_work, work_size, d_info_ + ci),
          "potrf");

    cudaFree(d_work);

    // Check factorization success.
    int h_info = 0;
    Check(cudaMemcpyAsync(&h_info, d_info_ + ci, sizeof(int),
                          cudaMemcpyDeviceToHost, stream_),
          "memcpy info");
    Check(cudaStreamSynchronize(stream_), "sync info");
    if (h_info != 0) return false;

    if (d.sep_size == 0) continue;

    // 2. Triangular solve: sep = sep * L^{-T}.
    //    sep is stored as sep_size x sn_size.
    //    Solve: X * L^T = sep  =>  X = sep * L^{-T}.
    //    cuBLAS dtrsm: op(A) * X = B  with side=Right, uplo=Lower, trans=Trans.
    Check(cublasDtrsm(cublas_, CUBLAS_SIDE_RIGHT, CUBLAS_FILL_MODE_LOWER,
                      CUBLAS_OP_T, CUBLAS_DIAG_NON_UNIT,
                      d.sep_size, d.sn_size, &one,
                      sn, d.sn_size,
                      sep, d.sep_size),
          "trsm");

    // 3. Cache L^{-1} S^T in temp for backward solve.
    //    temp = L^{-1} * sep^T.  Actually we need sep * L^{-T} which
    //    is already in sep after step 2.  Copy sep to temp.
    Check(cudaMemcpyAsync(temp, sep,
                          d.sep_size * d.sn_size * sizeof(double),
                          cudaMemcpyDeviceToDevice, stream_),
          "copy temp");

    // 4. Schur complement update: schur -= sep * sep^T.
    //    schur (sep x sep) -= sep (sep x sn) * sep^T (sn x sep).
    Check(cublasDsyrk(cublas_, CUBLAS_FILL_MODE_LOWER, CUBLAS_OP_N,
                      d.sep_size, d.sn_size, &neg_one,
                      sep, d.sep_size, &one,
                      schur, d.sep_size),
          "syrk");
  }

  // 5. Extend-add: scatter children's Schur complements into parents.
  if (scatter_op_counts_[lev] > 0) {
    LaunchExtendAdd(d_scatter_ops_ + scatter_op_offsets_[lev],
                    scatter_op_counts_[lev], stream_);
  }

  Check(cudaStreamSynchronize(stream_), "sync level");
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

  // Process levels leaf-to-root.
  for (int lev = 0; lev < num_levels(); ++lev) {
    for (int ci : levels_[lev]) {
      const auto& d = descriptors_[ci];
      if (d.sn_size == 0) continue;

      double* sn = arena_.sn_ptr(d);
      double* sep = arena_.sep_rows_ptr(d);

      // x_sn = L^{-1} * x_sn.
      // Use dtrsm with side=Left, uplo=Lower, trans=NoTrans.
      // x_sn is a contiguous block in d_x starting at the supernode's
      // first elimination position.
      int sn_start = 0;
      for (int v : clique_tree_.supernodes[ci]) {
        sn_start = perm_(v);
        break;
      }
      double* x_sn = d_x + sn_start;

      Check(cublasDtrsm(cublas_,
                        CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER,
                        CUBLAS_OP_N, CUBLAS_DIAG_NON_UNIT,
                        d.sn_size, cols, &one,
                        sn, d.sn_size,
                        x_sn, num_vars_),
            "forward trsm");

      if (d.sep_size == 0) continue;

      // x_sep -= sep * x_sn.
      // sep is sep_size x sn_size, x_sn is sn_size x cols.
      // Scatter the result to separator positions in d_x.
      // For simplicity, use a temporary buffer on device.
      // TODO: direct scatter to separator positions.
      // For now, we use the dense layout where separator variables
      // are at their elimination positions in d_x.

      // Compute temp = sep * x_sn (sep_size x cols).
      double* temp = arena_.temp_ptr(d);
      Check(cublasDgemm(cublas_,
                        CUBLAS_OP_N, CUBLAS_OP_N,
                        d.sep_size, cols, d.sn_size, &neg_one,
                        sep, d.sep_size,
                        x_sn, num_vars_,
                        &one,  // Note: we need to subtract from existing x_sep.
                        // This requires gathering separator positions.
                        // Simplified: assume contiguous for now.
                        temp, d.sep_size),
            "forward gemm");

      // Scatter temp back to separator positions in d_x.
      // Each separator variable has a known elimination position.
      // This requires a small scatter kernel. For now, do it on host.
      // TODO: device scatter kernel for solve.
    }

    Check(cudaStreamSynchronize(stream_),
          "sync forward level");
  }
}

void GpuTreeSolver::BackwardSolve(double* d_x, int cols) const {
  const double one = 1.0;
  const double neg_one = -1.0;

  // Process levels root-to-leaf.
  for (int lev = num_levels() - 1; lev >= 0; --lev) {
    for (int ci : levels_[lev]) {
      const auto& d = descriptors_[ci];
      if (d.sn_size == 0) continue;

      double* sn = arena_.sn_ptr(d);
      double* temp = arena_.temp_ptr(d);

      int sn_start = 0;
      for (int v : clique_tree_.supernodes[ci]) {
        sn_start = perm_(v);
        break;
      }
      double* x_sn = d_x + sn_start;

      if (d.sep_size > 0) {
        // x_sn -= L^{-T} * sep^T * x_sep.
        // temp holds L^{-1} S^T (sn x sep) from factorization.
        // x_sn -= temp * x_sep.
        // TODO: gather x_sep from separator positions, multiply, scatter.
      }

      // x_sn = L^{-T} * x_sn.
      Check(cublasDtrsm(cublas_,
                        CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER,
                        CUBLAS_OP_T, CUBLAS_DIAG_NON_UNIT,
                        d.sn_size, cols, &one,
                        sn, d.sn_size,
                        x_sn, num_vars_),
            "backward trsm");
    }

    Check(cudaStreamSynchronize(stream_),
          "sync backward level");
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

  // Update partition.
  partition_.ScatterFrom(b);
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
