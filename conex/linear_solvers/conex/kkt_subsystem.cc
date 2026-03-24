#define CONEX_ENABLE_TIMER 0
#include "conex/kkt_subsystem.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <thread>

#include "conex/debug_macros.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {


using T = KKTSubsystemBase;
using std::vector;

namespace {

bool AreContiguousLabels(const std::vector<int>& labels) {
  if (labels.empty()) {
    return true;
  }
  for (size_t i = 1; i < labels.size(); ++i) {
    if (labels.at(i) != labels.at(i - 1) + 1) {
      return false;
    }
  }
  return true;
}

KKTSubsystemBase::Offset GetOverlappingSegment(
    const std::vector<int>& supernodes_, const std::vector<int>& variables,
    size_t global_label) {
  KKTSubsystemBase::Offset y{0, 0, 0};
  y.second = global_label;
  size_t start = 0;
  size_t size = 0;
  for (; start < supernodes_.size(); ++start) {
    if (supernodes_.at(start) == variables.at(global_label)) {
      size = 1;
      break;
    }
  }
  if (size >= 1) {
    while (start + size < supernodes_.size() &&
           global_label + size < variables.size() &&
           supernodes_.at(start + size) == variables.at(global_label + size)) {
      size++;
    }
    y.first = start;
    y.size = size;
  }
  return y;
}

struct OffsetPattern {
  int first0 = 0;
  int second0 = 0;
  int size = 0;
  int stride_first = 0;
  int stride_second = 0;
  int count = 0;
  bool valid = false;
};

OffsetPattern DetectUniformStridePattern(
    const std::vector<KKTSubsystemBase::Offset>& offsets) {
  OffsetPattern pattern;
  if (offsets.empty()) {
    return pattern;
  }
  pattern.first0 = offsets.front().first;
  pattern.second0 = offsets.front().second;
  pattern.size = offsets.front().size;
  pattern.count = static_cast<int>(offsets.size());
  if (pattern.count == 1) {
    pattern.valid = true;
    return pattern;
  }
  pattern.stride_first = offsets.at(1).first - offsets.at(0).first;
  pattern.stride_second = offsets.at(1).second - offsets.at(0).second;
  for (int i = 1; i < pattern.count; ++i) {
    if (offsets.at(i).size != pattern.size) {
      return pattern;
    }
    if (offsets.at(i).first != pattern.first0 + i * pattern.stride_first) {
      return pattern;
    }
    if (offsets.at(i).second != pattern.second0 + i * pattern.stride_second) {
      return pattern;
    }
  }
  pattern.valid = true;
  return pattern;
}

void AddOffsetBlocks(Eigen::Ref<Eigen::MatrixXd> destination,
                     Eigen::Ref<const Eigen::MatrixXd> source,
                     const std::vector<KKTSubsystemBase::Offset>& row_offsets,
                     const std::vector<KKTSubsystemBase::Offset>& col_offsets) {
  if (row_offsets.empty() || col_offsets.empty()) {
    return;
  }
  const auto row_pattern = DetectUniformStridePattern(row_offsets);
  const auto col_pattern = DetectUniformStridePattern(col_offsets);

  if (row_pattern.valid && col_pattern.valid) {
    for (int c = 0; c < col_pattern.count; ++c) {
      const int dst_col = col_pattern.first0 + c * col_pattern.stride_first;
      const int src_col = col_pattern.second0 + c * col_pattern.stride_second;
      for (int r = 0; r < row_pattern.count; ++r) {
        const int dst_row = row_pattern.first0 + r * row_pattern.stride_first;
        const int src_row = row_pattern.second0 + r * row_pattern.stride_second;
        destination.block(dst_row, dst_col, row_pattern.size, col_pattern.size)
            .noalias() +=
            source.block(src_row, src_col, row_pattern.size, col_pattern.size);
      }
    }
    return;
  }

  for (const auto& c : col_offsets) {
    for (const auto& r : row_offsets) {
      destination.block(r.first, c.first, r.size, c.size).noalias() +=
          source.block(r.second, c.second, r.size, c.size);
    }
  }
}

void PartialScatter(const KKTSubsystemBase* source, KKTSubsystemBase* destination) {
  const auto& supernode_offsets =
      destination->local_supernode_to_source_separator(source);
  const auto& separator_offsets =
      destination->local_separator_to_source_separator(source);
  auto destination_supernode = destination->supernode_submatrix();
  auto destination_separator_rows = destination->separator_rows();
  const auto source_separator_schur = source->separator_schur_complement();
  AddOffsetBlocks(destination_supernode, source_separator_schur,
                  supernode_offsets, supernode_offsets);
  AddOffsetBlocks(destination_separator_rows, source_separator_schur,
                  separator_offsets, supernode_offsets);
}

// Scatter to direct parent only: writes sn×sn, sep×sn, AND sep×sep blocks.
// The sep×sep scatter into separator_schur_complement ensures the parent's
// sep_schur accumulates all descendant contributions, eliminating the need
// for recursive ancestor walks.
void Scatter(const KKTSubsystemBase* source,
                    KKTSubsystemBase* destination) {
  const auto& supernode_offsets =
      destination->local_supernode_to_source_separator(source);
  const auto& separator_offsets =
      destination->local_separator_to_source_separator(source);
  const auto source_separator_schur = source->separator_schur_complement();
  AddOffsetBlocks(destination->supernode_submatrix(), source_separator_schur,
                  supernode_offsets, supernode_offsets);
  AddOffsetBlocks(destination->separator_rows(), source_separator_schur,
                  separator_offsets, supernode_offsets);
  AddOffsetBlocks(destination->separator_schur_complement(),
                  source_separator_schur, separator_offsets, separator_offsets);
}

void AccumulateUpdate(const KKTSubsystemBase* source,
                      const KKTSubsystemBase* destination,
                      Eigen::Ref<Eigen::MatrixXd> supernode_delta,
                      Eigen::Ref<Eigen::MatrixXd> separator_delta) {
  const auto& supernode_offsets =
      destination->local_supernode_to_source_separator(source);
  const auto& separator_offsets =
      destination->local_separator_to_source_separator(source);
  const auto source_separator_schur = source->separator_schur_complement();
  AddOffsetBlocks(supernode_delta, source_separator_schur, supernode_offsets,
                  supernode_offsets);
  AddOffsetBlocks(separator_delta, source_separator_schur, separator_offsets,
                  supernode_offsets);
}

}  // namespace

void T::DoMultiplyAndDecrementByOffDiagonalSubMatrix(
    Eigen::Ref<MatrixXd> output, Eigen::Ref<const MatrixXd> input) const {
  const int nsep = static_cast<int>(separators_.size());
  if (nsep == 0) return;
  Eigen::Ref<Eigen::MatrixXd> temp =
      ws3().topLeftCorner(nsep, input.cols());
  temp.noalias() = separator_rows() * input;
  for (int i = 0; i < nsep; ++i) {
    output.row(separators_[i]) -= temp.row(i);
  }
}
// Iterate from the leafs of the tree upwards using recursion.
// At each leaf, we consider the triangular system
//
//  L           [x_supernodes] = b_[supernodes]
//  SR^{-1}  D  [x_seperator]    b_[separator]
//
// We then do one iteration of forward substitution
//
//  x_supernodes = L^{-1} b_supernodes
//  b_[separator] -=   SR^{-1} [x_supernodes]

void T::ApplyInverseOfLeftFactor(Eigen::Ref<Eigen::MatrixXd> x) const {
  // Do recursion all the way down to a leaf node.
  for (auto child : children_) {
    child->ApplyInverseOfLeftFactor(x);
  }
  if (supernodes_.size() == 0) {
    return;
  }

  // Solve supernode block in place.
  Eigen::Ref<Eigen::MatrixXd> x_supernodes = x.middleRows(
      supernodes_.at(0), supernodes_.back() - supernodes_.at(0) + 1);
  DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(x_supernodes);

  // Update residual via separator_rows * LeftFactor^{-1} * x_{supernodes}
  if (separators_.size() > 0) {
    Eigen::Ref<Eigen::MatrixXd> temp = ws1().topLeftCorner(
        x_supernodes.rows(), x_supernodes.cols());
    temp = x_supernodes;
    DoApplyInverseOfRightFactorOfSupernodeSubmatrix(temp);
    DoMultiplyAndDecrementByOffDiagonalSubMatrix(x, temp);
  }
}

bool T::IsRoot() const { return parent_ == nullptr; }

void T::AccumulateColumnUpdate(
    const KKTSubsystemBase* target, Eigen::Ref<Eigen::MatrixXd> supernode_delta,
    Eigen::Ref<Eigen::MatrixXd> separator_delta) const {
  const std::vector<int>& target_supernodes = target->supernodes();
  if (target_supernodes.size() == 0) {
    return;
  }
  if (separators_.size() == 0 || target_supernodes.at(0) > separators_.back()) {
    return;
  }
  AccumulateUpdate(this, target, supernode_delta, separator_delta);
  for (auto& c : children_) {
    c->AccumulateColumnUpdate(target, supernode_delta, separator_delta);
  }
}

void T::ApplyLeftLookingChildUpdates() {
  if (!left_looking_ || children_.empty()) {
    return;
  }
  const int supernode_rows = supernode_submatrix().rows();
  const int supernode_cols = supernode_submatrix().cols();
  const int separator_row_count = this->separator_rows().rows();
  const int separator_col_count = this->separator_rows().cols();
  const long long update_entries_per_accumulator =
      static_cast<long long>(supernode_rows) * supernode_cols +
      static_cast<long long>(separator_row_count) * separator_col_count;
  const long long total_update_entries =
      update_entries_per_accumulator * static_cast<long long>(children_.size());

  // For tiny updates (e.g., many 1x1 star leaves), threaded setup and
  // reduction overhead can dominate arithmetic.
  constexpr long long kSmallUpdateThresholdEntries = 1 << 14;
  if (num_threads_ <= 1 || children_.size() == 1) {
    for (auto* child : children_) {
      child->ProvideColumnUpdate(this);
    }
    return;
  }
  if (total_update_entries <= kSmallUpdateThresholdEntries) {
    for (auto* child : children_) {
      child->ProvideColumnUpdate(this);
    }
    return;
  }

  const size_t worker_count =
      std::min<size_t>(static_cast<size_t>(num_threads_), children_.size());
  std::vector<Eigen::MatrixXd> supernode_deltas(worker_count);
  std::vector<Eigen::MatrixXd> separator_deltas(worker_count);

  for (size_t i = 0; i < worker_count; ++i) {
    supernode_deltas.at(i) =
        Eigen::MatrixXd::Zero(supernode_rows, supernode_cols);
    separator_deltas.at(i) =
        Eigen::MatrixXd::Zero(separator_row_count, separator_col_count);
  }

  std::atomic<size_t> next_child(0);
  std::vector<std::thread> workers;
  workers.reserve(worker_count);
  for (size_t t = 0; t < worker_count; ++t) {
    workers.emplace_back([&, t]() {
      while (true) {
        const size_t child_index =
            next_child.fetch_add(1, std::memory_order_relaxed);
        if (child_index >= children_.size()) {
          return;
        }
        children_.at(child_index)
            ->AccumulateColumnUpdate(this, supernode_deltas.at(t),
                                     separator_deltas.at(t));
      }
    });
  }
  for (auto& worker : workers) {
    worker.join();
  }

  for (size_t i = 0; i < worker_count; ++i) {
    supernode_submatrix() += supernode_deltas.at(i);
    separator_rows() += separator_deltas.at(i);
  }
}

void T::DoMultiplyByTransposeOfOffDiagonalSubMatrix(
    Eigen::Ref<Eigen::MatrixXd> output,
    Eigen::Ref<const Eigen::MatrixXd> input) const {
  if (separators_.empty()) {
    output.setZero();
    return;
  }
  const int nsep = static_cast<int>(separators_.size());
  Eigen::Ref<Eigen::MatrixXd> gathered_separator_rows =
      ws3().topLeftCorner(nsep, input.cols());
  for (int i = 0; i < nsep; ++i) {
    gathered_separator_rows.row(i) = input.row(separators_[i]);
  }
  output.noalias() = separator_rows().transpose() * gathered_separator_rows;
}

void T::DoBackwardScatter(Eigen::Ref<Eigen::MatrixXd> output,
                          Eigen::Ref<const Eigen::MatrixXd> input) const {
  DoMultiplyByTransposeOfOffDiagonalSubMatrix(output, input);
  DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(output);
}
void T::DoBackwardScatterFromGatheredSeparator(
    Eigen::Ref<Eigen::MatrixXd> output,
    Eigen::Ref<const Eigen::MatrixXd> gathered_sep) const {
  output.noalias() = separator_rows().transpose() * gathered_sep;
  DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(output);
}
void T::ForwardSolveBlocked(Eigen::Ref<Eigen::MatrixXd> sn,
                            Eigen::Ref<Eigen::MatrixXd> sep) const {
  if (sn.rows() == 0) return;
  const int cols = sn.cols();
  DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(sn);
  if (sep.rows() > 0) {
    Eigen::Ref<Eigen::MatrixXd> temp =
        ws1().topLeftCorner(sn.rows(), cols);
    temp = sn;
    DoApplyInverseOfRightFactorOfSupernodeSubmatrix(temp);
    sep.noalias() += separator_rows() * temp;
  }
}

void T::BackwardSolveBlocked(Eigen::Ref<Eigen::MatrixXd> sn,
                             Eigen::Ref<const Eigen::MatrixXd> sep) const {
  if (sn.rows() == 0) return;
  if (sep.rows() > 0) {
    Eigen::Ref<Eigen::MatrixXd> temp =
        ws2().topLeftCorner(sn.rows(), sn.cols());
    DoBackwardScatterFromGatheredSeparator(temp, sep);
    sn -= temp;
  }
  DoApplyInverseOfRightFactorOfSupernodeSubmatrix(sn);
}
void T::BindSolveWorkspace(double* ws1, int ws1_rows, int ws1_cols,
                           double* ws2, int ws2_rows, int ws2_cols,
                           double* ws3, int ws3_rows, int ws3_cols) {
  ws1_data_ = ws1;
  ws1_rows_ = ws1_rows;
  ws2_data_ = ws2;
  ws2_rows_ = ws2_rows;
  ws3_data_ = ws3;
  ws3_rows_ = ws3_rows;
  solve_workspace_cols_ = ws1_cols;
  ws_arena_bound_ = true;
}

void T::ReserveSolveWorkspace(int rhs_cols) {
  for (auto child : children_) {
    child->ReserveSolveWorkspace(rhs_cols);
  }
  CONEX_DEMAND(AreContiguousLabels(supernodes_),
               "Non-contiguous supernodes are not supported in solve path.");
  if (rhs_cols <= solve_workspace_cols_) {
    return;
  }
  // If workspaces are arena-bound, the tree solver handles reallocation.
  if (ws1_data_) {
    return;
  }
  solve_workspace_cols_ = rhs_cols;
  int supernode_rows = 0;
  if (!supernodes_.empty()) {
    supernode_rows = supernodes_.back() - supernodes_.front() + 1;
  }
  const int separator_rows = static_cast<int>(separators_.size());
  solve_workspace1_.resize(supernode_rows, solve_workspace_cols_);
  solve_workspace2_.resize(supernode_rows, solve_workspace_cols_);
  solve_workspace3_.resize(separator_rows, solve_workspace_cols_);
}

namespace {
size_t AlignUp(size_t value, size_t alignment) {
  return ((value + alignment - 1) / alignment) * alignment;
}

bool IsAligned(const void* ptr, size_t alignment) {
  return (reinterpret_cast<std::uintptr_t>(ptr) & (alignment - 1)) == 0;
}

struct ArenaLayout {
  size_t supernode_offset_bytes = 0;
  size_t separator_rows_offset_bytes = 0;
  size_t separator_schur_offset_bytes = 0;
  size_t total_bytes = 0;
};

ArenaLayout ComputeArenaLayout(size_t n1, size_t n2) {
  constexpr size_t kAlign = EIGEN_MAX_ALIGN_BYTES;
  ArenaLayout layout;
  size_t cursor = 0;

  layout.supernode_offset_bytes = AlignUp(cursor, kAlign);
  cursor = layout.supernode_offset_bytes + n1 * n1 * sizeof(double);

  layout.separator_rows_offset_bytes = AlignUp(cursor, kAlign);
  cursor = layout.separator_rows_offset_bytes + n2 * n1 * sizeof(double);

  layout.separator_schur_offset_bytes = AlignUp(cursor, kAlign);
  cursor = layout.separator_schur_offset_bytes + n2 * n2 * sizeof(double);

  layout.total_bytes = cursor;
  return layout;
}
}  // namespace

void DenseKKTSubsystemStorage::Initialize(size_t num_supernodes,
                                          size_t num_separators) {
  if (!using_arena_memory_) {
    supernode_submatrix_storage_.resize(static_cast<int>(num_supernodes),
                                        static_cast<int>(num_supernodes));
    separator_rows_storage_.resize(static_cast<int>(num_separators),
                                   static_cast<int>(num_supernodes));
    separator_schur_complement_storage_.resize(
        static_cast<int>(num_separators), static_cast<int>(num_separators));
  }
}

size_t DenseKKTSubsystemStorage::RequiredArenaBytes(
    size_t num_supernodes, size_t num_separators) const {
  return ComputeArenaLayout(num_supernodes, num_separators).total_bytes;
}

void DenseKKTSubsystemStorage::BindArenaMemory(double* ptr, size_t bytes,
                                               size_t num_supernodes,
                                               size_t num_separators) {
  const size_t n1 = num_supernodes;
  const size_t n2 = num_separators;
  const auto layout = ComputeArenaLayout(n1, n2);
  const size_t required_bytes = layout.total_bytes;
  CONEX_DEMAND(bytes >= required_bytes, "Insufficient arena memory provided.");
  using_arena_memory_ = true;

  char* base = reinterpret_cast<char*>(ptr);
  auto* supernode_ptr =
      reinterpret_cast<double*>(base + layout.supernode_offset_bytes);
  auto* separator_rows_ptr =
      reinterpret_cast<double*>(base + layout.separator_rows_offset_bytes);
  auto* separator_schur_ptr =
      reinterpret_cast<double*>(base + layout.separator_schur_offset_bytes);
  constexpr size_t kAlign = EIGEN_MAX_ALIGN_BYTES;
  CONEX_DEMAND(IsAligned(supernode_ptr, kAlign),
               "Supernode arena pointer must be aligned.");
  CONEX_DEMAND(IsAligned(separator_rows_ptr, kAlign),
               "Separator rows arena pointer must be aligned.");
  CONEX_DEMAND(IsAligned(separator_schur_ptr, kAlign),
               "Separator schur arena pointer must be aligned.");

  supernode_submatrix_map_.emplace(supernode_ptr, static_cast<int>(n1),
                                   static_cast<int>(n1));
  separator_rows_map_.emplace(separator_rows_ptr, static_cast<int>(n2),
                              static_cast<int>(n1));
  separator_schur_complement_map_.emplace(
      separator_schur_ptr, static_cast<int>(n2), static_cast<int>(n2));
}

Eigen::Ref<Eigen::MatrixXd> DenseKKTSubsystemStorage::supernode_submatrix() {
  if (supernode_submatrix_map_) {
    return *supernode_submatrix_map_;
  }
  return supernode_submatrix_storage_;
}

Eigen::Ref<Eigen::MatrixXd>
DenseKKTSubsystemStorage::separator_schur_complement() {
  if (separator_schur_complement_map_) {
    return *separator_schur_complement_map_;
  }
  return separator_schur_complement_storage_;
}

Eigen::Ref<Eigen::MatrixXd> DenseKKTSubsystemStorage::separator_rows() {
  if (separator_rows_map_) {
    return *separator_rows_map_;
  }
  return separator_rows_storage_;
}

Eigen::Ref<const Eigen::MatrixXd>
DenseKKTSubsystemStorage::supernode_submatrix() const {
  if (supernode_submatrix_map_) {
    return *supernode_submatrix_map_;
  }
  return supernode_submatrix_storage_;
}

Eigen::Ref<const Eigen::MatrixXd>
DenseKKTSubsystemStorage::separator_schur_complement() const {
  if (separator_schur_complement_map_) {
    return *separator_schur_complement_map_;
  }
  return separator_schur_complement_storage_;
}

Eigen::Ref<const Eigen::MatrixXd> DenseKKTSubsystemStorage::separator_rows()
    const {
  if (separator_rows_map_) {
    return *separator_rows_map_;
  }
  return separator_rows_storage_;
}

KKTSubsystem::KKTSubsystem()
    : KKTSubsystem(std::make_unique<DenseKKTSubsystemStorage>()) {}

KKTSubsystem::KKTSubsystem(std::unique_ptr<KKTSubsystemStorage>&& storage)
    : storage_(std::move(storage)) {
  CONEX_DEMAND(storage_ != nullptr, "Subsystem storage must not be null.");
}

size_t KKTSubsystem::RequiredArenaBytes() const {
  return storage_->RequiredArenaBytes(supernodes_.size(), separators_.size());
}

void KKTSubsystem::BindArenaMemory(double* ptr, size_t bytes) {
  storage_->BindArenaMemory(ptr, bytes, supernodes_.size(), separators_.size());
}

// Iterate from the root of the tree downwards using depth-first search. At each
// node, we consider the triangular system
//
//  R    L^{-1} S^T          [x_supernodes] = b_[supernodes]
//            R_{seperator}  [x_separator]    b_[separator]
//
//
// We then compute x_supernodes = R^{-1} b_supernodes.
void T::ApplyInverseOfRightFactor(Eigen::Ref<Eigen::MatrixXd> x) const {
  if (supernodes_.size() > 0) {
    Eigen::Ref<Eigen::MatrixXd> x_supernodes = x.middleRows(
        supernodes_.at(0), supernodes_.back() - supernodes_.at(0) + 1);

    if (separators_.size() > 0) {
      Eigen::Ref<Eigen::MatrixXd> temp = ws2().topLeftCorner(
          x_supernodes.rows(), x_supernodes.cols());
      DoBackwardScatter(temp, x);
      x_supernodes.noalias() -= temp;
    }
    DoApplyInverseOfRightFactorOfSupernodeSubmatrix(x_supernodes);
  }

  // Children have disjoint supernodes, so backward solve is safe in parallel.
  if (num_threads_ > 1 && children_.size() > 1) {
    const size_t num_workers =
        std::min<size_t>(static_cast<size_t>(num_threads_), children_.size());
    std::atomic<size_t> next_child(0);
    std::vector<std::thread> workers;
    workers.reserve(num_workers);
    for (size_t t = 0; t < num_workers; ++t) {
      workers.emplace_back([&]() {
        while (true) {
          const size_t i =
              next_child.fetch_add(1, std::memory_order_relaxed);
          if (i >= children_.size()) return;
          children_[i]->ApplyInverseOfRightFactor(x);
        }
      });
    }
    for (auto& w : workers) {
      w.join();
    }
  } else {
    for (auto child : children_) {
      child->ApplyInverseOfRightFactor(x);
    }
  }
}

// Loop over the separators of source and record the intersections.
void T::ComputeOffsets(const KKTSubsystemBase* descendant, int start_index) {
  const auto& source_column_labels = descendant->separators();
  size_t source_separator_index = start_index;

  if (local_supernode_to_source_separator_.find(descendant) !=
      local_supernode_to_source_separator_.end()) {
    return;
  } else {
    local_supernode_to_source_separator_[descendant];
  }

  if (local_separator_to_source_separator_.find(descendant) !=
      local_separator_to_source_separator_.end()) {
    return;
  } else {
    local_separator_to_source_separator_[descendant];
  }

  while (source_separator_index < source_column_labels.size()) {
    auto local_row = GetOverlappingSegment(supernodes_, source_column_labels,
                                           source_separator_index);
    if (local_row.size != 0) {
      local_supernode_to_source_separator_[descendant].push_back(local_row);
      source_separator_index += local_row.size;
    } else {
      break;
    }
  }

  size_t index = source_separator_index;
  while (index < source_column_labels.size()) {
    auto local_row =
        GetOverlappingSegment(separators_, source_column_labels, index);
    if (local_row.size != 0) {
      local_separator_to_source_separator_[descendant].push_back(local_row);
      index += local_row.size;
    } else {
      // By the running intersection property, all separators must be present.
      throw std::runtime_error("Tree fails the running intersection property.");
    }
  }

  if (source_separator_index < source_column_labels.size()) {
    CONEX_DEMAND(parent_, "Parent pointer is null.");
    parent_->ComputeOffsets(descendant, source_separator_index);
  }
}

void T::MakeKKTMatrix(Eigen::MatrixXd* full_matrix) const {
  for (auto child : children_) {
    child->MakeKKTMatrix(full_matrix);
  }
  for (size_t j = 0; j < supernodes_.size(); j++) {
    for (size_t i = 0; i < supernodes_.size(); i++) {
      (*full_matrix)(supernodes_.at(i), supernodes_.at(j)) =
          supernode_submatrix()(i, j);
    }
    for (size_t i = 0; i < separators_.size(); i++) {
      (*full_matrix)(separators_.at(i), supernodes_.at(j)) =
          separator_rows()(i, j);
    }
  }
}

bool T::AssembleAndFactor() {
  DoInitialize();
  for (auto child : children_) {
    if (!child->AssembleAndFactor()) {
      return false;
    }
  }
  if (left_looking_) {
    START_TIMER(Update)
    ApplyLeftLookingChildUpdates();
    END_TIMER
  }
  START_TIMER(Eliminate)
  if (!DoEliminateSupernodeColumns()) {
    return false;
  }
  END_TIMER

  START_TIMER(ComputeSep)
  DoComputeSeparatorSchurComplement();
  END_TIMER
  if (!IsRoot() && !left_looking_) {
    START_TIMER(Scatter)
    DoScatterSeparatorSubmatrix();
    END_TIMER
  }
  return true;
}

void T::Assemble() {
  DoInitialize();
  for (auto& child : children_) {
    child->Assemble();
  }
  if (left_looking_) {
    ApplyLeftLookingChildUpdates();
  }

  if (!IsRoot() && !left_looking_) {
    DoScatterSeparatorSubmatrix();
  }
}

bool T::Factor() {
  for (auto& child : children_) {
    if (!child->Factor()) {
      return false;
    }
  }
  if (left_looking_) {
    ApplyLeftLookingChildUpdates();
  }
  if (supernodes_.size() > 0) {
    if (!DoEliminateSupernodeColumns()) {
      return false;
    }
  }
  // We assume that Assemble() has been called and already
  // scattered the separator sub-matrix.
  separator_schur_complement().setZero();
  DoComputeSeparatorSchurComplement();
  if (!IsRoot() && !left_looking_) {
    DoScatterSeparatorSubmatrix();
  }
  return true;
}

int T::ComputePostOrdering(int offset,
                           std::vector<int>* variable_to_elimination_position) {
  for (auto& child : children_) {
    offset =
        child->ComputePostOrdering(offset, variable_to_elimination_position);
  }
  for (auto& s : supernodes_) {
    variable_to_elimination_position->at(s) = offset++;
  }
  return offset;
};

void T::SetVariableOrdering(
    const std::vector<int>& shared_variable_to_elimination_position) {
  // Relabel and sort supernodes and separators.
  for (auto& s : supernodes_) {
    s = shared_variable_to_elimination_position.at(s);
  }

  for (auto& e : separators_) {
    e = shared_variable_to_elimination_position.at(e);
  }
  std::sort(supernodes_.begin(), supernodes_.end());
  std::sort(separators_.begin(), separators_.end());
};

void T::ReceiveColumnUpdate(const KKTSubsystemBase* source,
                            size_t start_index) {
  const auto& vars = source->separators();
  if (start_index > vars.size()) {
    return;
  }

  PartialScatter(source, this);

  size_t col_index = start_index;
  for (; col_index < vars.size(); col_index++) {
    if (supernodes_.size() == 0 || vars.at(col_index) > supernodes_.back()) {
      // The remaining columns must belong to our parent.
      break;
    }
  }

  if (col_index < vars.size()) {
    CONEX_DEMAND(parent_, "Parent pointer is null.");
    parent_->ReceiveColumnUpdate(source, col_index);
  }
}

void T::DoScatterSeparatorSubmatrix() {
  if (parent_ && separators_.size() > 0) {
    if (scatter_to_parent_) {
      Scatter(this, parent_);
    } else {
      parent_->ReceiveColumnUpdate(this, 0 /*start index*/);
    }
  }
}

void T::ComputeSeparatorOffsets() {
  for (auto& child : children_) {
    child->ComputeSeparatorOffsets();
  }

  if (!IsRoot()) {
    parent_->ComputeOffsets(this, 0);
  }
}

// Update target columns with local separator schur complement information.
// We update target column i if there is a local separator pair (j, i),
// with (j \ge i).
void T::ProvideColumnUpdate(KKTSubsystemBase* target) {
  const std::vector<int>& target_supernodes = target->supernodes();
  if (target_supernodes.size() == 0) {
    return;
  }
  if (separators_.size() == 0 || target_supernodes.at(0) > separators_.back()) {
    return;
  }

  if (scatter_to_parent_) {
    // Scatter to direct parent only (sn×sn + sep×sn + sep×sep).
    // Parent's sep_schur accumulates all descendant contributions.
    Scatter(this, target);
  } else {
    // Legacy: scatter sn×sn + sep×sn, then recurse into children
    // to scatter their sep×sep contributions to ancestors.
    PartialScatter(this, target);
    for (auto& c : children_) {
      c->ProvideColumnUpdate(target);
    }
  }
}

}  // namespace conex
