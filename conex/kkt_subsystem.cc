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

Eigen::MatrixXd Submatrix(const Eigen::MatrixXd& x,
                          const std::vector<int>& rows) {
  Eigen::MatrixXd separator_rows_of_x(rows.size(), x.cols());
  for (int i = 0; i < separator_rows_of_x.rows(); i++) {
    separator_rows_of_x.row(i) = x.row(rows[i]);
  }
  return separator_rows_of_x;
}

void ScatterRows(const Eigen::MatrixXd& source, const std::vector<int>& rows,
                 Eigen::Ref<Eigen::MatrixXd> destination) {
  CONEX_DEMAND(source.rows() == static_cast<int>(rows.size()),
               "Source row count must match index set.");
  CONEX_DEMAND(source.cols() == destination.cols(),
               "Source and destination column counts must match.");
  for (int i = 0; i < source.rows(); ++i) {
    destination.row(rows.at(i)) = source.row(i);
  }
}

void CheckBlockBounds(const Eigen::MatrixXd& matrix, int start_row,
                      int start_col, int block_rows, int block_cols,
                      const char* label) {
  CONEX_DEMAND(start_row >= 0 && start_col >= 0 && block_rows >= 0 &&
                   block_cols >= 0,
               label);
  CONEX_DEMAND(start_row + block_rows <= matrix.rows(), label);
  CONEX_DEMAND(start_col + block_cols <= matrix.cols(), label);
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

void Update(const KKTSubsystemBase* source, KKTSubsystemBase* destination) {
  const auto supernode_offsets =
      destination->local_supernode_to_source_separator(source);
  const auto separator_offsets =
      destination->local_separator_to_source_separator(source);
  for (const auto& c : supernode_offsets) {
    for (const auto& r : supernode_offsets) {
      CheckBlockBounds(destination->supernode_submatrix(), r.first, c.first,
                       r.size, c.size, "supernode destination block OOB");
      CheckBlockBounds(source->separator_schur_complement(), r.second, c.second,
                       r.size, c.size, "source schur block OOB");
      destination->supernode_submatrix()
          .block(r.first, c.first, r.size, c.size)
          .noalias() += source->separator_schur_complement().block(
          r.second, c.second, r.size, c.size);
    }
    for (const auto& r : separator_offsets) {
      CheckBlockBounds(destination->separator_rows(), r.first, c.first, r.size,
                       c.size, "separator rows destination block OOB");
      CheckBlockBounds(source->separator_schur_complement(), r.second, c.second,
                       r.size, c.size, "source schur block OOB");
      destination->separator_rows()
          .block(r.first, c.first, r.size, c.size)
          .noalias() += source->separator_schur_complement().block(
          r.second, c.second, r.size, c.size);
    }
  }
}

void AccumulateUpdate(const KKTSubsystemBase* source,
                      const KKTSubsystemBase* destination,
                      Eigen::Ref<Eigen::MatrixXd> supernode_delta,
                      Eigen::Ref<Eigen::MatrixXd> separator_delta) {
  const auto supernode_offsets =
      destination->local_supernode_to_source_separator(source);
  const auto separator_offsets =
      destination->local_separator_to_source_separator(source);
  for (const auto& c : supernode_offsets) {
    for (const auto& r : supernode_offsets) {
      CheckBlockBounds(supernode_delta, r.first, c.first, r.size, c.size,
                       "supernode delta block OOB");
      CheckBlockBounds(source->separator_schur_complement(), r.second, c.second,
                       r.size, c.size, "source schur block OOB");
      supernode_delta.block(r.first, c.first, r.size, c.size).noalias() +=
          source->separator_schur_complement().block(r.second, c.second, r.size,
                                                     c.size);
    }
    for (const auto& r : separator_offsets) {
      CheckBlockBounds(separator_delta, r.first, c.first, r.size, c.size,
                       "separator delta block OOB");
      CheckBlockBounds(source->separator_schur_complement(), r.second, c.second,
                       r.size, c.size, "source schur block OOB");
      separator_delta.block(r.first, c.first, r.size, c.size).noalias() +=
          source->separator_schur_complement().block(r.second, c.second, r.size,
                                                     c.size);
    }
  }
}

}  // namespace

void T::DoMultiplyAndDecrementByOffDiagonalSubMatrix(
    Eigen::Ref<MatrixXd> output, Eigen::Ref<const MatrixXd> input) const {
  for (int i = 0; i < separator_rows().rows(); i++) {
    output.row(separators()[i]) -= separator_rows().row(i) * input;
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

  // Gather supernode rows explicitly because labels are not necessarily
  // contiguous in the global ordering.
  Eigen::MatrixXd x_supernodes = Submatrix(x, supernodes_);
  DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(x_supernodes);

  // Update residual via separator_rows * LeftFactor^{-1} * x_{supernodes}
  if (separators_.size() > 0) {
    Eigen::Ref<Eigen::MatrixXd> temp =
        solve_workspace1_.topLeftCorner(x_supernodes.rows(), x_supernodes.cols());
    temp = x_supernodes;
    DoApplyInverseOfRightFactorOfSupernodeSubmatrix(temp);
    DoMultiplyAndDecrementByOffDiagonalSubMatrix(x, temp);
  }
  ScatterRows(x_supernodes, supernodes_, x);
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
  if (num_threads_ <= 1 || children_.size() == 1) {
    for (auto* child : children_) {
      child->ProvideColumnUpdate(this);
    }
    return;
  }

  const size_t worker_count =
      std::min<size_t>(static_cast<size_t>(num_threads_), children_.size());
  std::vector<Eigen::MatrixXd> supernode_deltas(worker_count);
  std::vector<Eigen::MatrixXd> separator_deltas(worker_count);

  const int supernode_rows = supernode_submatrix().rows();
  const int supernode_cols = supernode_submatrix().cols();
  const int separator_row_count = this->separator_rows().rows();
  const int separator_col_count = this->separator_rows().cols();
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
  output.noalias() = separator_rows().transpose() * Submatrix(input, separators_);
}

void T::ReserveSolveWorkspace(int rhs_cols) {
  for (auto child : children_) {
    child->ReserveSolveWorkspace(rhs_cols);
  }
  if (rhs_cols <= solve_workspace_cols_) {
    return;
  }
  solve_workspace_cols_ = rhs_cols;
  const int supernode_rows = static_cast<int>(supernodes_.size());
  solve_workspace1_.resize(supernode_rows, solve_workspace_cols_);
  solve_workspace2_.resize(supernode_rows, solve_workspace_cols_);
}

namespace {
size_t AlignUp(size_t value, size_t alignment) {
  return ((value + alignment - 1) / alignment) * alignment;
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

size_t KKTSubsystem::RequiredArenaBytes() const {
  return ComputeArenaLayout(supernodes_.size(), separators_.size()).total_bytes;
}

void KKTSubsystem::BindArenaMemory(double* ptr, size_t bytes) {
  const size_t n1 = supernodes_.size();
  const size_t n2 = separators_.size();
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

  supernode_submatrix_map_.emplace(supernode_ptr, static_cast<int>(n1),
                                   static_cast<int>(n1));
  separator_rows_map_.emplace(separator_rows_ptr, static_cast<int>(n2),
                              static_cast<int>(n1));
  separator_schur_complement_map_.emplace(separator_schur_ptr,
                                          static_cast<int>(n2),
                                          static_cast<int>(n2));
}

// Iterate from the root of the tree downwards using depth-first search. At each
// node, we consider the triangular system
//
//  R    L^{-1} S^T          [x_supernodes] = b_[supernodes]
//            R_{seperator}  [x_separator]    b_[separator]
//
// Since we have already solved for x_separator, we first
// update the residual via
//
//  b_[supernodes] -=  L^{-1} S^T [x_separator]
//
// We then compute x_supernodes = R^{-1} b_supernodes.
void T::ApplyInverseOfRightFactor(Eigen::Ref<Eigen::MatrixXd> x) const {
  if (supernodes_.size() > 0) {
    // Gather supernode rows explicitly because labels are not necessarily
    // contiguous in the global ordering.
    Eigen::MatrixXd x_supernodes = Submatrix(x, supernodes_);

    // Update residual using x_separator computed by ascendants in tree.
    if (separators_.size() > 0) {
      Eigen::Ref<Eigen::MatrixXd> temp =
          solve_workspace2_.topLeftCorner(x_supernodes.rows(), x_supernodes.cols());
      DoMultiplyByTransposeOfOffDiagonalSubMatrix(temp, x);
      DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(temp);
      x_supernodes.noalias() -= temp;
    }
    DoApplyInverseOfRightFactorOfSupernodeSubmatrix(x_supernodes);
    ScatterRows(x_supernodes, supernodes_, x);
  }

  for (auto child : children_) {
    child->ApplyInverseOfRightFactor(x);
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
      std::runtime_error("Tree fails the running intersection property.");
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

void T::DoComputeOffsets() {
  if (parent_ && separators_.size() > 0) {
    parent_->ComputeOffsets(this, 0 /*start index*/);
  }
}

void T::ReceiveColumnUpdate(const KKTSubsystemBase* source,
                            size_t start_index) {
  const auto& vars = source->separators();
  if (start_index > vars.size()) {
    return;
  }

  Update(source, this);

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

void T::AddSparseMatrixTriplets(
    vector<Eigen::Triplet<double>>* triplets) const {
  for (size_t j = 0; j < supernodes_.size(); j++) {
    for (size_t i = 0; i < supernodes_.size(); i++) {
      triplets->emplace_back(supernodes_.at(i), supernodes_.at(j),
                             supernode_submatrix()(i, j));
    }
    for (size_t i = 0; i < separators_.size(); i++) {
      triplets->emplace_back(separators_.at(i), supernodes_.at(j),
                             separator_rows()(i, j));
    }
  }
}

void T::DoScatterSeparatorSubmatrix() {
  if (parent_ && separators_.size() > 0) {
    parent_->ReceiveColumnUpdate(this, 0 /*start index*/);
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
// We update target column i if their is a local separator pair (j, i),
// with (j \ge i).
void T::ProvideColumnUpdate(KKTSubsystemBase* target) {
  const std::vector<int>& target_supernodes = target->supernodes();
  const std::vector<int>& target_separators = target->separators();
  if (target_supernodes.size() == 0) {
    return;
  }
  if (separators_.size() == 0 || target_supernodes.at(0) > separators_.back()) {
    return;
  }
  Update(this, target);

  for (auto& c : children_) {
    c->ProvideColumnUpdate(target);
  }
}

}  // namespace conex
