#pragma once
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include "conex/common/block_partition.h"
#include "conex/common/block_variable.h"
#include "conex/common/symmetric_cone_operations.h"

namespace conex {

// Arena-allocated separator scratch: one block per subsystem, contiguous
// in memory.  Used for separator temporaries during blocked solve and
// vector multiply.
struct SeparatorScratch {
  std::vector<int> sep_rows;
  std::vector<int> offsets;
  int total_rows = 0;
  int reserved_cols = 0;
  std::unique_ptr<void, decltype(&std::free)> arena{nullptr, &std::free};
  std::vector<double*> block_ptrs;

  template <typename Subsystem>
  void Init(const std::vector<Subsystem*>& subsystems, int cols) {
    sep_rows.clear();
    offsets.clear();
    int off = 0;
    for (auto* s : subsystems) {
      int sr = static_cast<int>(s->separators().size());
      sep_rows.push_back(sr);
      offsets.push_back(off);
      off += sr * cols;  // Each block occupies sr * cols doubles.
    }
    total_rows = 0;
    for (int sr : sep_rows) total_rows += sr;
    reserved_cols = cols;
    constexpr size_t kAlign = EIGEN_MAX_ALIGN_BYTES;
    size_t bytes = static_cast<size_t>(off) * sizeof(double);
    bytes = ((bytes + kAlign - 1) / kAlign) * kAlign;
    if (bytes > 0) {
      void* raw = nullptr;
      if (posix_memalign(&raw, kAlign, bytes) != 0) throw std::bad_alloc();
      arena.reset(raw);
    }
    block_ptrs.resize(sep_rows.size());
    double* base = static_cast<double*>(arena.get());
    for (size_t k = 0; k < sep_rows.size(); ++k) {
      block_ptrs[k] = base ? base + offsets[k] : nullptr;
    }
  }

  void SetZero() const {
    if (arena) {
      // Total doubles = last offset + last block's size * cols.
      size_t total_doubles = 0;
      if (!sep_rows.empty()) {
        total_doubles = offsets.back() + sep_rows.back() * reserved_cols;
      }
      std::memset(arena.get(), 0, total_doubles * sizeof(double));
    }
  }

  Eigen::Map<Eigen::MatrixXd> block(int k, int cols) const {
    return {block_ptrs[k], sep_rows[k], cols};
  }
};

// Solver-agnostic right-hand side: supernode blocks plus optional
// separator scratch.  For dense/GPU solvers, separators is null and
// all data lives in the supernode blocks.  For the tree solver,
// separators holds unscattered contributions that get folded during
// the blocked solve.
struct SolverRHS {
  BlockPartition* supernodes = nullptr;
  SeparatorScratch* separators = nullptr;  // null for dense/GPU solvers
  bool blocks_fully_gathered = true;

  bool has_separators() const { return separators != nullptr; }

  void SetZero() {
    supernodes->SetZero();
    if (has_separators()) separators->SetZero();
    blocks_fully_gathered = !has_separators();
  }

  int cols() const { return supernodes->cols(); }
  int num_blocks() const { return supernodes->num_blocks(); }

  // Copy a single-column SolverRHS into column `col` of this multi-column
  // SolverRHS (supernodes and separators).
  void SetColumn(int col, const SolverRHS& src) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k).col(col) = src.supernodes->block(k).col(0);
    if (has_separators() && src.has_separators()) {
      int nc = cols();
      int src_nc = src.cols();
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc).col(col) =
            src.separators->block(k, src_nc).col(0);
    }
  }

  // Assign from a BlockVariable (copies supernode blocks, zeros sep).
  SolverRHS& operator=(const BlockVariable& bv) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) = bv.partition().block(k);
    if (has_separators()) separators->SetZero();
    blocks_fully_gathered = true;
    return *this;
  }

  // Assign from another SolverRHS (copy blocks + sep).
  SolverRHS& operator=(const SolverRHS& other) {
    if (this == &other) return *this;
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) = other.supernodes->block(k);
    if (has_separators() && other.has_separators()) {
      int nc = cols();
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) = other.separators->block(k, nc);
    }
    blocks_fully_gathered = other.blocks_fully_gathered;
    return *this;
  }

  SolverRHS& operator*=(double alpha) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) *= alpha;
    if (has_separators() && !blocks_fully_gathered) {
      int nc = cols();
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) *= alpha;
    }
    return *this;
  }

  SolverRHS& operator+=(const SolverRHS& other) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) += other.supernodes->block(k);
    if (has_separators() && !other.blocks_fully_gathered) {
      int nc = cols();
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) += other.separators->block(k, nc);
    }
    blocks_fully_gathered = blocks_fully_gathered && other.blocks_fully_gathered;
    return *this;
  }

  SolverRHS& operator-=(const SolverRHS& other) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) -= other.supernodes->block(k);
    if (has_separators() && !other.blocks_fully_gathered) {
      int nc = cols();
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) -= other.separators->block(k, nc);
    }
    blocks_fully_gathered = blocks_fully_gathered && other.blocks_fully_gathered;
    return *this;
  }

  // Add a BlockVariable (scattered data — only touches supernode blocks).
  SolverRHS& operator+=(const BlockVariable& bv) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) += bv.partition().block(k);
    return *this;
  }

  // Dot product (block-wise, no dense gather).
  double dot(const SolverRHS& other) const {
    double result = 0;
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      result += supernodes->block(k)
                    .cwiseProduct(other.supernodes->block(k))
                    .sum();
    return result;
  }

  // Dot product with a BlockVariable (block-wise).
  double dot(const BlockVariable& bv) const {
    double result = 0;
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      result += supernodes->block(k)
                    .cwiseProduct(bv.partition().block(k))
                    .sum();
    return result;
  }

  // AddScaled: this += alpha * other (block-wise).
  SolverRHS& AddScaled(double alpha, const SolverRHS& other) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) += alpha * other.supernodes->block(k);
    if (has_separators() && !other.blocks_fully_gathered) {
      int nc = cols();
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) += alpha * other.separators->block(k, nc);
    }
    blocks_fully_gathered = blocks_fully_gathered && other.blocks_fully_gathered;
    return *this;
  }
};

// Pretty-print a SolverRHS by gathering into a dense vector.
inline std::ostream& operator<<(std::ostream& os, const SolverRHS& rhs) {
  if (!rhs.supernodes) { os << "(null SolverRHS)"; return os; }
  int n = rhs.supernodes->num_variables();
  int nc = rhs.cols();
  Eigen::MatrixXd dense(n, nc);
  rhs.supernodes->GatherInto(dense);
  os << "SolverRHS (" << n << " x " << nc << "):\n" << dense.transpose();
  return os;
}

namespace EuclideanJordanAlgebra {

class BarrierConeOperations;       // forward declaration
class SymmetricConeOperations;     // forward declaration

// Element of a product of Euclidean Jordan algebras.
// Single-column by default; supports n-column for batched operations.
// Each segment has an associated BarrierConeOperations for dispatching.
class Variable {
 public:
  // Segment metadata (public for read access by MakeRowSpace builders).
  std::vector<int> offsets;
  std::vector<int> sizes;
  std::vector<const BarrierConeOperations*> ops;  // one per segment (non-owning)

  // Allocate storage.
  void resize(int rows, int ncols) { data_.resize(rows, ncols); }
  void setZero(int rows, int ncols) { data_.setZero(rows, ncols); }

  // Multi-column block for constraint i (rows x cols).
  Eigen::Block<Eigen::MatrixXd> segment(int i) {
    return data_.block(offsets[i], 0, sizes[i], data_.cols());
  }
  const Eigen::Block<const Eigen::MatrixXd> segment(int i) const {
    return data_.block(offsets[i], 0, sizes[i], data_.cols());
  }

  // Raw pointer to segment data (for BarrierConeOperations dispatch).
  // Arena path: returns aligned pointer via padded offsets.
  double* segment_ptr(int i) {
    return use_arena_ ? arena_buf_ + offsets[i] : &data_(offsets[i], 0);
  }
  const double* segment_ptr(int i) const {
    return use_arena_ ? arena_buf_ + offsets[i] : &data_(offsets[i], 0);
  }

  // Column access (for KKT interface: SetWeights, GetAffineTerm, etc.).
  // Arena path: returns a Map over the padded buffer.
  Eigen::Map<Eigen::VectorXd> col(int c = 0) {
    if (use_arena_) {
      return Eigen::Map<Eigen::VectorXd>(
          arena_buf_ + c * arena_total_padded_, arena_total_padded_);
    }
    return Eigen::Map<Eigen::VectorXd>(data_.col(c).data(), data_.rows());
  }
  Eigen::Map<const Eigen::VectorXd> col(int c = 0) const {
    if (use_arena_) {
      return Eigen::Map<const Eigen::VectorXd>(
          arena_buf_ + c * arena_total_padded_, arena_total_padded_);
    }
    return Eigen::Map<const Eigen::VectorXd>(data_.col(c).data(), data_.rows());
  }

  int total_rows() const {
    return use_arena_ ? arena_total_padded_ : static_cast<int>(data_.rows());
  }
  int cols() const {
    return use_arena_ ? arena_cols_ : static_cast<int>(data_.cols());
  }
  int num_constraints() const { return static_cast<int>(sizes.size()); }
  void SetZero() {
    if (use_arena_) {
      std::memset(arena_buf_, 0,
                  arena_total_padded_ * arena_cols_ * sizeof(double));
    } else {
      data_.setZero();
    }
  }

  // Set all entries to a scalar value (bypasses cone dispatch).
  // Useful for setting per-row scalar weights uniformly (e.g., identity
  // Gram weights where setOnes would produce the EJA identity element).
  void SetScalarWeights(double val) { data_.setConstant(val); }

  Variable& operator*=(double alpha) {
    if (use_arena_) {
      for (int i = 0; i < arena_total_padded_ * arena_cols_; ++i)
        arena_buf_[i] *= alpha;
    } else {
      data_ *= alpha;
    }
    return *this;
  }
  Variable& operator+=(const Variable& o) {
    if (use_arena_ && o.use_arena_) {
      for (int i = 0; i < arena_total_padded_ * arena_cols_; ++i)
        arena_buf_[i] += o.arena_buf_[i];
    } else if (!use_arena_ && !o.use_arena_) {
      data_ += o.data_;
    } else {
      // Mixed: operate per-segment.
      for (int s = 0; s < num_constraints(); ++s)
        for (int j = 0; j < sizes[s]; ++j)
          segment_ptr(s)[j] += o.segment_ptr(s)[j];
    }
    return *this;
  }
  Variable& operator-=(const Variable& o) {
    if (use_arena_ && o.use_arena_) {
      for (int i = 0; i < arena_total_padded_ * arena_cols_; ++i)
        arena_buf_[i] -= o.arena_buf_[i];
    } else if (!use_arena_ && !o.use_arena_) {
      data_ -= o.data_;
    } else {
      for (int s = 0; s < num_constraints(); ++s)
        for (int j = 0; j < sizes[s]; ++j)
          segment_ptr(s)[j] -= o.segment_ptr(s)[j];
    }
    return *this;
  }

  friend Variable operator+(const Variable& a, const Variable& b) {
    Variable out = a;
    out += b;
    return out;
  }

  friend Variable operator-(const Variable& a, const Variable& b) {
    Variable out = a;
    out -= b;
    return out;
  }

  friend Variable operator*(double alpha, const Variable& a) {
    Variable out = a;
    out *= alpha;
    return out;
  }

  friend Variable operator*(const Variable& a, double alpha) {
    return alpha * a;
  }

  // Jordan product: a * b.
  //   Nonneg: elementwise a_i * b_i.
  //   PSD: (AB + BA) / 2.
  friend Variable operator*(const Variable& a, const Variable& b) {
    Variable out;
    out.offsets = a.offsets;
    out.sizes = a.sizes;
    out.ops = a.ops;
    out.data_.resizeLike(a.data_);
    for (int i = 0; i < a.num_constraints(); ++i)
      static_cast<const SymmetricConeOperations*>(a.ops[i])->product(
          out.segment_ptr(i), a.segment_ptr(i),
          b.segment_ptr(i), a.sizes[i]);
    return out;
  }

  // Bind to arena-allocated memory with padded segment offsets.
  // After binding, segment_ptr(i) returns aligned pointers.
  // The data is NOT owned by this Variable — the arena owns it.
  void BindArenaData(double* buf, const std::vector<int>& padded_offsets,
                     const std::vector<int>& seg_sizes, int ncols) {
    offsets = padded_offsets;
    sizes = seg_sizes;
    arena_buf_ = buf;
    arena_total_padded_ = padded_offsets.empty() ? 0 :
        padded_offsets.back() + ((seg_sizes.back() + 3) & ~3);  // approx
    arena_cols_ = ncols;
    // Point data_ at the arena buffer (non-owning via Map).
    // We use a 1-row dummy to keep data_.rows()/cols() working for
    // total_rows() and cols() queries. Segment access goes through arena_buf_.
    new (&data_) Eigen::MatrixXd(0, 0);  // empty, not used for storage
    use_arena_ = true;
  }

  bool uses_arena() const { return use_arena_; }

 private:
  Eigen::MatrixXd data_;     // heap-owned storage (non-arena path)
  double* arena_buf_ = nullptr;
  int arena_total_padded_ = 0;
  int arena_cols_ = 0;
  bool use_arena_ = false;
};

// Pretty-print a Variable.  PSD segments (n² rows where n = sqrt(size))
// are reshaped into n×n matrices.  Nonneg segments print as vectors.
inline std::ostream& operator<<(std::ostream& os, const Variable& v) {
  for (int i = 0; i < v.num_constraints(); ++i) {
    int sz = v.sizes[i];
    int n = static_cast<int>(std::round(std::sqrt(static_cast<double>(sz))));
    bool is_square = (n * n == sz && n > 1);
    os << "segment " << i << " (" << sz << " entries)";
    if (is_square) {
      os << " [" << n << "x" << n << " matrix]:\n";
      Eigen::Map<const Eigen::MatrixXd> M(v.segment_ptr(i), n, n);
      os << M << "\n";
    } else {
      os << ":\n";
      Eigen::Map<const Eigen::VectorXd> vec(v.segment_ptr(i), sz);
      os << vec.transpose() << "\n";
    }
  }
  return os;
}

}  // namespace EuclideanJordanAlgebra

// Alias for use in conex namespace.
using RowSpace = EuclideanJordanAlgebra::Variable;

}  // namespace conex
