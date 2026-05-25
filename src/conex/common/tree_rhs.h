#pragma once
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include "conex/common/arena.h"
#include "conex/common/block_partition.h"
#include "conex/common/block_variable.h"
#include "conex/common/symmetric_cone_operations.h"

namespace conex {

// Separator scratch: per-subsystem buffers for the blocked solve.
// Non-owning view: pointers into arena or externally managed memory.
struct SeparatorScratch {
  double** block_ptrs = nullptr;  // block_ptrs[k] = start of block k
  const int* sep_rows = nullptr;  // sep_rows[k] = rows in block k
  int num_blocks = 0;
  int total_doubles = 0;          // total buffer size for SetZero

  void SetZero() const {
    if (total_doubles > 0 && block_ptrs && block_ptrs[0])
      std::memset(block_ptrs[0], 0, total_doubles * sizeof(double));
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
    blocks_fully_gathered = true;
  }

  // Scatter a dense vector into supernode blocks + zero separators.
  void ScatterFrom(const double* data, int size) {
    supernodes->ScatterFrom(data, size);
    if (has_separators()) separators->SetZero();
    blocks_fully_gathered = true;
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

  // Assign from another SolverRHS (copy blocks + sep, or shallow copy if uninitialized).
  SolverRHS& operator=(const SolverRHS& other) {
    if (this == &other) return *this;
    if (!supernodes) {
      // Uninitialized destination: shallow copy (pointer sharing).
      supernodes = other.supernodes;
      separators = other.separators;
      blocks_fully_gathered = other.blocks_fully_gathered;
      return *this;
    }
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

  // Dot product (block-wise, includes separator contributions).
  double dot(const SolverRHS& other) const {
    double result = 0;
    int nb = supernodes->num_blocks();
    int nc = supernodes->cols();
    for (int k = 0; k < nb; ++k)
      result += supernodes->block(k)
                    .cwiseProduct(other.supernodes->block(k))
                    .sum();
    // Include separator data when not fully gathered into supernodes.
    if (has_separators() && other.has_separators() &&
        (!blocks_fully_gathered || !other.blocks_fully_gathered)) {
      for (int k = 0; k < nb; ++k)
        result += separators->block(k, nc)
                      .cwiseProduct(other.separators->block(k, nc))
                      .sum();
    }
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
// Variable: segmented vector for cone operations.
//
// Storage is a raw double* buffer (not Eigen::MatrixXd). The Variable
// can own its memory (heap via shared_ptr) or borrow it (arena).
// This enables zero-allocation solve loops when backed by an arena.
//
// Each segment starts at offsets[i] doubles from the buffer start.
// segment_ptr(i) returns a raw pointer; segment(i) returns an Eigen::Map.
class Variable {
 public:
  // Segment metadata.
  std::vector<int> offsets;
  std::vector<int> sizes;
  std::vector<const BarrierConeOperations*> ops;

  Variable() = default;

  // Deep copy: heap-backed Variables always get their own buffer.
  Variable(const Variable& o);
  Variable& operator=(const Variable& o);
  Variable(Variable&&) = default;
  Variable& operator=(Variable&&) = default;

  // Heap allocation.
  void resize(int rows, int ncols);
  void setZero(int rows, int ncols);

  // Arena allocation (zero heap allocation).
  void BindArenaData(double* buf, const std::vector<int>& padded_offsets,
                     const std::vector<int>& seg_sizes, int ncols);

  // Segment access (with outer stride matching the full buffer layout).
  using SegmentMap = Eigen::Map<Eigen::MatrixXd, 0, Eigen::OuterStride<>>;
  using ConstSegmentMap = Eigen::Map<const Eigen::MatrixXd, 0, Eigen::OuterStride<>>;
  SegmentMap segment(int i);
  ConstSegmentMap segment(int i) const;
  double* segment_ptr(int i) { return data_ + offsets[i]; }
  const double* segment_ptr(int i) const { return data_ + offsets[i]; }

  // Column access.
  Eigen::Map<Eigen::VectorXd> col(int c = 0);
  Eigen::Map<const Eigen::VectorXd> col(int c = 0) const;

  int total_rows() const { return rows_; }
  int cols() const { return cols_; }
  int num_constraints() const { return static_cast<int>(sizes.size()); }

  void SetZero();
  void SetScalarWeights(double val);

  Variable& operator*=(double alpha);
  Variable& operator+=(const Variable& o);
  Variable& operator-=(const Variable& o);

  friend Variable operator+(const Variable& a, const Variable& b);
  friend Variable operator-(const Variable& a, const Variable& b);
  friend Variable operator*(double alpha, const Variable& a);
  friend Variable operator*(const Variable& a, double alpha);
  friend Variable operator*(const Variable& a, const Variable& b);

  bool uses_arena() const { return !heap_storage_; }
  double* data() { return data_; }
  const double* data() const { return data_; }

 private:
  double* data_ = nullptr;
  int rows_ = 0;
  int cols_ = 0;
  std::shared_ptr<double[]> heap_storage_;
};

// Pretty-print.
std::ostream& operator<<(std::ostream& os, const Variable& v);

}  // namespace EuclideanJordanAlgebra

// Alias for use in conex namespace.
using RowSpace = EuclideanJordanAlgebra::Variable;

}  // namespace conex
