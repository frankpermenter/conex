#pragma once
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include "conex/common/block_partition.h"
#include "conex/common/block_variable.h"

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
      off += sr;
    }
    total_rows = off;
    reserved_cols = cols;
    constexpr size_t kAlign = EIGEN_MAX_ALIGN_BYTES;
    size_t bytes = static_cast<size_t>(total_rows) * cols * sizeof(double);
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
      std::memset(arena.get(), 0,
                  static_cast<size_t>(total_rows) * reserved_cols *
                      sizeof(double));
    }
  }

  Eigen::Map<Eigen::MatrixXd> block(int k, int cols) const {
    return {block_ptrs[k], sep_rows[k], cols};
  }
};

// A right-hand side in elimination-tree form: one supernode block per
// clique plus unscattered separator contributions.  The forward pass
// consumes separator data incrementally (child sep → parent sn/sep).
//
// blocks_fully_gathered: if true, all data is in supernode blocks
// and separator scratch can be ignored.  If false, separator
// contributions are still in the scratch buffer.
struct SolverRHS {
  BlockPartition* supernodes;
  SeparatorScratch* separators;
  bool blocks_fully_gathered = false;

  void SetZero() {
    supernodes->SetZero();
    separators->SetZero();
    blocks_fully_gathered = false;
  }

  int cols() const { return supernodes->cols(); }
  int num_blocks() const { return supernodes->num_blocks(); }

  // Assign from a BlockVariable (copies supernode blocks, zeros sep).
  SolverRHS& operator=(const BlockVariable& bv) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) = bv.partition().block(k);
    separators->SetZero();
    blocks_fully_gathered = true;
    return *this;
  }

  // Assign from another SolverRHS (copy blocks + sep).
  SolverRHS& operator=(const SolverRHS& other) {
    if (this == &other) return *this;
    int nb = supernodes->num_blocks();
    int nc = cols();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) = other.supernodes->block(k);
    for (int k = 0; k < nb; ++k)
      separators->block(k, nc) = other.separators->block(k, nc);
    blocks_fully_gathered = other.blocks_fully_gathered;
    return *this;
  }

  SolverRHS& operator*=(double alpha) {
    int nb = supernodes->num_blocks();
    int nc = cols();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) *= alpha;
    if (!blocks_fully_gathered) {
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) *= alpha;
    }
    return *this;
  }

  SolverRHS& operator+=(const SolverRHS& other) {
    int nb = supernodes->num_blocks();
    int nc = cols();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) += other.supernodes->block(k);
    if (!other.blocks_fully_gathered) {
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) += other.separators->block(k, nc);
    }
    blocks_fully_gathered = blocks_fully_gathered && other.blocks_fully_gathered;
    return *this;
  }

  SolverRHS& operator-=(const SolverRHS& other) {
    int nb = supernodes->num_blocks();
    int nc = cols();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) -= other.supernodes->block(k);
    if (!other.blocks_fully_gathered) {
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
  // Both operands must be scattered (data fully in supernode blocks).
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
    int nc = cols();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) += alpha * other.supernodes->block(k);
    if (!other.blocks_fully_gathered) {
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) += alpha * other.separators->block(k, nc);
    }
    blocks_fully_gathered = blocks_fully_gathered && other.blocks_fully_gathered;
    return *this;
  }
};

// Concatenated row-space vector for all linear constraints.
struct RowSpace {
  Eigen::VectorXd data;
  std::vector<int> offsets;
  std::vector<int> sizes;

  Eigen::Ref<Eigen::VectorXd> segment(int i) {
    return data.segment(offsets[i], sizes[i]);
  }
  Eigen::Ref<const Eigen::VectorXd> segment(int i) const {
    return data.segment(offsets[i], sizes[i]);
  }
  int total_rows() const { return static_cast<int>(data.size()); }
  int num_constraints() const { return static_cast<int>(sizes.size()); }
  void SetZero() { data.setZero(); }

  RowSpace& operator*=(double alpha) { data *= alpha; return *this; }
  RowSpace& operator+=(const RowSpace& o) { data += o.data; return *this; }
  RowSpace& operator-=(const RowSpace& o) { data -= o.data; return *this; }
};

}  // namespace conex
