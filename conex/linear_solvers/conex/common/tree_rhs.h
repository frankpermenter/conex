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
// is_scattered: if true, separator data has been gathered into
// supernode blocks (e.g. after GatherSeparators).  If false,
// separator contributions are still in the scratch buffer.
struct TreeRHS {
  BlockPartition* supernodes;
  SeparatorScratch* separators;
  bool is_scattered = false;

  void SetZero() {
    supernodes->SetZero();
    separators->SetZero();
    is_scattered = false;
  }

  int cols() const { return supernodes->cols(); }
  int num_blocks() const { return supernodes->num_blocks(); }

  // Assign from a BlockVariable (copies supernode blocks, zeros sep).
  TreeRHS& operator=(const BlockVariable& bv) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) = bv.partition().block(k);
    separators->SetZero();
    is_scattered = true;
    return *this;
  }

  // Assign from another TreeRHS (copy blocks + sep).
  TreeRHS& operator=(const TreeRHS& other) {
    if (this == &other) return *this;
    int nb = supernodes->num_blocks();
    int nc = cols();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) = other.supernodes->block(k);
    for (int k = 0; k < nb; ++k)
      separators->block(k, nc) = other.separators->block(k, nc);
    is_scattered = other.is_scattered;
    return *this;
  }

  TreeRHS& operator*=(double alpha) {
    int nb = supernodes->num_blocks();
    int nc = cols();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) *= alpha;
    if (!is_scattered) {
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) *= alpha;
    }
    return *this;
  }

  TreeRHS& operator+=(const TreeRHS& other) {
    int nb = supernodes->num_blocks();
    int nc = cols();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) += other.supernodes->block(k);
    if (!is_scattered && !other.is_scattered) {
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) += other.separators->block(k, nc);
    }
    return *this;
  }

  TreeRHS& operator-=(const TreeRHS& other) {
    int nb = supernodes->num_blocks();
    int nc = cols();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) -= other.supernodes->block(k);
    if (!is_scattered && !other.is_scattered) {
      for (int k = 0; k < nb; ++k)
        separators->block(k, nc) -= other.separators->block(k, nc);
    }
    return *this;
  }

  // Add a BlockVariable (scattered data — only touches supernode blocks).
  TreeRHS& operator+=(const BlockVariable& bv) {
    int nb = supernodes->num_blocks();
    for (int k = 0; k < nb; ++k)
      supernodes->block(k) += bv.partition().block(k);
    return *this;
  }
};

}  // namespace conex
