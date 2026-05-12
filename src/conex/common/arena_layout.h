// Arena-based layout templates for RowSpace and SolverRHS.
//
// Computed once at build time (from clique tree structure), then used
// to stamp out aligned instances from an Arena with zero heap allocation.

#pragma once
#include <cstring>
#include <vector>
#include "conex/common/arena.h"
#include "conex/common/tree_rhs.h"

namespace conex {

// Layout for RowSpace (= Variable): describes segment offsets and sizes.
// Each segment is padded to Arena::kAlign bytes so that
// segment_ptr(i) is always aligned for Eigen::Map<..., Aligned32>.
struct RowSpaceLayout {
  std::vector<int> offsets;    // padded offset (in doubles) per segment
  std::vector<int> sizes;      // actual size (in doubles) per segment
  int total_padded = 0;        // total doubles including padding

  static RowSpaceLayout Build(const std::vector<int>& segment_sizes) {
    RowSpaceLayout layout;
    constexpr int kAlignDoubles = Arena::kAlign / sizeof(double);
    int offset = 0;
    for (int s : segment_sizes) {
      layout.offsets.push_back(offset);
      layout.sizes.push_back(s);
      // Round up to alignment boundary.
      offset += (s + kAlignDoubles - 1) & ~(kAlignDoubles - 1);
    }
    layout.total_padded = offset;
    return layout;
  }

  int num_segments() const { return static_cast<int>(sizes.size()); }

  // Allocate a RowSpace from the arena. The data buffer is 32-byte
  // aligned, and each segment starts at a 32-byte boundary.
  // ops must be set separately by the caller.
  EuclideanJordanAlgebra::Variable Alloc(Arena& arena, int cols = 1) const {
    EuclideanJordanAlgebra::Variable rs;
    double* buf = arena.AllocArray<double>(total_padded * cols);
    std::memset(buf, 0, total_padded * cols * sizeof(double));
    rs.BindArenaData(buf, offsets, sizes, cols);
    return rs;
  }

  // Bytes required for one instance with given column count.
  size_t BytesPerInstance(int cols = 1) const {
    return static_cast<size_t>(total_padded) * cols * sizeof(double)
           + Arena::kAlign;  // alignment padding
  }
};

}  // namespace conex
