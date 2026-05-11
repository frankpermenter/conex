#pragma once
#include <cstddef>

namespace conex {

// Base class for objects whose workspace can be arena-allocated.
// The tree solver (or builder) queries RequiredArenaBytes(), does one
// contiguous allocation, then calls BindArenaMemory() on each object.
class ArenaAllocatable {
 public:
  virtual ~ArenaAllocatable() = default;
  virtual size_t RequiredArenaBytes() const = 0;
  virtual void BindArenaMemory(double* ptr, size_t bytes) = 0;
};

}  // namespace conex
