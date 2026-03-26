#pragma once
#include <cstddef>
#include <vector>

namespace conex {

// Layout descriptor for one supernode's data on device.
struct SupernodeDescriptor {
  int sn_size;       // supernode dimension
  int sep_size;      // separator dimension
  int total_size;    // sn_size + sep_size

  // Byte offsets into the device arena.
  size_t sn_offset;         // supernode_submatrix: sn x sn
  size_t sep_rows_offset;   // separator_rows: sep x sn
  size_t sep_schur_offset;  // separator_schur_complement: sep x sep
  size_t temp_offset;       // workspace for L^{-1} S^T: sn x sep

  int level;         // tree depth (leaves = 0)
  int parent_index;  // index of parent supernode (-1 for root)
};

// Single device allocation backing all per-supernode dense blocks.
// Layout is computed from the CliqueTree on the host, then a single
// cudaMalloc provides the backing storage.
class GpuDeviceArena {
 public:
  GpuDeviceArena() = default;
  ~GpuDeviceArena();

  // Non-copyable.
  GpuDeviceArena(const GpuDeviceArena&) = delete;
  GpuDeviceArena& operator=(const GpuDeviceArena&) = delete;
  GpuDeviceArena(GpuDeviceArena&&) noexcept;
  GpuDeviceArena& operator=(GpuDeviceArena&&) noexcept;

  // Compute layout from descriptors and allocate device memory.
  // descriptors[i].{sn_size, sep_size} must be filled; this function
  // fills the offset fields and allocates d_arena_.
  void Allocate(std::vector<SupernodeDescriptor>& descriptors);

  // Zero all arena memory (async on the given stream, or default stream).
  void ZeroAsync(void* stream = nullptr);

  // Raw device pointer.
  double* data() { return d_arena_; }
  const double* data() const { return d_arena_; }
  size_t bytes() const { return arena_bytes_; }

  // Device pointer to a specific block.
  double* sn_ptr(const SupernodeDescriptor& d) {
    return d_arena_ + d.sn_offset / sizeof(double);
  }
  double* sep_rows_ptr(const SupernodeDescriptor& d) {
    return d_arena_ + d.sep_rows_offset / sizeof(double);
  }
  double* sep_schur_ptr(const SupernodeDescriptor& d) {
    return d_arena_ + d.sep_schur_offset / sizeof(double);
  }
  double* temp_ptr(const SupernodeDescriptor& d) {
    return d_arena_ + d.temp_offset / sizeof(double);
  }

 private:
  double* d_arena_ = nullptr;
  size_t arena_bytes_ = 0;
};

}  // namespace conex
