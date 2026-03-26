#include "conex/gpu_tree_solver/gpu_device_arena.h"

#include <cuda_runtime.h>
#include <stdexcept>

namespace conex {

namespace {
constexpr size_t kAlignment = 256;  // GPU-friendly alignment.

size_t AlignUp(size_t v) { return ((v + kAlignment - 1) / kAlignment) * kAlignment; }

void CheckCuda(cudaError_t err, const char* msg) {
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string(msg) + ": " + cudaGetErrorString(err));
  }
}
}  // namespace

GpuDeviceArena::~GpuDeviceArena() {
  if (d_arena_) cudaFree(d_arena_);
}

GpuDeviceArena::GpuDeviceArena(GpuDeviceArena&& o) noexcept
    : d_arena_(o.d_arena_), arena_bytes_(o.arena_bytes_) {
  o.d_arena_ = nullptr;
  o.arena_bytes_ = 0;
}

GpuDeviceArena& GpuDeviceArena::operator=(GpuDeviceArena&& o) noexcept {
  if (this != &o) {
    if (d_arena_) cudaFree(d_arena_);
    d_arena_ = o.d_arena_;
    arena_bytes_ = o.arena_bytes_;
    o.d_arena_ = nullptr;
    o.arena_bytes_ = 0;
  }
  return *this;
}

void GpuDeviceArena::Allocate(std::vector<SupernodeDescriptor>& descriptors) {
  size_t offset = 0;
  for (auto& d : descriptors) {
    int sn = d.sn_size;
    int sep = d.sep_size;

    d.sn_offset = offset;
    offset += AlignUp(sn * sn * sizeof(double));

    d.sep_rows_offset = offset;
    offset += AlignUp(sep * sn * sizeof(double));

    d.sep_schur_offset = offset;
    offset += AlignUp(sep * sep * sizeof(double));

    d.temp_offset = offset;
    offset += AlignUp(sn * sep * sizeof(double));
  }

  arena_bytes_ = offset;
  if (arena_bytes_ > 0) {
    CheckCuda(cudaMalloc(&d_arena_, arena_bytes_), "cudaMalloc arena");
  }
}

void GpuDeviceArena::ZeroAsync(void* stream) {
  if (d_arena_ && arena_bytes_ > 0) {
    CheckCuda(cudaMemsetAsync(d_arena_, 0, arena_bytes_,
                              static_cast<cudaStream_t>(stream)),
              "cudaMemsetAsync arena");
  }
}

}  // namespace conex
