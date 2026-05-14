#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

namespace conex {

// Monotonic bump allocator. Allocations are fast (pointer bump).
// Individual deallocations are not supported — the entire arena is
// freed at once when destroyed or Reset().
//
// All allocations are aligned to kAlign bytes (32 = AVX).
// Use SaveCursor/RestoreCursor for scoped temporary allocations.
class Arena {
 public:
  static constexpr size_t kAlign = 32;  // AVX alignment

  explicit Arena(size_t initial_bytes = 65536) {
    GrowTo(initial_bytes);
  }

  Arena(const Arena&) = delete;
  Arena& operator=(const Arena&) = delete;
  Arena(Arena&&) = default;
  Arena& operator=(Arena&&) = default;

  void* Alloc(size_t bytes, bool zero = false) {
    // Round up cursor to alignment boundary.
    uintptr_t cur = reinterpret_cast<uintptr_t>(cursor_);
    uintptr_t aligned = (cur + kAlign - 1) & ~(kAlign - 1);
    size_t padding = aligned - cur;
    size_t total = padding + bytes;
    if (cursor_ + total > end_) {
      GrowTo(total);
      cur = reinterpret_cast<uintptr_t>(cursor_);
      aligned = (cur + kAlign - 1) & ~(kAlign - 1);
      padding = aligned - cur;
      total = padding + bytes;
    }
    void* p = reinterpret_cast<void*>(aligned);
    cursor_ = reinterpret_cast<char*>(aligned) + bytes;
    if (zero) std::memset(p, 0, bytes);
    return p;
  }

  template <typename T>
  T* AllocArray(size_t n, bool zero = false) {
    return static_cast<T*>(Alloc(n * sizeof(T), zero));
  }

  // Save the current cursor position (for scoped allocation).
  // RestoreCursor frees everything allocated after the save point.
  char* SaveCursor() const { return cursor_; }
  void RestoreCursor(char* saved) {
    // Pop any blocks allocated after the save point.
    while (blocks_.size() > 1) {
      char* block_start = blocks_.back().get();
      char* block_end = block_start + block_sizes_.back();
      if (saved >= block_start && saved <= block_end) break;
      // saved is not in this block — pop it.
#ifndef NDEBUG
      std::memset(block_start, 0xCD, block_sizes_.back());
#endif
      blocks_.pop_back();
      block_sizes_.pop_back();
    }
    // Now saved is in the last (current) block.
    end_ = blocks_.back().get() + block_sizes_.back();
#ifndef NDEBUG
    // Poison from saved to end of block (not to cursor_, which may be stale).
    char* block_end = blocks_.back().get() + block_sizes_.back();
    if (saved < block_end) {
      std::memset(saved, 0xCD, block_end - saved);
    }
#endif
    cursor_ = saved;
  }

  void Reset() {
    if (!blocks_.empty()) {
      cursor_ = blocks_.front().get();
      end_ = cursor_ + block_sizes_.front();
    }
    // Keep only the first block, free the rest.
    while (blocks_.size() > 1) {
      blocks_.pop_back();
      block_sizes_.pop_back();
    }
  }

  size_t TotalAllocated() const {
    size_t total = 0;
    for (size_t s : block_sizes_) total += s;
    return total;
  }

 private:
  void GrowTo(size_t min_bytes) {
    // Ensure new block is aligned.
    size_t sz = std::max(min_bytes + kAlign, size_t(65536));
    if (!blocks_.empty()) {
      sz = std::max(sz, block_sizes_.back() * 2);
    }
    blocks_.push_back(std::unique_ptr<char[]>(new char[sz]));
    block_sizes_.push_back(sz);
    cursor_ = blocks_.back().get();
    end_ = cursor_ + sz;
  }

  std::vector<std::unique_ptr<char[]>> blocks_;
  std::vector<size_t> block_sizes_;
  char* cursor_ = nullptr;
  char* end_ = nullptr;
};

// RAII scope guard for arena allocations. Saves the cursor on construction,
// restores it on destruction. Use in loops and scoped blocks to prevent
// unbounded arena growth:
//
//   for (...) {
//     ArenaFrame frame(arena);
//     auto* p = arena.Alloc(...);  // freed when frame goes out of scope
//   }
class ArenaFrame {
 public:
  explicit ArenaFrame(Arena& arena) : arena_(arena), saved_(arena.SaveCursor()) {}
  ~ArenaFrame() { arena_.RestoreCursor(saved_); }
  ArenaFrame(const ArenaFrame&) = delete;
  ArenaFrame& operator=(const ArenaFrame&) = delete;
 private:
  Arena& arena_;
  char* saved_;
};

// Fixed-size contiguous array allocated from an Arena.
// Aligned to Arena::kAlign. Cannot grow after creation.
template <typename T>
struct ArenaVec {
  T* data = nullptr;
  int size = 0;

  T& operator[](int i) { return data[i]; }
  const T& operator[](int i) const { return data[i]; }
  T* begin() { return data; }
  T* end() { return data + size; }
  const T* begin() const { return data; }
  const T* end() const { return data + size; }
  bool empty() const { return size == 0; }

  static ArenaVec Alloc(Arena& arena, int n) {
    ArenaVec v;
    v.data = arena.AllocArray<T>(n);
    v.size = n;
    return v;
  }
};

// Open-addressing hash map allocated from an Arena.
// Fixed capacity — cannot rehash. Intended for known-size datasets.
template <typename V>
class ArenaIntMap {
 public:
  ArenaIntMap() = default;

  void Init(Arena& arena, int capacity) {
    int cap = 1;
    while (cap < capacity) cap <<= 1;
    mask_ = cap - 1;
    keys_ = arena.AllocArray<int>(cap);
    vals_ = arena.AllocArray<V>(cap);
    occupied_ = arena.AllocArray<char>(cap);
    std::memset(occupied_, 0, cap);
  }

  V* find(int key) {
    int i = key & mask_;
    while (occupied_[i]) {
      if (keys_[i] == key) return &vals_[i];
      i = (i + 1) & mask_;
    }
    return nullptr;
  }

  V& operator[](int key) {
    int i = key & mask_;
    while (occupied_[i] && keys_[i] != key) {
      i = (i + 1) & mask_;
    }
    if (!occupied_[i]) {
      occupied_[i] = 1;
      keys_[i] = key;
      vals_[i] = V{};
    }
    return vals_[i];
  }

  bool count(int key) const {
    int i = key & mask_;
    while (occupied_[i]) {
      if (keys_[i] == key) return true;
      i = (i + 1) & mask_;
    }
    return false;
  }

 private:
  int* keys_ = nullptr;
  V* vals_ = nullptr;
  char* occupied_ = nullptr;
  int mask_ = 0;
};

}  // namespace conex
