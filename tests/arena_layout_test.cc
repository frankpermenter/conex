// Test arena-based RowSpace allocation with aligned segments.
#include <gtest/gtest.h>
#include "conex/common/arena.h"
#include "conex/common/arena_layout.h"

using namespace conex;

TEST(ArenaLayout, Alignment) {
  Arena arena;

  // Segments of various sizes (including non-multiple-of-4).
  std::vector<int> sizes = {3, 5, 9, 4, 1};  // exp cone=3, power=4, etc.
  auto layout = RowSpaceLayout::Build(sizes);

  EXPECT_EQ(layout.num_segments(), 5);

  auto rs = layout.Alloc(arena);

  for (int i = 0; i < layout.num_segments(); ++i) {
    double* ptr = rs.segment_ptr(i);
    uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
    EXPECT_EQ(addr % Arena::kAlign, 0)
        << "segment " << i << " not aligned to " << Arena::kAlign << " bytes";
    EXPECT_EQ(rs.sizes[i], sizes[i]);
  }
}

TEST(ArenaLayout, SegmentReadWrite) {
  Arena arena;
  std::vector<int> sizes = {3, 4};
  auto layout = RowSpaceLayout::Build(sizes);

  auto rs = layout.Alloc(arena);

  // Write to segment 0.
  for (int i = 0; i < 3; ++i) rs.segment_ptr(0)[i] = i + 1.0;
  // Write to segment 1.
  for (int i = 0; i < 4; ++i) rs.segment_ptr(1)[i] = 10.0 + i;

  // Read back.
  EXPECT_DOUBLE_EQ(rs.segment_ptr(0)[0], 1.0);
  EXPECT_DOUBLE_EQ(rs.segment_ptr(0)[2], 3.0);
  EXPECT_DOUBLE_EQ(rs.segment_ptr(1)[0], 10.0);
  EXPECT_DOUBLE_EQ(rs.segment_ptr(1)[3], 13.0);

  // Segments don't overlap.
  rs.segment_ptr(0)[0] = 99.0;
  EXPECT_DOUBLE_EQ(rs.segment_ptr(1)[0], 10.0);  // unchanged
}

TEST(ArenaLayout, SaveRestore) {
  Arena arena;
  std::vector<int> sizes = {3, 4};
  auto layout = RowSpaceLayout::Build(sizes);

  char* mark = arena.SaveCursor();

  auto rs1 = layout.Alloc(arena);
  for (int i = 0; i < 3; ++i) rs1.segment_ptr(0)[i] = 1.0;

  auto rs2 = layout.Alloc(arena);
  for (int i = 0; i < 3; ++i) rs2.segment_ptr(0)[i] = 2.0;

  // Both live simultaneously.
  EXPECT_DOUBLE_EQ(rs1.segment_ptr(0)[0], 1.0);
  EXPECT_DOUBLE_EQ(rs2.segment_ptr(0)[0], 2.0);

  // Restore frees both.
  arena.RestoreCursor(mark);

  // Allocate again — reuses the same memory.
  auto rs3 = layout.Alloc(arena);
  // rs3 should be at the same address as rs1 was.
  EXPECT_EQ(rs3.segment_ptr(0), rs1.segment_ptr(0));
}

TEST(ArenaLayout, MultipleColumns) {
  Arena arena;
  std::vector<int> sizes = {3};
  auto layout = RowSpaceLayout::Build(sizes);

  auto rs = layout.Alloc(arena, 2);
  EXPECT_EQ(rs.cols(), 2);

  // Write column 0.
  rs.col(0)(0) = 1.0;
  rs.col(0)(1) = 2.0;
  rs.col(0)(2) = 3.0;

  // Write column 1 (at offset total_padded).
  rs.col(1)(0) = 10.0;

  EXPECT_DOUBLE_EQ(rs.col(0)(0), 1.0);
  EXPECT_DOUBLE_EQ(rs.col(1)(0), 10.0);
}

TEST(ArenaLayout, SetZero) {
  Arena arena;
  std::vector<int> sizes = {3, 4};
  auto layout = RowSpaceLayout::Build(sizes);

  auto rs = layout.Alloc(arena);
  for (int i = 0; i < 3; ++i) rs.segment_ptr(0)[i] = 99.0;
  for (int i = 0; i < 4; ++i) rs.segment_ptr(1)[i] = 99.0;

  rs.SetZero();

  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(rs.segment_ptr(0)[i], 0.0);
  for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(rs.segment_ptr(1)[i], 0.0);
}
