#include "conex/clique_ordering.h"

#include <algorithm>
#include <map>
#include <set>

#include "gtest/gtest.h"

namespace conex {
namespace {

// Reconstruct full cliques from CliqueTree supernodes + separators.
std::vector<std::vector<int>> ReconstructCliques(const CliqueTree& ct) {
  const int k = static_cast<int>(ct.supernodes.size());
  std::vector<std::vector<int>> cliques(k);
  for (int i = 0; i < k; i++) {
    auto bag = ct.supernodes[i];
    bag.insert(bag.end(), ct.separators[i].begin(), ct.separators[i].end());
    std::sort(bag.begin(), bag.end());
    cliques[i] = std::move(bag);
  }
  return cliques;
}

// Verify the Running Intersection Property: for each variable, the set of
// cliques containing it forms a connected subtree.
void VerifyRIP(const CliqueTree& ct,
               const std::vector<std::vector<int>>& cliques) {
  const int k = static_cast<int>(cliques.size());
  if (k == 0) return;

  // Build children from node_to_parent.
  std::vector<std::vector<int>> children(k);
  int root = -1;
  for (int i = 0; i < k; i++) {
    if (ct.node_to_parent[i] == -1)
      root = i;
    else
      children[ct.node_to_parent[i]].push_back(i);
  }
  ASSERT_NE(root, -1);

  // Map variable -> set of clique indices containing it.
  std::map<int, std::set<int>> var_to_cliques;
  for (int ci = 0; ci < k; ci++) {
    for (int v : cliques[ci]) {
      var_to_cliques[v].insert(ci);
    }
  }

  // For each variable, BFS from any containing clique along tree edges
  // restricted to containing cliques.  Must reach all of them.
  for (const auto& [var, cset] : var_to_cliques) {
    if (cset.size() <= 1) continue;
    int start = *cset.begin();
    std::vector<int> stack = {start};
    std::set<int> visited = {start};
    while (!stack.empty()) {
      int node = stack.back();
      stack.pop_back();
      // Parent edge.
      int p = ct.node_to_parent[node];
      if (p != -1 && cset.count(p) && !visited.count(p)) {
        visited.insert(p);
        stack.push_back(p);
      }
      // Children edges.
      for (int c : children[node]) {
        if (cset.count(c) && !visited.count(c)) {
          visited.insert(c);
          stack.push_back(c);
        }
      }
    }
    EXPECT_EQ(visited, cset) << "RIP violated for variable " << var;
  }
}

// Verify that every input row support is a subset of some maximal clique.
void VerifyCover(const std::vector<std::vector<int>>& cliques,
                 const std::vector<std::vector<int>>& row_supports) {
  for (size_t r = 0; r < row_supports.size(); r++) {
    bool covered = false;
    for (const auto& c : cliques) {
      if (std::includes(c.begin(), c.end(), row_supports[r].begin(),
                        row_supports[r].end())) {
        covered = true;
        break;
      }
    }
    EXPECT_TRUE(covered) << "Row " << r << " not covered by any clique";
  }
}

// Verify that the CliqueTree has the right sizes and exactly one root.
void VerifyStructure(const CliqueTree& ct, int expected_cliques) {
  ASSERT_EQ(static_cast<int>(ct.supernodes.size()), expected_cliques);
  ASSERT_EQ(static_cast<int>(ct.separators.size()), expected_cliques);
  ASSERT_EQ(static_cast<int>(ct.node_to_parent.size()), expected_cliques);
  ASSERT_EQ(static_cast<int>(ct.post_order_position_to_clique.size()),
            expected_cliques);

  int num_roots = 0;
  for (int p : ct.node_to_parent) {
    if (p == -1) num_roots++;
  }
  EXPECT_EQ(num_roots, 1);

  // Root has empty separator.
  for (int i = 0; i < expected_cliques; i++) {
    if (ct.node_to_parent[i] == -1) {
      EXPECT_TRUE(ct.separators[i].empty());
    }
  }
}

// Run the full suite of checks on MakeCliqueTreeMinDegreeFromRowSupports.
void TestMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    int expected_cliques = -1) {
  std::vector<std::vector<int>> maximal_cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(row_supports,
                                                   &maximal_cliques);

  if (expected_cliques >= 0) {
    EXPECT_EQ(static_cast<int>(maximal_cliques.size()), expected_cliques);
  }

  const int k = static_cast<int>(maximal_cliques.size());
  if (k == 0) return;

  VerifyStructure(ct, k);

  // Reconstructed cliques from supernodes+separators must match maximal
  // cliques (as sets).
  auto reconstructed = ReconstructCliques(ct);
  std::set<std::vector<int>> mc_set, recon_set;
  for (auto& c : maximal_cliques) mc_set.insert(c);
  for (auto& c : reconstructed) recon_set.insert(c);
  EXPECT_EQ(mc_set, recon_set);

  VerifyRIP(ct, maximal_cliques);
  VerifyCover(maximal_cliques, row_supports);
}

}  // namespace

// ===================================================================
// Single clique — no fill, single node tree.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, SingleClique) {
  TestMinDegreeFromRowSupports({{1, 2, 3, 4}}, /*expected_cliques=*/1);
}

// ===================================================================
// Two overlapping cliques.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, TwoCliques) {
  TestMinDegreeFromRowSupports({{0, 1, 3}, {2, 3}}, /*expected_cliques=*/2);
}

// ===================================================================
// Star graph: edges (30,10), (30,20), (30,40), (30,50).
// Same example as MakeImplicitCliqueTreeFromRowSupportsStar test.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, Star) {
  const std::vector<std::vector<int>> supports = {
      {10, 30}, {20, 30}, {30, 40}, {30, 50}};
  std::vector<std::vector<int>> cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(supports, &cliques);

  EXPECT_EQ(cliques.size(), 4u);
  VerifyStructure(ct, 4);
  VerifyRIP(ct, cliques);
  VerifyCover(cliques, supports);
}

// ===================================================================
// Path graph: edges (7,9), (9,12), (12,15), (15,17), (17,18).
// Same example as MakeImplicitCliqueTreeFromRowSupportsPath test.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, Path) {
  const std::vector<std::vector<int>> supports = {
      {7, 9}, {9, 12}, {12, 15}, {15, 17}, {17, 18}};
  TestMinDegreeFromRowSupports(supports, /*expected_cliques=*/5);
}

// ===================================================================
// Diagonal: disjoint singletons — no edges, each is its own clique.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, Diagonal) {
  TestMinDegreeFromRowSupports({{1}, {2}, {3}, {4}, {5}},
                               /*expected_cliques=*/5);
}

// ===================================================================
// Non-chordal input that requires fill: 4-cycle (0,1), (1,2), (2,3), (0,3).
// Min-degree will add one fill edge to triangulate.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, FourCycleNeedsFill) {
  const std::vector<std::vector<int>> supports = {
      {0, 1}, {1, 2}, {2, 3}, {0, 3}};
  std::vector<std::vector<int>> cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(supports, &cliques);

  // 4-cycle triangulation produces exactly 2 maximal cliques (two triangles).
  EXPECT_EQ(cliques.size(), 2u);
  VerifyStructure(ct, 2);
  VerifyRIP(ct, cliques);
  VerifyCover(cliques, supports);
}

// ===================================================================
// Non-maximal input cliques: {0,1} ⊂ {0,1,2} ⊂ {0,1,2,3,4}.
// Should reduce to a single maximal clique.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, NonMaximalInput) {
  TestMinDegreeFromRowSupports({{0, 1}, {0, 1, 2}, {0, 1, 2, 3, 4}},
                               /*expected_cliques=*/1);
}

// ===================================================================
// Two connected cliques sharing a triangle: {1,2,3,4} and {1,2,3,5}.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, TwoCliquesSharedTriangle) {
  const std::vector<std::vector<int>> supports = {
      {1, 2, 3, 4}, {1, 2, 3, 5}};
  std::vector<std::vector<int>> cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(supports, &cliques);

  VerifyStructure(ct, static_cast<int>(cliques.size()));
  VerifyRIP(ct, cliques);
  VerifyCover(cliques, supports);
}

// ===================================================================
// Three cliques sharing a triangle: {1,2,3,4}, {1,2,3,5}, {1,2,3,6}.
// No fill needed — already chordal.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, ThreeCliquesZeroFill) {
  const std::vector<std::vector<int>> supports = {
      {1, 2, 3, 4}, {1, 2, 3, 5}, {1, 2, 3, 6}};
  std::vector<std::vector<int>> cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(supports, &cliques);

  EXPECT_EQ(cliques.size(), 3u);
  VerifyStructure(ct, 3);
  VerifyRIP(ct, cliques);
  VerifyCover(cliques, supports);
}

// ===================================================================
// Larger non-chordal example from the existing test suite.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, LargerMixed) {
  TestMinDegreeFromRowSupports(
      {{1, 2, 3, 5}, {3, 4, 5}, {4, 5, 6, 7}, {8, 9}, {1, 11}});
}

// ===================================================================
// Empty input.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, Empty) {
  std::vector<std::vector<int>> cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports({}, &cliques);
  EXPECT_TRUE(cliques.empty());
  EXPECT_TRUE(ct.supernodes.empty());
}

// ===================================================================
// Nullptr for maximal_cliques_out should not crash.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, NullCliquesOut) {
  const std::vector<std::vector<int>> supports = {{0, 1, 2}, {1, 2, 3}};
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(supports, nullptr);
  EXPECT_EQ(ct.supernodes.size(), ct.separators.size());
}

// ===================================================================
// Complete graph on 6 vertices — single maximal clique.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, CompleteGraph) {
  // All pairs from {0,...,5} — the union is one big clique.
  const std::vector<std::vector<int>> supports = {
      {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5},
      {1, 2}, {1, 3}, {1, 4}, {1, 5},
      {2, 3}, {2, 4}, {2, 5},
      {3, 4}, {3, 5},
      {4, 5}};
  TestMinDegreeFromRowSupports(supports, /*expected_cliques=*/1);
}

// ===================================================================
// Disconnected components: {0,1,2} and {10,11,12} with no overlap.
// ===================================================================
GTEST_TEST(CliqueOrderingMinDegree, Disconnected) {
  const std::vector<std::vector<int>> supports = {{0, 1, 2}, {10, 11, 12}};
  std::vector<std::vector<int>> cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(supports, &cliques);

  EXPECT_EQ(cliques.size(), 2u);
  VerifyStructure(ct, 2);
  VerifyRIP(ct, cliques);
  VerifyCover(cliques, supports);
}

}  // namespace conex
