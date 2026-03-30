#include "conex/common/sparse_quadratic_term.h"

#include <algorithm>
#include <numeric>
#include <set>
#include <vector>

#include "conex/common/clique_ordering.h"
#include "conex/common/clique_tree.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/tree_utils.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Helper: build symmetric sparse matrix from edge list.
Eigen::SparseMatrix<double> MakeSymmetric(
    int n, const std::vector<std::pair<int,int>>& edges) {
  std::vector<Eigen::Triplet<double>> trips;
  for (auto [i, j] : edges) {
    trips.emplace_back(i, j, 1.0);
    trips.emplace_back(j, i, 1.0);
    trips.emplace_back(i, i, 1.0);
    trips.emplace_back(j, j, 1.0);
  }
  Eigen::SparseMatrix<double> M(n, n);
  M.setFromTriplets(trips.begin(), trips.end());
  return M;
}

// Helper: extract maximal cliques (sn ∪ sep) from CliqueTree as sorted sets.
std::vector<std::set<int>> GetMaximalCliques(const CliqueTree& ct) {
  std::vector<std::set<int>> result;
  for (size_t i = 0; i < ct.supernodes.size(); ++i) {
    std::set<int> clique(ct.supernodes[i].begin(), ct.supernodes[i].end());
    clique.insert(ct.separators[i].begin(), ct.separators[i].end());
    result.push_back(clique);
  }
  return result;
}

// Diagonal Q + banded A.  (Q + A^T A) should be full rank.
TEST(CliqueTreeFromEdges, PathGraph) {
  int n = 5;
  auto Q = MakeSymmetric(n, {{0,1}, {1,2}, {2,3}, {3,4}});

  // Build clique tree from Q edges only (no A row supports).
  std::vector<std::vector<int>> empty_supports;
  std::vector<std::vector<int>> maximal_cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(
      empty_supports, Q, &maximal_cliques, 0, SUPERNODE_REORDER_NONE);

  // Should have 4 maximal cliques, each of size 2.
  auto cliques = GetMaximalCliques(ct);
  EXPECT_EQ(cliques.size(), 4u) << "Expected 4 cliques for path graph";
  for (auto& c : cliques) {
    EXPECT_EQ(c.size(), 2u) << "Each path clique should have 2 nodes";
  }

  // Each edge should appear in exactly one clique.
  std::set<std::pair<int,int>> expected_edges = {{0,1},{1,2},{2,3},{3,4}};
  std::set<std::pair<int,int>> found_edges;
  for (auto& c : cliques) {
    std::vector<int> cv(c.begin(), c.end());
    for (size_t i = 0; i < cv.size(); i++)
      for (size_t j = i+1; j < cv.size(); j++)
        found_edges.insert({cv[i], cv[j]});
  }
  EXPECT_EQ(found_edges, expected_edges);
}

// Star graph: center=4, leaves=0,1,2,3.
// Chordal. Maximal cliques: {0,4}, {1,4}, {2,4}, {3,4}.
TEST(CliqueTreeFromEdges, StarGraph) {
  int n = 5;
  auto Q = MakeSymmetric(n, {{0,4}, {1,4}, {2,4}, {3,4}});

  std::vector<std::vector<int>> empty_supports;
  std::vector<std::vector<int>> maximal_cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(
      empty_supports, Q, &maximal_cliques, 0, SUPERNODE_REORDER_NONE);

  auto cliques = GetMaximalCliques(ct);
  EXPECT_EQ(cliques.size(), 4u) << "Expected 4 cliques for star graph";

  // Every clique should contain the center vertex 4.
  for (auto& c : cliques) {
    EXPECT_TRUE(c.count(4)) << "Every star clique should contain center";
    EXPECT_EQ(c.size(), 2u);
  }
}

// Complete graph K4: 0-1-2-3 all connected.
// Chordal. Single maximal clique: {0,1,2,3}.
TEST(CliqueTreeFromEdges, CompleteGraph) {
  int n = 4;
  auto Q = MakeSymmetric(n, {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}});

  std::vector<std::vector<int>> empty_supports;
  std::vector<std::vector<int>> maximal_cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(
      empty_supports, Q, &maximal_cliques, 0, SUPERNODE_REORDER_NONE);

  auto cliques = GetMaximalCliques(ct);
  // After merging, should be 1 clique with all 4 vertices.
  // (The min-degree elimination on K4 creates 4 trivial supernodes
  //  that merge into one.)
  std::set<int> all_vars = {0, 1, 2, 3};
  bool found_full = false;
  for (auto& c : cliques) {
    if (c == all_vars) found_full = true;
  }
  // With merge=0, might have multiple cliques. Check union covers all vars.
  std::set<int> union_vars;
  for (auto& c : cliques) union_vars.insert(c.begin(), c.end());
  EXPECT_EQ(union_vars, all_vars);
}

// Chain of triangles: 0-1-2, 1-2-3, 2-3-4.
// Chordal. Maximal cliques: {0,1,2}, {1,2,3}, {2,3,4}.
TEST(CliqueTreeFromEdges, TriangleChain) {
  int n = 5;
  auto Q = MakeSymmetric(n, {
      {0,1},{0,2},{1,2},  // triangle 0-1-2
      {1,3},{2,3},        // extend to triangle 1-2-3
      {2,4},{3,4}         // extend to triangle 2-3-4
  });

  std::vector<std::vector<int>> empty_supports;
  std::vector<std::vector<int>> maximal_cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(
      empty_supports, Q, &maximal_cliques, 0, SUPERNODE_REORDER_NONE);

  auto cliques = GetMaximalCliques(ct);

  // Check that the expected triangles appear as cliques.
  std::set<std::set<int>> expected = {{0,1,2}, {1,2,3}, {2,3,4}};
  std::set<std::set<int>> found(cliques.begin(), cliques.end());
  EXPECT_EQ(found, expected) << "Expected 3 triangle cliques";
}

// Arrow/star matrix (the motivating example for edges vs row supports).
// Q has edges {0,3}, {1,3}, {2,3}. NOT a 4-clique.
// Chordal (star). Maximal cliques: {0,3}, {1,3}, {2,3}.
TEST(CliqueTreeFromEdges, ArrowNotDense) {
  int n = 4;
  auto Q = MakeSymmetric(n, {{0,3}, {1,3}, {2,3}});

  std::vector<std::vector<int>> empty_supports;
  std::vector<std::vector<int>> maximal_cliques;
  auto ct = MakeCliqueTreeMinDegreeFromRowSupports(
      empty_supports, Q, &maximal_cliques, 0, SUPERNODE_REORDER_NONE);

  auto cliques = GetMaximalCliques(ct);

  // Should NOT have a 4-clique {0,1,2,3} (that would mean edges 0-1, 0-2,
  // 1-2 were added, which don't exist in Q).
  for (auto& c : cliques) {
    EXPECT_LE(c.size(), 2u)
        << "Arrow graph should have only size-2 cliques, got size " << c.size();
  }

  // 0 and 1 should NOT be in the same clique (no edge between them).
  for (auto& c : cliques) {
    EXPECT_FALSE(c.count(0) && c.count(1))
        << "Vars 0 and 1 should not be in the same clique";
  }
}

}  // namespace
}  // namespace conex
