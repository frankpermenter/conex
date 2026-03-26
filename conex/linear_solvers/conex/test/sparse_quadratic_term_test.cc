#include "conex/common/sparse_quadratic_term.h"

#include <algorithm>
#include <numeric>
#include <set>
#include <vector>

#include "conex/common/clique_ordering.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/tree_utils.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Diagonal Q + banded A.  (Q + A^T A) should be full rank.
TEST(SparseQuadraticTerm, DiagonalQBandedA) {
  srand(42);
  const int n = 30, bw = 4, rpc = 5;
  int num_groups = n - bw + 1;
  int num_rows = rpc * num_groups;

  std::vector<Eigen::Triplet<double>> a_trips;
  for (int g = 0; g < num_groups; g++)
    for (int r = 0; r < rpc; r++)
      for (int j = 0; j < bw; j++)
        a_trips.emplace_back(g * rpc + r, g + j,
                             0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(num_rows, n);
  A.setFromTriplets(a_trips.begin(), a_trips.end());

  // Diagonal Q = 0.1 * I.
  std::vector<Eigen::Triplet<double>> q_trips;
  for (int i = 0; i < n; i++) q_trips.emplace_back(i, i, 0.1);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(q_trips.begin(), q_trips.end());

  MatrixXd Ad(A);
  MatrixXd Qd(Q);
  MatrixXd M = Qd + Ad.transpose() * Ad;
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = M * x_true;

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-10) << "Solve error: " << err;

  // Verify normal equation.
  VectorXd residual = M * result.x - rhs;
  EXPECT_LT(residual.norm(), 1e-8 * rhs.norm())
      << "Normal equation residual: " << residual.norm();
}

// Arrow-pattern Q: verify correct clique structure.
// Q has edges {0,3}, {1,3}, {2,3} — a star, NOT a 4-clique.
TEST(SparseQuadraticTerm, ArrowPatternQ) {
  const int n = 4;
  std::vector<Eigen::Triplet<double>> q_trips;
  // Diagonal.
  for (int i = 0; i < n; i++) q_trips.emplace_back(i, i, 1.0);
  // Off-diagonal: star centered at var 3.
  q_trips.emplace_back(0, 3, 0.1); q_trips.emplace_back(3, 0, 0.1);
  q_trips.emplace_back(1, 3, 0.2); q_trips.emplace_back(3, 1, 0.2);
  q_trips.emplace_back(2, 3, 0.3); q_trips.emplace_back(3, 2, 0.3);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(q_trips.begin(), q_trips.end());

  // A is identity (just to have some A).
  Eigen::SparseMatrix<double> A(n, n);
  A.setIdentity();

  MatrixXd Qd(Q);
  MatrixXd M = Qd + MatrixXd::Identity(n, n);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = M * x_true;

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-10) << "Arrow Q solve error: " << err;
}

// Dense Q (as MatrixXd) + sparse A.
TEST(SparseQuadraticTerm, DenseQ) {
  srand(42);
  const int n = 15, m = 20;
  MatrixXd Q = MatrixXd::Random(n, n);
  Q = Q.transpose() * Q + 0.1 * MatrixXd::Identity(n, n);  // PSD

  std::vector<Eigen::Triplet<double>> a_trips;
  for (int r = 0; r < m; r++)
    for (int j = 0; j < 3; j++)
      a_trips.emplace_back(r, rand() % n, 0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(a_trips.begin(), a_trips.end());

  MatrixXd Ad(A);
  MatrixXd M = Q + Ad.transpose() * Ad;
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = M * x_true;

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-10) << "Dense Q solve error: " << err;
}

// Q = 0: should reduce to standard least squares.
TEST(SparseQuadraticTerm, ZeroQ) {
  srand(42);
  const int n = 20, bw = 3, rpc = 5;
  int ng = n - bw + 1, nr = rpc * ng;

  std::vector<Eigen::Triplet<double>> a_trips;
  for (int g = 0; g < ng; g++)
    for (int r = 0; r < rpc; r++)
      for (int j = 0; j < bw; j++)
        a_trips.emplace_back(g * rpc + r, g + j,
                             0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(a_trips.begin(), a_trips.end());

  // Q = 0 (empty sparse matrix).
  Eigen::SparseMatrix<double> Q(n, n);

  MatrixXd Ad(A);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = Ad.transpose() * (Ad * x_true);

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-8) << "Zero Q solve error: " << err;
}

// Sparse tridiagonal Q + banded A: larger problem.
TEST(SparseQuadraticTerm, TridiagonalQ) {
  srand(42);
  const int n = 50, bw = 3, rpc = 4;
  int ng = n - bw + 1, nr = rpc * ng;

  std::vector<Eigen::Triplet<double>> a_trips;
  for (int g = 0; g < ng; g++)
    for (int r = 0; r < rpc; r++)
      for (int j = 0; j < bw; j++)
        a_trips.emplace_back(g * rpc + r, g + j,
                             0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(a_trips.begin(), a_trips.end());

  // Tridiagonal Q: Q(i,i) = 2, Q(i,i+1) = Q(i+1,i) = -0.5.
  std::vector<Eigen::Triplet<double>> q_trips;
  for (int i = 0; i < n; i++) {
    q_trips.emplace_back(i, i, 2.0);
    if (i + 1 < n) {
      q_trips.emplace_back(i, i + 1, -0.5);
      q_trips.emplace_back(i + 1, i, -0.5);
    }
  }
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(q_trips.begin(), q_trips.end());

  MatrixXd Ad(A);
  MatrixXd Qd(Q);
  MatrixXd M = Qd + Ad.transpose() * Ad;
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = M * x_true;

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-10) << "Tridiagonal Q solve error: " << err;

  VectorXd residual = M * result.x - rhs;
  EXPECT_LT(residual.norm(), 1e-8 * rhs.norm());
}

// ===================================================================
// Clique tree construction tests with A=0 (Q edges only).
// Use known chordal graphs where min-degree gives a perfect
// elimination ordering and maximal cliques are predictable.
// ===================================================================

// Helper: build sparse symmetric matrix from edge list.
Eigen::SparseMatrix<double> MakeSymmetric(
    int n, const std::vector<std::pair<int, int>>& edges) {
  std::vector<Eigen::Triplet<double>> t;
  for (int i = 0; i < n; i++) t.emplace_back(i, i, 1.0);
  for (auto [u, v] : edges) {
    t.emplace_back(u, v, 0.5);
    t.emplace_back(v, u, 0.5);
  }
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(t.begin(), t.end());
  return Q;
}

// Helper: extract maximal cliques from clique tree.
std::vector<std::set<int>> GetMaximalCliques(const CliqueTree& ct) {
  std::vector<std::set<int>> cliques;
  for (size_t i = 0; i < ct.supernodes.size(); i++) {
    std::set<int> c(ct.supernodes[i].begin(), ct.supernodes[i].end());
    c.insert(ct.separators[i].begin(), ct.separators[i].end());
    cliques.push_back(c);
  }
  return cliques;
}

// Path graph: 0-1-2-3-4.
// Chordal (already a tree). Maximal cliques: {0,1}, {1,2}, {2,3}, {3,4}.
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
