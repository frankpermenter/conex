#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <vector>
#include <Eigen/Dense>

using Eigen::VectorXd;

namespace {

// Original SOC: ||(x_1, ..., x_n)|| <= t.
bool FeasibleOriginal(double t, const VectorXd& x) {
  return t >= x.norm() - 1e-12;
}

// Binary tree decomposition of ||(x_1, ..., x_n)|| <= t.
// Returns auxiliary variables s and checks all 3D constraints.
//
// Tree structure: recursively split x into two halves.
//   ||(left_half)|| <= s_left
//   ||(right_half)|| <= s_right
//   ||(s_left, s_right)|| <= parent
//
// Leaf: single x_i, "norm" is |x_i|.
// Base case (2 elements): ||(x_i, x_j)|| <= parent (one 3D SOC).
// Base case (1 element): |x_i| <= parent (trivial, no SOC needed).

struct SOCNode {
  double value;          // the "norm" at this node
  int left = -1;         // child index (-1 = leaf)
  int right = -1;
  int leaf_start = -1;   // if leaf: index into x
  int leaf_count = 0;
};

struct SOCTree {
  std::vector<SOCNode> nodes;
  std::vector<std::pair<int, int>> constraints_3d;  // (parent, child_pair)

  // Build binary tree over x[start..start+count).
  // Returns node index.  Leaves are pushed first, parent after children.
  int Build(const VectorXd& x, int start, int count) {
    if (count == 1) {
      int idx = static_cast<int>(nodes.size());
      SOCNode n;
      n.value = std::abs(x(start));
      n.leaf_start = start;
      n.leaf_count = 1;
      nodes.push_back(n);
      return idx;
    }

    int half = count / 2;
    int left = Build(x, start, half);
    int right = Build(x, start + half, count - half);

    int idx = static_cast<int>(nodes.size());
    SOCNode n;
    n.left = left;
    n.right = right;
    n.value = std::sqrt(
        nodes[left].value * nodes[left].value +
        nodes[right].value * nodes[right].value);
    nodes.push_back(n);

    constraints_3d.push_back({idx, left});
    return idx;
  }

  // Check all 3D constraints: ||(left.value, right.value)|| <= parent.value.
  bool CheckAll(double t_root) const {
    // Root constraint: ||(left.value, right.value)|| <= t_root.
    int root = static_cast<int>(nodes.size()) - 1;
    // Override root value with the given t.
    double root_val = t_root;

    for (const auto& [parent, left_child] : constraints_3d) {
      int right_child = nodes[parent].right;
      double lv = nodes[left_child].value;
      double rv = nodes[right_child].value;
      double pv = (parent == root) ? root_val : nodes[parent].value;
      double norm = std::sqrt(lv * lv + rv * rv);
      if (norm > pv + 1e-12) return false;
    }
    return true;
  }

  int num_auxiliary() const {
    int count = 0;
    for (const auto& n : nodes) {
      if (n.left >= 0) ++count;  // internal node = auxiliary variable
    }
    return count - 1;  // root is t, not auxiliary
  }

  int num_3d_constraints() const {
    return static_cast<int>(constraints_3d.size());
  }
};

TEST(SOCDecomp, FeasibilityAgreement) {
  srand(42);

  for (int n : {2, 3, 4, 7, 8, 15, 16, 31}) {
    int num_agree = 0, num_disagree = 0;

    for (int trial = 0; trial < 1000; ++trial) {
      VectorXd x = VectorXd::Random(n);
      // Random t: sometimes feasible, sometimes not.
      double t = x.norm() + 0.5 * ((double)rand() / RAND_MAX - 0.5);

      bool feas_orig = FeasibleOriginal(t, x);

      SOCTree tree;
      tree.Build(x, 0, n);
      bool feas_decomp = tree.CheckAll(t);

      if (feas_orig == feas_decomp)
        num_agree++;
      else
        num_disagree++;
    }

    printf("  n=%2d: %d agree, %d disagree", n, num_agree, num_disagree);
    if (n > 1) {
      SOCTree tree;
      VectorXd dummy = VectorXd::Zero(n);
      tree.Build(dummy, 0, n);
      printf(", %d aux vars, %d 3D constraints",
             tree.num_auxiliary(), tree.num_3d_constraints());
    }
    printf("\n");
    EXPECT_EQ(num_disagree, 0);
  }
}

// Verify that the decomposition is tight: for a point ON the boundary
// of the original SOC (t = ||x||), all 3D constraints are also tight.
TEST(SOCDecomp, BoundaryTightness) {
  srand(99);
  const int n = 8;

  for (int trial = 0; trial < 100; ++trial) {
    VectorXd x = VectorXd::Random(n);
    double t = x.norm();  // exactly on boundary

    SOCTree tree;
    tree.Build(x, 0, n);

    // Root constraint should be tight.
    int root = static_cast<int>(tree.nodes.size()) - 1;
    int left = tree.nodes[root].left;
    int right = tree.nodes[root].right;
    double lv = tree.nodes[left].value;
    double rv = tree.nodes[right].value;
    double root_norm = std::sqrt(lv * lv + rv * rv);
    EXPECT_NEAR(root_norm, t, 1e-12);

    // All internal constraints should be tight (auxiliary = exact norm).
    EXPECT_TRUE(tree.CheckAll(t));
  }
}

}  // namespace
