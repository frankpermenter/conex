// Test that the AMD delayed variable logic correctly interleaves
// primal and dual eliminations, and that the clique tree preserves
// this interleaving (at most one dual per supernode).
#include <cstdio>
#include <cmath>
#include <numeric>
#include <set>
#include <vector>
#include "conex/common/clique_ordering.h"
using namespace conex;

// Helper: build a simple QP+equality adjacency graph.
// Primal vars: 0..n-1.  Dual vars: n..n+p-1.
// Q edges: diagonal + off-diagonal pairs.
// C rows: each row connects a few primals to one dual.
struct TestProblem {
  int n;  // primal vars
  int p;  // equality rows (= dual vars)
  std::vector<std::vector<int>> supports;  // cliques for the clique ordering
  std::vector<int> delayed;                // dual variable indices
};

// Test 1: Ladder structure.
// n primals, n/2 duals.  Each dual i connects to primals 2i and 2i+1.
// Q = I (diagonal only).
// Expected: perfect interleaving, at most 1 dual per supernode.
TestProblem MakeLadder(int n) {
  int p = n / 2;
  TestProblem tp;
  tp.n = n;
  tp.p = p;
  // Q diagonal: each primal is a singleton clique.
  for (int i = 0; i < n; ++i)
    tp.supports.push_back({i});
  // Q off-diagonal: connect consecutive primals for fill structure.
  for (int i = 0; i + 1 < n; ++i)
    tp.supports.push_back({i, i + 1});
  // Equality rows: dual n+i connects primals 2i and 2i+1.
  for (int i = 0; i < p; ++i)
    tp.supports.push_back({2 * i, 2 * i + 1, n + i});
  // Delayed: all duals.
  for (int i = 0; i < p; ++i)
    tp.delayed.push_back(n + i);
  return tp;
}

// Test 2: Star structure.
// n primals, p duals.  All duals connect to primal 0 + one other primal.
// This creates a worst case: one hub primal adjacent to all duals.
TestProblem MakeStar(int n, int p) {
  TestProblem tp;
  tp.n = n;
  tp.p = p;
  // Q diagonal.
  for (int i = 0; i < n; ++i)
    tp.supports.push_back({i});
  // Equality rows: dual n+i connects primal 0 and primal i+1.
  for (int i = 0; i < p; ++i)
    tp.supports.push_back({0, i + 1, n + i});
  for (int i = 0; i < p; ++i)
    tp.delayed.push_back(n + i);
  return tp;
}

// Test 3: Dense structure matching CVXQP pattern.
// n primals, p duals.  Each dual connects to 3 primals.
// Multiple duals share the same primal.
TestProblem MakeDense(int n, int p) {
  TestProblem tp;
  tp.n = n;
  tp.p = p;
  // Q diagonal + some off-diagonal.
  for (int i = 0; i < n; ++i)
    tp.supports.push_back({i});
  for (int i = 0; i + 1 < n; ++i)
    tp.supports.push_back({i, i + 1});
  // Equality rows: dual n+i connects primals i%n, (2i)%n, (3i+1)%n.
  for (int i = 0; i < p; ++i) {
    std::set<int> primals;
    primals.insert(i % n);
    primals.insert((2 * i) % n);
    primals.insert((3 * i + 1) % n);
    std::vector<int> sup(primals.begin(), primals.end());
    sup.push_back(n + i);
    tp.supports.push_back(sup);
  }
  for (int i = 0; i < p; ++i)
    tp.delayed.push_back(n + i);
  return tp;
}

bool RunTest(const char* name, const TestProblem& tp) {
  printf("=== %s (n=%d, p=%d) ===\n", name, tp.n, tp.p);

  // Run the clique ordering.
  std::vector<std::vector<int>> maximal_cliques;
  CliqueTree ct = MakeCliqueTreeMinDegreeFromRowSupports(
      tp.supports, &maximal_cliques, 0,  // no merge
      SUPERNODE_REORDER_NONE, tp.delayed);

  int nc = (int)ct.supernodes.size();
  std::set<int> dset(tp.delayed.begin(), tp.delayed.end());

  // Check 1: count max duals per supernode.
  int max_duals = 0;
  int worst_node = -1;
  for (int i = 0; i < nc; ++i) {
    int nd = 0;
    for (int v : ct.supernodes[i])
      if (dset.count(v)) nd++;
    if (nd > max_duals) {
      max_duals = nd;
      worst_node = i;
    }
  }
  printf("  %d nodes, max duals per supernode = %d", nc, max_duals);
  if (worst_node >= 0 && max_duals > 1) {
    printf(" (node %d: {", worst_node);
    for (int v : ct.supernodes[worst_node])
      printf("%d%s ", v, dset.count(v) ? "*" : "");
    printf("})");
  }
  printf("\n");

  // Check 2: verify all variables appear exactly once as a supernode.
  std::set<int> all_sn;
  for (int i = 0; i < nc; ++i)
    for (int v : ct.supernodes[i])
      all_sn.insert(v);
  int total_vars = tp.n + tp.p;
  bool all_present = ((int)all_sn.size() == total_vars);
  printf("  All %d vars in supernodes: %s\n", total_vars,
         all_present ? "yes" : "NO");

  // Check 3: post-order variable sequence — count consecutive dual runs.
  int max_run = 0, cur_run = 0;
  for (int pi = 0; pi < nc; ++pi) {
    int ci = ct.post_order_position_to_clique[pi];
    for (int v : ct.supernodes[ci]) {
      if (dset.count(v)) { cur_run++; if (cur_run > max_run) max_run = cur_run; }
      else cur_run = 0;
    }
  }
  printf("  Max consecutive duals in post-order: %d\n", max_run);

  bool pass = (max_duals <= 1) && all_present;
  printf("  %s\n\n", pass ? "PASS" : "FAIL");
  return pass;
}

int main() {
  bool all_pass = true;
  all_pass &= RunTest("Ladder(10)", MakeLadder(10));
  all_pass &= RunTest("Ladder(20)", MakeLadder(20));
  all_pass &= RunTest("Star(10,5)", MakeStar(10, 5));
  all_pass &= RunTest("Star(20,10)", MakeStar(20, 10));
  all_pass &= RunTest("Dense(20,10)", MakeDense(20, 10));
  all_pass &= RunTest("Dense(100,50)", MakeDense(100, 50));
  printf("%s\n", all_pass ? "ALL PASSED" : "SOME FAILED");
  return all_pass ? 0 : 1;
}
