// Synthetic test: small problem with multiple overlapping PSD blocks.
// Goal: reproduce the truss6/7 Gram assembly bug in a minimal setup.

#include <cstdio>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/clique_ordering.h"
#include "conex/common/clique_tree.h"
#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"

using Eigen::SparseMatrix;
using Eigen::Triplet;

void AddQ(conex::Model& p, int m) {
  SparseMatrix<double> Q(m, m);
  Q.setIdentity();
  p.AddQuadraticCost(Q);
}

// Make a sparse n×n matrix with a single symmetric entry at (i, j).
SparseMatrix<double> SymEntry(int n, int i, int j, double val) {
  SparseMatrix<double> M(n, n);
  std::vector<Triplet<double>> trips;
  trips.emplace_back(i, j, val);
  if (i != j) trips.emplace_back(j, i, val);
  M.setFromTriplets(trips.begin(), trips.end());
  return M;
}

void RunCase(const std::string& name,
             const conex::Model& problem,
             int merge_size = 5) {
  printf("=== %s (merge=%d) ===\n", name.c_str(), merge_size);
  conex::SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = merge_size;
  auto solver = conex::Solver::Build(problem, cfg);
  auto* kkt = solver.kkt();
  const int nvars = kkt->number_of_variables();
  printf("  variables: %d\n", nvars);

  auto W = kkt->MakeRowSpace();
  setOnes(W);
  kkt->SetScaling(W);
  bool ok = kkt->AssembleAndFactor();
  printf("  AssembleAndFactor: %s\n", ok ? "ok" : "FAILED");

  std::mt19937 gen(42);
  std::normal_distribution<double> dist(0.0, 1.0);
  Eigen::VectorXd x(nvars);
  for (int i = 0; i < nvars; ++i) x(i) = dist(gen);

  auto x_rhs = kkt->MakeSolverRHS();
  x_rhs = kkt->MakeBlockVariable(x);
  auto Ax = kkt->MakeRowSpace();
  kkt->MultiplyA(x_rhs, Ax);
  auto b_rhs = kkt->MakeSolverRHS();
  b_rhs.SetZero();
  kkt->AccumulateAtranspose(Ax, b_rhs);
  kkt->AccumulateQx(x_rhs, b_rhs);

  auto y_rhs = b_rhs;
  kkt->SolveSolverRHS(y_rhs);
  Eigen::VectorXd y(nvars);
  y_rhs.supernodes->GatherInto(y);

  double err = (y - x).cwiseAbs().maxCoeff();
  printf("  ||y - x||_inf = %.6e %s\n", err,
         err < 1e-8 ? "OK" : "FAIL");
  printf("\n");
}

void PrintCliqueTree(int m, int offset, int merge_size = 5) {
  // Row supports: for each variable v, the list of variables it connects to.
  // Edge (c1, c2) exists for each block b where c1 = b%m, c2 = (b+offset)%m.
  // For nb = m blocks (b = 0..m-1), the graph is a cycle of length m.
  std::vector<std::set<int>> adj(m);
  for (int b = 0; b < m; ++b) {
    int c1 = b % m, c2 = (b + offset) % m;
    adj[c1].insert(c1);
    adj[c1].insert(c2);
    adj[c2].insert(c1);
    adj[c2].insert(c2);
  }
  std::vector<std::vector<int>> row_supports(m);
  for (int v = 0; v < m; ++v)
    row_supports[v] = std::vector<int>(adj[v].begin(), adj[v].end());

  std::vector<std::vector<int>> maximal_cliques;
  conex::CliqueTree tree = conex::MakeCliqueTreeMinDegreeFromRowSupports(
      row_supports, &maximal_cliques, merge_size);

  printf("\n=== CLIQUE TREE for m=%d, offset=%d, merge=%d ===\n",
         m, offset, merge_size);
  int nc = static_cast<int>(tree.supernodes.size());
  printf("  num_cliques: %d\n", nc);
  for (int c = 0; c < nc; ++c) {
    printf("  clique %d: supernodes={", c);
    for (int v : tree.supernodes[c]) printf(" %d", v);
    printf(" }  separators={");
    for (int v : tree.separators[c]) printf(" %d", v);
    printf(" }  parent=%d\n", tree.node_to_parent[c]);
  }
  printf("  maximal_cliques (%d):\n", (int)maximal_cliques.size());
  for (size_t c = 0; c < maximal_cliques.size(); ++c) {
    printf("    MC %zu: {", c);
    for (int v : maximal_cliques[c]) printf(" %d", v);
    printf(" }\n");
  }
  printf("  RIP check: %s\n",
         tree.CheckRunningIntersectionProperty() ? "ok" : "FAILED");
}

int main() {
  PrintCliqueTree(15, 1);  // failing case (default merge=5)
  PrintCliqueTree(14, 1);  // passing case
  PrintCliqueTree(15, 1, 0);  // no merge
  PrintCliqueTree(15, 1, 100);  // aggressive merge

  // Test: same clique tree as m=15 ring, but with LINEAR constraints.
  // Variables 0..14. Two linear constraints:
  //   C1 on {9,10,11,12,13,14, 0,1,7,8}  (matches clique 0 maximal)
  //   C2 on {0,1,2,3,4,5,6,7,8}          (matches clique 1 maximal)
  // Each constraint has many rows so A^T A is invertible.
  {
    conex::Model p;
    int m = 15;
    std::vector<int> c1_vars = {9,10,11,12,13,14, 0,1,7,8};
    std::vector<int> c2_vars = {0,1,2,3,4,5,6,7,8};
    int rows = 30;  // plenty

    std::mt19937 gen(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    auto make_dense = [&](int n, int ncols) {
      Eigen::MatrixXd M(n, ncols);
      for (int i = 0; i < n; ++i)
        for (int j = 0; j < ncols; ++j) M(i, j) = dist(gen);
      return M;
    };
    auto dense_to_sparse = [](const Eigen::MatrixXd& M) {
      std::vector<Triplet<double>> trips;
      for (int i = 0; i < M.rows(); ++i)
        for (int j = 0; j < M.cols(); ++j)
          trips.emplace_back(i, j, M(i, j));
      SparseMatrix<double> S(M.rows(), M.cols());
      S.setFromTriplets(trips.begin(), trips.end());
      return S;
    };

    auto A1 = dense_to_sparse(make_dense(rows, (int)c1_vars.size()));
    auto A2 = dense_to_sparse(make_dense(rows, (int)c2_vars.size()));
    Eigen::VectorXd b1 = Eigen::VectorXd::Ones(rows);
    Eigen::VectorXd b2 = Eigen::VectorXd::Ones(rows);
    p.AddLinearConstraint(A1, b1, c1_vars);
    p.AddLinearConstraint(A2, b2, c2_vars);
    AddQ(p, m);

    // Try with default merge, to see if we get the same clique tree.
    RunCase("linear constraints, same clique support", p, 5);
  }

  return 0;

  // Test m=15 cyclic with different merge sizes.
  for (int ms : {0, 1, 2, 3, 4, 5, 10, 100}) {
    conex::Model p;
    int m = 15;
    int bsize = 2;
    std::vector<int> vars(m);
    for (int i = 0; i < m; ++i) vars[i] = i;
    for (int b = 0; b < m; ++b) {
      std::vector<SparseMatrix<double>> A_list;
      int c1 = b % m, c2 = (b + 1) % m;
      for (int i = 0; i < m; ++i) {
        if (i == c1) A_list.push_back(SymEntry(bsize, 0, 1, 1.0));
        else if (i == c2) A_list.push_back(SymEntry(bsize, 0, 0, 1.0));
        else A_list.push_back(SparseMatrix<double>(bsize, bsize));
      }
      SparseMatrix<double> B(bsize, bsize);
      p.AddPSDConstraint(A_list, B, vars, /*use_chordal=*/false);
    }
    AddQ(p, m);
    char name[64];
    snprintf(name, sizeof(name), "m=15 ring, merge=%d", ms);
    RunCase(name, p, ms);
  }
  return 0;  // skip rest

  // --- Case 1: 3 blocks of size 2, 4 constraints, each constraint touches 2 blocks ---
  {
    conex::Model p;
    std::vector<int> vars = {0, 1, 2, 3};
    int m = 4;
    int bsize = 2;

    // Block 0: constraints 0, 1 touch it
    {
      std::vector<SparseMatrix<double>> A_list;
      A_list.push_back(SymEntry(bsize, 0, 1, 1.0));  // c0
      A_list.push_back(SymEntry(bsize, 0, 0, 1.0));  // c1
      A_list.push_back(SparseMatrix<double>(bsize, bsize));  // c2 = 0
      A_list.push_back(SparseMatrix<double>(bsize, bsize));  // c3 = 0
      SparseMatrix<double> B(bsize, bsize);
      p.AddPSDConstraint(A_list, B, vars, /*use_chordal=*/false);
    }
    // Block 1: constraints 0, 2 touch it
    {
      std::vector<SparseMatrix<double>> A_list;
      A_list.push_back(SymEntry(bsize, 1, 1, 1.0));  // c0
      A_list.push_back(SparseMatrix<double>(bsize, bsize));  // c1 = 0
      A_list.push_back(SymEntry(bsize, 0, 0, 1.0));  // c2
      A_list.push_back(SparseMatrix<double>(bsize, bsize));  // c3 = 0
      SparseMatrix<double> B(bsize, bsize);
      p.AddPSDConstraint(A_list, B, vars, /*use_chordal=*/false);
    }
    // Block 2: constraints 1, 2, 3 touch it
    {
      std::vector<SparseMatrix<double>> A_list;
      A_list.push_back(SparseMatrix<double>(bsize, bsize));  // c0 = 0
      A_list.push_back(SymEntry(bsize, 0, 1, 0.5));  // c1
      A_list.push_back(SymEntry(bsize, 1, 1, 1.0));  // c2
      A_list.push_back(SymEntry(bsize, 0, 0, 1.0));  // c3
      SparseMatrix<double> B(bsize, bsize);
      p.AddPSDConstraint(A_list, B, vars, /*use_chordal=*/false);
    }

    AddQ(p, m);
    RunCase("3 PSD blocks size 2, overlapping constraints", p);
  }

  // --- Case 2: many blocks of size 2, each with 1-2 sparse constraints ---
  {
    conex::Model p;
    int num_blocks = 50;
    int m = num_blocks + 10;  // 60 constraints
    int bsize = 2;
    std::vector<int> vars(m);
    for (int i = 0; i < m; ++i) vars[i] = i;

    for (int b = 0; b < num_blocks; ++b) {
      std::vector<SparseMatrix<double>> A_list;
      for (int i = 0; i < m; ++i) {
        // Constraint b touches this block. Also constraint (b+1) % m.
        if (i == b) {
          A_list.push_back(SymEntry(bsize, 0, 1, 1.0));
        } else if (i == (b + 1) % m) {
          A_list.push_back(SymEntry(bsize, 0, 0, 1.0));
        } else {
          A_list.push_back(SparseMatrix<double>(bsize, bsize));
        }
      }
      SparseMatrix<double> B(bsize, bsize);
      p.AddPSDConstraint(A_list, B, vars, /*use_chordal=*/false);
    }
    AddQ(p, m);
    RunCase("50 PSD blocks size 2, sparse constraints", p);
  }

  // Deterministic: each block touched by constraints c1 = b%m, c2 = (b+k)%m.
  // Try different (m, num_blocks, k).
  for (int m_try : {9, 11, 13, 14}) {
    for (int k : {1, 2, 3}) {
      for (int nb_mult : {1, 2, 3, 4}) {
        int m = m_try;
        int num_blocks = nb_mult * m;
        int bsize = 2;
        std::vector<int> vars(m);
        for (int i = 0; i < m; ++i) vars[i] = i;

        conex::Model p;
        for (int b = 0; b < num_blocks; ++b) {
          std::vector<SparseMatrix<double>> A_list;
          int c1 = b % m;
          int c2 = (b + k) % m;
          for (int i = 0; i < m; ++i) {
            if (i == c1) A_list.push_back(SymEntry(bsize, 0, 1, 1.0));
            else if (i == c2 && c2 != c1) A_list.push_back(SymEntry(bsize, 0, 0, 1.0));
            else A_list.push_back(SparseMatrix<double>(bsize, bsize));
          }
          SparseMatrix<double> B(bsize, bsize);
          p.AddPSDConstraint(A_list, B, vars, /*use_chordal=*/false);
        }
        AddQ(p, m);
        char name[128];
        snprintf(name, sizeof(name),
                 "deterministic: m=%d, %d blocks, offset=%d", m, num_blocks, k);
        RunCase(name, p);
      }
    }
  }

  // (unused scan kept for backward compat)
  for (int nb : std::vector<int>{}) {
    for (int seed = 0; seed < 0; ++seed) {
      conex::Model p;
      int num_blocks = nb;
      int m = 40;
      int bsize = 2;
      std::vector<int> vars(m);
      for (int i = 0; i < m; ++i) vars[i] = i;

      std::mt19937 gen(seed);
      for (int b = 0; b < num_blocks; ++b) {
        std::vector<SparseMatrix<double>> A_list;
        int c1 = gen() % m;
        int c2 = gen() % m;
        for (int i = 0; i < m; ++i) {
          if (i == c1) A_list.push_back(SymEntry(bsize, 0, 1, 1.0));
          else if (i == c2 && c2 != c1) A_list.push_back(SymEntry(bsize, 0, 0, 1.0));
          else A_list.push_back(SparseMatrix<double>(bsize, bsize));
        }
        SparseMatrix<double> B(bsize, bsize);
        p.AddPSDConstraint(A_list, B, vars, /*use_chordal=*/false);
      }
      AddQ(p, m);
      char name[64];
      snprintf(name, sizeof(name), "%d blocks, m=40, seed=%d", nb, seed);
      RunCase(name, p);
    }
  }

  return 0;
}
