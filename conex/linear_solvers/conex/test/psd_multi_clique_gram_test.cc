// Regression tests for a bug in the PSD Gram assembly when the clique tree
// has more than one clique.
//
// The bug: A^T·A computed via the KKT solver's factor-and-solve disagrees
// with A^T·A computed via the MultiplyA / AccumulateAtranspose primitives,
// for PSD constraints whose constraint graph produces a multi-clique tree.
//
// Reproducer: 15 PSD blocks of size 2 on 15 variables, with a cyclic
// overlap pattern (each block b touched by constraints c1 = b%15 and
// c2 = (b+1)%15). This triggers a 2-clique tree with a nontrivial
// separator under the default max_merge_supernode_size=5.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

namespace conex {
namespace {

using Eigen::SparseMatrix;
using Eigen::Triplet;

// Sparse n×n matrix with one symmetric entry at (i, j).
SparseMatrix<double> SymEntry(int n, int i, int j, double val) {
  SparseMatrix<double> M(n, n);
  std::vector<Triplet<double>> trips;
  trips.emplace_back(i, j, val);
  if (i != j) trips.emplace_back(j, i, val);
  M.setFromTriplets(trips.begin(), trips.end());
  return M;
}

// Build a problem with m variables and m PSD blocks of size 2, where
// block b is touched by constraints c1 = b%m (off-diagonal entry) and
// c2 = (b+1)%m (diagonal entry). Adds Q = I as a regularizer.
Problem MakeCyclicRingPSD(int m) {
  Problem p;
  std::vector<int> vars(m);
  for (int i = 0; i < m; ++i) vars[i] = i;
  int bsize = 2;
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
  SparseMatrix<double> Q(m, m);
  Q.setIdentity();
  p.AddQuadraticCost(Q);
  return p;
}

// Round-trip test: sample random x, compute b = (A^T A + I) x via the
// KKT solver's primitives, solve G y = b, verify y ≈ x.
double GramRoundTripError(const Problem& p, int seed,
                          const SolverConfiguration& cfg) {
  auto solver = Solver::Build(p, cfg);
  auto* kkt = solver.solver();
  const int nvars = kkt->number_of_variables();

  auto W = kkt->MakeRowSpace();
  setOnes(W);
  kkt->SetScaling(W);
  EXPECT_TRUE(kkt->AssembleAndFactor());

  std::srand(seed);
  Eigen::VectorXd x = Eigen::VectorXd::Random(nvars);

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
  return (y - x).cwiseAbs().maxCoeff();
}

// Regression test for the PSD multi-clique Gram assembly bug.
// The problem has a 2-clique tree under default merge size.
// Without the bug, the round-trip error should be ~1e-15.
TEST(PSDMultiCliqueGram, CyclicRing15RoundTrip) {
  Problem p = MakeCyclicRingPSD(15);
  SolverConfiguration cfg;  // default merge=5 → 2 cliques
  double err = GramRoundTripError(p, 42, cfg);
  EXPECT_LT(err, 1e-10) << "Gram round-trip error too large: " << err;
}

// Sanity: the single-clique path (forced by large merge) is correct.
TEST(PSDMultiCliqueGram, CyclicRing15SingleClique) {
  Problem p = MakeCyclicRingPSD(15);
  SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = 100000;  // force single clique
  double err = GramRoundTripError(p, 42, cfg);
  EXPECT_LT(err, 1e-10) << "Single-clique error: " << err;
}

// Sanity: smaller m=14 doesn't trigger the bug under default merge.
TEST(PSDMultiCliqueGram, CyclicRing14RoundTrip) {
  Problem p = MakeCyclicRingPSD(14);
  SolverConfiguration cfg;
  double err = GramRoundTripError(p, 42, cfg);
  EXPECT_LT(err, 1e-10) << "m=14 error: " << err;
}

}  // namespace
}  // namespace conex
