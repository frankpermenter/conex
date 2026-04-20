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
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/tree_solver/kkt_tree_solver.h"

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
Model MakeCyclicRingPSD(int m) {
  Model p;
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
double GramRoundTripError(const Model& p, int seed,
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

Model MakeCyclicRingLinearMulti() {
  std::vector<int> var1{0,3,4};
  std::vector<int> var2{0,2,3};
  std::vector<int> var3{0,1,2};
  Eigen::SparseMatrix<double> A1 = Eigen::MatrixXd::Random(7, var1.size()).sparseView();
  Eigen::SparseMatrix<double> A2 = Eigen::MatrixXd::Random(7, var2.size()).sparseView();
  Eigen::SparseMatrix<double> A3 = Eigen::MatrixXd::Random(7, var3.size()).sparseView();
  Model p;
  p.AddLinearConstraint(A1, Eigen::VectorXd::Ones(A1.rows()), var1);
  p.AddLinearConstraint(A2, Eigen::VectorXd::Ones(A2.rows()), var2);
  p.AddLinearConstraint(A3, Eigen::VectorXd::Ones(A3.rows()), var3);
  // Add Q=I for full rank.
  Eigen::SparseMatrix<double> Q(5, 5);
  Q.setIdentity();
  p.AddQuadraticCost(Q);
  return p;
}

// Equivalent to MakeCyclicRingLinear but combined into a single
// AddLinearConstraint call with one big sparse matrix.  Each sub-block
// (A1, A2, A3) is placed into the appropriate columns of the big A.
// This avoids the multi-constraint multi-clique assembly path entirely.
Model MakeCyclicRingLinearSingle() {
  std::vector<int> var1{0,3,4};
  std::vector<int> var2{0,2,3};
  std::vector<int> var3{0,1,2};
  // Use the SAME random data as MakeCyclicRingLinear (same srand).
  Eigen::MatrixXd A1d = Eigen::MatrixXd::Random(7, var1.size());
  Eigen::MatrixXd A2d = Eigen::MatrixXd::Random(7, var2.size());
  Eigen::MatrixXd A3d = Eigen::MatrixXd::Random(7, var3.size());

  // Build big A: 21 rows × 5 columns (vars 0..4).
  std::vector<Eigen::Triplet<double>> trips;
  auto add_block = [&](int row_offset, const Eigen::MatrixXd& A,
                       const std::vector<int>& vars) {
    for (int i = 0; i < A.rows(); ++i)
      for (int j = 0; j < A.cols(); ++j)
        trips.emplace_back(row_offset + i, vars[j], A(i, j));
  };
  add_block(0, A1d, var1);
  add_block(7, A2d, var2);
  add_block(14, A3d, var3);
  Eigen::SparseMatrix<double> Abig(21, 5);
  Abig.setFromTriplets(trips.begin(), trips.end());

  std::vector<int> all_vars{0, 1, 2, 3, 4};
  Model p;
  p.AddLinearConstraint(Abig, Eigen::VectorXd::Ones(Abig.rows()), all_vars);
  return p;
}


std::vector<Eigen::SparseMatrix<double>> RandomLMI(int n, int m) {
  std::vector<Eigen::SparseMatrix<double>> a1;
  for (int i = 0; i < m; i++){
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(n, n);
    A =  Eigen::MatrixXd(A + A.transpose());
    a1.push_back(A.sparseView());
  }
  return a1;
}

Model MakeCyclicRingPSDExplicit() {
  std::vector<int> var1{0,8,9,10,11,12,13,14};
  std::vector<int> var2{0,2,3,4,5,6,7,8};
  std::vector<int> var3{0,1,2};

  int n = 3;
  auto A1 = RandomLMI(n, var1.size()); 
  auto A2 = RandomLMI(n, var2.size()); 
  auto A3 = RandomLMI(n, var3.size()); 
  Model p;
  p.AddPSDConstraint(A1, Eigen::MatrixXd::Identity(n,n).sparseView(), var1);
  p.AddPSDConstraint(A2, Eigen::MatrixXd::Identity(n,n).sparseView(), var2);
  p.AddPSDConstraint(A3, Eigen::MatrixXd::Identity(n,n).sparseView(), var3);
  return p;
}




// Test that AddLinearConstraint(A, b, vars) actually uses vars to remap
// to the bigger Gram matrix. A is a 4x2 dense block, vars selects {1, 3}
// out of 5 total variables. Round-trip through (Q + A^T A) should give x.
TEST(LinearVarsRemap, RoundTrip) {
  Model p;
  std::vector<int> vars = {1, 3};
  Eigen::MatrixXd A_dense(4, 2);
  A_dense << 1, 2,  3, 4,  5, 6,  7, 8;
  Eigen::SparseMatrix<double> A = A_dense.sparseView();
  Eigen::VectorXd b = Eigen::VectorXd::Ones(4);
  p.AddLinearConstraint(A, b, vars);
  Eigen::SparseMatrix<double> Q(5, 5);
  Q.setIdentity();
  p.AddQuadraticCost(Q);
  SolverConfiguration cfg;
  double err = GramRoundTripError(p, 42, cfg);
  EXPECT_LT(err, 1e-10) << "vars-remap round-trip error: " << err;
}

// Regression test for the PSD multi-clique Gram assembly bug.
// The problem has a 2-clique tree under default merge size.
// Without the bug, the round-trip error should be ~1e-15.
TEST(PSDMultiCliqueGram, CyclicRing15RoundTrip) {
  Model p = MakeCyclicRingPSD(5);
  //Model p = MakeCyclicRingPSDExplicit();
  SolverConfiguration cfg;  // default merge=5 → 2 cliques
  cfg.tree.max_merge_supernode_size = 0;
  double err = GramRoundTripError(p, 42, cfg);
  EXPECT_LT(err, 1e-10) << "Gram round-trip error too large: " << err;
}

TEST(PSDMultiCliqueGram, CyclicRingLinearSingle) {
  Model p = MakeCyclicRingLinearSingle();
  SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = 0;
  double err = GramRoundTripError(p, 42, cfg);
  EXPECT_LT(err, 1e-10) << "single-AddLinearConstraint err: " << err;
}

TEST(PSDMultiCliqueGram, CyclicRingLinearMulti) {
  Model p = MakeCyclicRingLinearMulti();
  SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = 0;
  double err = GramRoundTripError(p, 42, cfg);
  EXPECT_LT(err, 1e-10) << "single-AddLinearConstraint err: " << err;
}

// Diagnostic: compute b = G_true*x (G built manually) and call dense Solve.
TEST(PSDMultiCliqueGram, DenseRoundTripMulti) {
  // Reproduce the random data from MakeCyclicRingLinearMulti.
  std::srand(1);  // Eigen::MatrixXd::Random uses its own seed; reproduce here
  std::vector<std::vector<int>> all_vars{{0,3,4},{0,2,3},{0,1,2}};
  std::vector<Eigen::MatrixXd> A_list;
  for (size_t i = 0; i < 3; ++i) {
    A_list.push_back(Eigen::MatrixXd::Random(7, all_vars[i].size()));
  }
  // Build true G = Q + sum_i A_i^T A_i in variable space, where vars_i remap.
  const int n = 5;
  Eigen::MatrixXd G_true = Eigen::MatrixXd::Identity(n, n);  // Q = I
  for (size_t k = 0; k < 3; ++k) {
    Eigen::MatrixXd ATA = A_list[k].transpose() * A_list[k];
    for (size_t i = 0; i < all_vars[k].size(); ++i)
      for (size_t j = 0; j < all_vars[k].size(); ++j)
        G_true(all_vars[k][i], all_vars[k][j]) += ATA(i, j);
  }

  // NOTE: the Model regenerates Random independently — we can't control it
  // exactly without refactoring. So just compare the assembled K (post-factor
  // is wrong; pre-factor would be ideal). Skip strict G_true check and
  // just verify Solve(G_assembled * x) = x. This still tells us if the
  // factor inverts what KKTMatrix returns.
  Model p = MakeCyclicRingLinearMulti();
  SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = 0;
  auto solver = Solver::Build(p, cfg);
  auto* kkt = solver.solver();

  // Get the Gram BEFORE factor by calling Assemble (not AssembleAndFactor).
  kkt->Assemble();
  Eigen::MatrixXd G_pre = kkt->KKTMatrix(false);
  G_pre = G_pre.selfadjointView<Eigen::Lower>();

  ASSERT_TRUE(kkt->AssembleAndFactor());

  std::srand(42);
  Eigen::VectorXd x = Eigen::VectorXd::Random(n);
  Eigen::VectorXd b = G_pre * x;
  Eigen::VectorXd y = kkt->Solve(b);
  double err = (y - x).cwiseAbs().maxCoeff();
  std::cerr << "G_pre=\n" << G_pre << "\n";
  std::cerr << "Dense round-trip err = " << err << "\n";
  EXPECT_LT(err, 1e-10);
}

// Diagnostic: compare assembled Gram (DoKKTMatrix) vs primitive-computed Gram.
TEST(PSDMultiCliqueGram, CompareAssembledVsPrimitives) {
  Model p = MakeCyclicRingLinearMulti();
  SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = 0;
  auto solver = Solver::Build(p, cfg);
  auto* kkt = solver.solver();
  const int n = kkt->number_of_variables();

  auto W = kkt->MakeRowSpace();
  setOnes(W);
  kkt->SetScaling(W);
  // Capture true (pre-factor) Gram.
  kkt->Assemble();
  Eigen::MatrixXd G_true = kkt->KKTMatrix(false);
  G_true = G_true.selfadjointView<Eigen::Lower>();
  ASSERT_TRUE(kkt->AssembleAndFactor());

  // Build G via primitives column by column: G(:,j) = (A^T A + Q) e_j.
  Eigen::MatrixXd G_prim(n, n);
  for (int j = 0; j < n; ++j) {
    Eigen::VectorXd ej = Eigen::VectorXd::Zero(n);
    ej(j) = 1.0;
    auto x_rhs = kkt->MakeSolverRHS();
    x_rhs = kkt->MakeBlockVariable(ej);
    auto Ax = kkt->MakeRowSpace();
    kkt->MultiplyA(x_rhs, Ax);
    auto b_rhs = kkt->MakeSolverRHS();
    b_rhs.SetZero();
    kkt->AccumulateAtranspose(Ax, b_rhs);
    kkt->AccumulateQx(x_rhs, b_rhs);
    // Fold separator contributions back into supernodes before gathering.
    auto* tree = dynamic_cast<SymmetricLinearSystemTreeSolver*>(kkt);
    ASSERT_TRUE(tree != nullptr);
    tree->GatherSeparators(b_rhs);
    Eigen::VectorXd col(n);
    b_rhs.supernodes->GatherInto(col);
    G_prim.col(j) = col;
  }

  // Get the assembled KKT matrix (in original variable order).
  Eigen::MatrixXd G_asm = kkt->KKTMatrix();
  // Symmetrize lower → full (DoKKTMatrix may only fill lower triangle).
  G_asm = G_asm.selfadjointView<Eigen::Lower>();

  std::cerr << "\nG_true (correct):\n" << G_true << "\n";
  std::cerr << "\nG_primitives:\n" << G_prim << "\n";
  std::cerr << "\nDiff (G_prim - G_true):\n" << (G_prim - G_true) << "\n";
  std::cerr << "\nMax |G_prim - G_true| = "
            << (G_prim - G_true).cwiseAbs().maxCoeff() << "\n";
}



// Sanity: the single-clique path (forced by large merge) is correct.
TEST(PSDMultiCliqueGram, CyclicRing15SingleClique) {
  Model p = MakeCyclicRingPSD(15);
  SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = 100000;  // force single clique
  double err = GramRoundTripError(p, 42, cfg);
  EXPECT_LT(err, 1e-10) << "Single-clique error: " << err;
}

// Sanity: smaller m=14 doesn't trigger the bug under default merge.
TEST(PSDMultiCliqueGram, CyclicRing14RoundTrip) {
  Model p = MakeCyclicRingPSD(14);
  SolverConfiguration cfg;
  double err = GramRoundTripError(p, 42, cfg);
  EXPECT_LT(err, 1e-10) << "m=14 error: " << err;
}

}  // namespace
}  // namespace conex
