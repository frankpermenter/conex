#include "conex/sparse_linear_constraint.h"

#include <chrono>
#include <iostream>
#include <set>

#include "conex/clique_ordering.h"
#include "conex/cone_program.h"
#include "conex/linear_constraint.h"
#include "conex/test/default_solver_config.h"
#include "conex/tree_utils.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Build a block-diagonal sparse matrix from dense blocks.
Eigen::SparseMatrix<double> BlockDiagonal(
    const std::vector<MatrixXd>& blocks) {
  int total_rows = 0, total_cols = 0;
  for (const auto& B : blocks) {
    total_rows += B.rows();
    total_cols += B.cols();
  }
  Eigen::SparseMatrix<double> A(total_rows, total_cols);
  std::vector<Eigen::Triplet<double>> triplets;
  int row_offset = 0, col_offset = 0;
  for (const auto& B : blocks) {
    for (int i = 0; i < B.rows(); i++) {
      for (int j = 0; j < B.cols(); j++) {
        if (B(i, j) != 0.0) {
          triplets.emplace_back(row_offset + i, col_offset + j, B(i, j));
        }
      }
    }
    row_offset += B.rows();
    col_offset += B.cols();
  }
  A.setFromTriplets(triplets.begin(), triplets.end());
  return A;
}

GTEST_TEST(SparseLinearConstraint, Decomposition) {
  // 3 blocks of different sizes.
  std::vector<MatrixXd> blocks = {
      MatrixXd::Random(4, 2), MatrixXd::Random(3, 3), MatrixXd::Random(5, 2)};
  auto A = BlockDiagonal(blocks);
  VectorXd b = VectorXd::Ones(A.rows());

  SparseLinearConstraint slc(A, b);
  // Disjoint supports => no containment merging => 3 groups.
  EXPECT_EQ(slc.num_groups(), 3);

  // Total rows across all groups should equal A.rows().
  int total_rows = 0;
  for (const auto& g : slc.groups()) {
    total_rows += g.A.rows();
  }
  EXPECT_EQ(total_rows, A.rows());
}

// Test that rows with subset supports are merged into a single group.
GTEST_TEST(SparseLinearConstraint, ContainmentMerging) {
  // Row 0: nonzeros in columns {0, 1, 2}
  // Row 1: nonzeros in columns {0, 1}       (subset of row 0)
  // Row 2: nonzeros in columns {1, 2}       (subset of row 0)
  // Row 3: nonzeros in columns {3, 4}       (disjoint)
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.emplace_back(0, 0, 1.0);
  triplets.emplace_back(0, 1, 2.0);
  triplets.emplace_back(0, 2, 3.0);
  triplets.emplace_back(1, 0, 4.0);
  triplets.emplace_back(1, 1, 5.0);
  triplets.emplace_back(2, 1, 6.0);
  triplets.emplace_back(2, 2, 7.0);
  triplets.emplace_back(3, 3, 8.0);
  triplets.emplace_back(3, 4, 9.0);

  Eigen::SparseMatrix<double> A(4, 5);
  A.setFromTriplets(triplets.begin(), triplets.end());
  VectorXd b(4);
  b << 10, 11, 12, 13;

  SparseLinearConstraint slc(A, b);
  // Rows 0,1,2 should merge (supp({0,1}) ⊆ supp({0,1,2}), etc.)
  // Row 3 is disjoint => separate group.
  EXPECT_EQ(slc.num_groups(), 2);

  // Find the group with 3 rows.
  const auto& groups = slc.groups();
  bool found_big = false, found_small = false;
  for (const auto& g : groups) {
    if (g.A.rows() == 3) {
      EXPECT_EQ(g.A.cols(), 3);  // support {0,1,2}
      found_big = true;
    }
    if (g.A.rows() == 1) {
      EXPECT_EQ(g.A.cols(), 2);  // support {3,4}
      found_small = true;
    }
  }
  EXPECT_TRUE(found_big);
  EXPECT_TRUE(found_small);
}

// Solve a block-diagonal LP using SparseLinearConstraint and verify
// that the solution matches a dense solve.
GTEST_TEST(SparseLinearConstraint, SolveBlockDiagonal) {
  srand(42);
  double eps = 1e-8;

  int num_blocks = 4;
  int rows_per_block = 6;
  int cols_per_block = 3;

  std::vector<MatrixXd> blocks(num_blocks);
  for (int i = 0; i < num_blocks; i++) {
    blocks[i] = MatrixXd::Random(rows_per_block, cols_per_block);
  }

  auto A_sparse = BlockDiagonal(blocks);
  int num_rows = A_sparse.rows();
  int num_vars = A_sparse.cols();

  VectorXd b_affine = VectorXd::Ones(num_rows);

  // Build a feasible cost vector.
  VectorXd x0 = VectorXd::Random(num_rows).cwiseAbs() * 0.01;
  MatrixXd A_dense(A_sparse);
  VectorXd cost = A_dense.transpose() * x0;

  // Solve with SparseLinearConstraint (multiple sub-constraints).
  SolverConfiguration config = DefaultTestConfiguration();
  config.prepare_dual_variables = true;
  config.inv_sqrt_mu_max = 5e3;
  config.final_centering_tolerance = 1.01;
  config.final_centering_steps = 0;

  Program prog_sparse(num_vars);
  SparseLinearConstraint slc(A_sparse, b_affine);
  auto ids = slc.AddToProgram(prog_sparse);
  EXPECT_EQ(static_cast<int>(ids.size()), num_blocks);

  VectorXd y_sparse(num_vars);
  Solve(cost, prog_sparse, config, y_sparse.data());

  // Verify KKT conditions per group.
  VectorXd Atx = VectorXd::Zero(num_vars);
  for (int i = 0; i < num_blocks; i++) {
    const auto& group = slc.groups()[i];
    VectorXd y_sub(group.variables.size());
    for (int j = 0; j < static_cast<int>(group.variables.size()); j++) {
      y_sub(j) = y_sparse(group.variables[j]);
    }

    VectorXd slack = group.b - group.A * y_sub;
    EXPECT_GE(slack.minCoeff(), -eps);

    MatrixXd xi(group.A.rows(), 1);
    prog_sparse.GetDualVariable(ids[i], &xi);
    EXPECT_GE(xi.minCoeff(), -eps);

    VectorXd temp = group.A.transpose() * xi;
    for (int j = 0; j < static_cast<int>(group.variables.size()); j++) {
      Atx(group.variables[j]) += temp(j);
    }
  }
  EXPECT_NEAR((Atx - cost).norm(), 0, eps * cost.norm());

  // Solve with single dense LinearConstraint for comparison.
  Program prog_dense(num_vars);
  prog_dense.AddConstraint(LinearConstraint(A_dense, b_affine));
  VectorXd y_dense(num_vars);
  Solve(cost, prog_dense, config, y_dense.data());

  EXPECT_NEAR((y_sparse - y_dense).norm(), 0, 1e-5);
}

// Test with overlapping sparsity pattern (banded matrix).
GTEST_TEST(SparseLinearConstraint, BandedMatrix) {
  srand(7);
  int num_vars = 8;
  int bandwidth = 3;

  // Create groups of rows, each touching 'bandwidth' consecutive variables.
  // 3 rows per group, with overlapping variable sets.
  int rows_per_group = 3;
  int num_groups = num_vars - bandwidth + 1;
  int num_rows = rows_per_group * num_groups;

  std::vector<Eigen::Triplet<double>> triplets;
  for (int g = 0; g < num_groups; g++) {
    for (int r = 0; r < rows_per_group; r++) {
      int row = g * rows_per_group + r;
      for (int j = 0; j < bandwidth; j++) {
        triplets.emplace_back(row, g + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
      }
    }
  }
  Eigen::SparseMatrix<double> A(num_rows, num_vars);
  A.setFromTriplets(triplets.begin(), triplets.end());

  VectorXd b = VectorXd::Ones(num_rows) * 2.0;

  SparseLinearConstraint slc(A, b);
  EXPECT_EQ(slc.num_groups(), num_groups);

  // Solve and check feasibility.
  SolverConfiguration config = DefaultTestConfiguration();
  config.prepare_dual_variables = true;
  config.inv_sqrt_mu_max = 5e3;
  config.final_centering_tolerance = 1.01;

  // Build a feasible cost from a strictly feasible dual point.
  MatrixXd A_dense(A);
  VectorXd x0 = VectorXd::Random(num_rows).cwiseAbs() * 0.01;
  VectorXd cost = A_dense.transpose() * x0;

  Program prog(num_vars);
  slc.AddToProgram(prog);

  VectorXd y(num_vars);
  Solve(cost, prog, config, y.data());

  VectorXd slack = b - A_dense * y;
  EXPECT_GE(slack.minCoeff(), -1e-6);
}

// Test SparseLeastSquares: solve A^T A x = rhs via tree solver.
GTEST_TEST(SparseLeastSquares, BlockDiagonal) {
  srand(99);
  int num_blocks = 5;
  int rows_per_block = 8;
  int cols_per_block = 3;

  std::vector<MatrixXd> blocks(num_blocks);
  for (int i = 0; i < num_blocks; i++) {
    blocks[i] = MatrixXd::Random(rows_per_block, cols_per_block);
  }
  auto A = BlockDiagonal(blocks);
  int num_vars = A.cols();

  // Build rhs = A^T A * x_true for a known solution.
  VectorXd x_true = VectorXd::Random(num_vars);
  MatrixXd A_dense(A);
  VectorXd rhs = A_dense.transpose() * (A_dense * x_true);

  auto result = SparseLeastSquares(A, rhs);

  EXPECT_NEAR((result.x - x_true).norm(), 0, 1e-8 * x_true.norm());

  std::cout << "SparseLeastSquares (block-diagonal " << A.rows() << "x"
            << A.cols() << ", " << A.nonZeros() << " nnz):\n"
            << "  construction:       " << result.construction_time_us
            << " us\n"
            << "  assemble_and_factor: " << result.assemble_and_factor_time_us
            << " us\n"
            << "  solve:              " << result.solve_time_us << " us\n";
}

GTEST_TEST(SparseLeastSquares, Banded) {
  srand(42);
  int num_vars = 50;
  int bandwidth = 5;
  int rows_per_group = 10;
  int num_groups = num_vars - bandwidth + 1;
  int num_rows = rows_per_group * num_groups;

  std::vector<Eigen::Triplet<double>> triplets;
  for (int g = 0; g < num_groups; g++) {
    for (int r = 0; r < rows_per_group; r++) {
      int row = g * rows_per_group + r;
      for (int j = 0; j < bandwidth; j++) {
        triplets.emplace_back(row, g + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
      }
    }
  }
  Eigen::SparseMatrix<double> A(num_rows, num_vars);
  A.setFromTriplets(triplets.begin(), triplets.end());

  VectorXd x_true = VectorXd::Random(num_vars);
  MatrixXd A_dense(A);
  VectorXd rhs = A_dense.transpose() * (A_dense * x_true);

  auto result = SparseLeastSquares(A, rhs);

  EXPECT_NEAR((result.x - x_true).norm(), 0, 1e-8 * x_true.norm());

  std::cout << "SparseLeastSquares (banded " << A.rows() << "x" << A.cols()
            << ", " << A.nonZeros() << " nnz):\n"
            << "  construction:       " << result.construction_time_us
            << " us\n"
            << "  assemble_and_factor: " << result.assemble_and_factor_time_us
            << " us\n"
            << "  solve:              " << result.solve_time_us << " us\n";
}

// Verify that SparseLinearConstraintAssembler throws when used with a
// non-tree solver path.
GTEST_TEST(SparseLinearConstraintAssembler, ThrowsOnNonTreeSolver) {
  std::vector<MatrixXd> blocks = {MatrixXd::Random(4, 2),
                                  MatrixXd::Random(3, 3)};
  auto A = BlockDiagonal(blocks);
  VectorXd b = VectorXd::Ones(A.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A, b);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);

  // SetDenseData should throw since Decompose is required.
  EXPECT_THROW(assembler->SetDenseData(), std::runtime_error);
}

// Helper: solve LP with SparseLinearConstraintAssembler using given solver.
VectorXd SolveWithAssembler(
    const Eigen::SparseMatrix<double>& A_sparse,
    const VectorXd& b_affine,
    const VectorXd& cost,
    int kkt_solver,
    bool precompute_gram = false) {
  int num_vars = A_sparse.cols();

  auto slc = std::make_unique<SparseLinearConstraint>(A_sparse, b_affine);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  Program prog(num_vars);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  prog.constraint_manager().AddCustomAssembler(assembler.get());

  SolverConfiguration config = DefaultTestConfiguration();
  config.prepare_dual_variables = true;
  config.inv_sqrt_mu_max = 5e3;
  config.final_centering_tolerance = 1.01;
  config.final_centering_steps = 0;
  config.kkt_solver = kkt_solver;
  config.tree.precompute_gram = precompute_gram;

  VectorXd y(num_vars);
  Solve(cost, prog, config, y.data());
  return y;
}

// Solve a cone program (LP) using SparseLinearConstraintAssembler.
// Compare tree solver and supernodal solver against AddToProgram reference.
GTEST_TEST(SparseLinearConstraintAssembler, ConeProgram) {
  srand(42);
  double eps = 1e-6;

  int num_blocks = 4;
  int rows_per_block = 6;
  int cols_per_block = 3;

  std::vector<MatrixXd> blocks(num_blocks);
  for (int i = 0; i < num_blocks; i++) {
    blocks[i] = MatrixXd::Random(rows_per_block, cols_per_block);
  }

  auto A_sparse = BlockDiagonal(blocks);
  int num_rows = A_sparse.rows();
  int num_vars = A_sparse.cols();

  VectorXd b_affine = VectorXd::Ones(num_rows);

  VectorXd x0 = VectorXd::Random(num_rows).cwiseAbs() * 0.01;
  MatrixXd A_dense(A_sparse);
  VectorXd cost = A_dense.transpose() * x0;

  // --- Reference: AddToProgram ---
  SolverConfiguration config = DefaultTestConfiguration();
  config.prepare_dual_variables = true;
  config.inv_sqrt_mu_max = 5e3;
  config.final_centering_tolerance = 1.01;
  config.final_centering_steps = 0;

  SparseLinearConstraint slc_ref(A_sparse, b_affine);
  Program prog_ref(num_vars);
  slc_ref.AddToProgram(prog_ref);
  VectorXd y_ref(num_vars);
  Solve(cost, prog_ref, config, y_ref.data());

  // --- Tree solver ---
  VectorXd y_tree = SolveWithAssembler(
      A_sparse, b_affine, cost, CONEX_KKT_SOLVER_TREE);
  EXPECT_NEAR((y_tree - y_ref).norm(), 0, eps);

  // --- Supernodal solver ---
  VectorXd y_sn = SolveWithAssembler(
      A_sparse, b_affine, cost, CONEX_KKT_SOLVER_SUPERNODAL);
  EXPECT_NEAR((y_sn - y_ref).norm(), 0, eps);

  // Verify feasibility for both.
  VectorXd slack_tree = b_affine - A_dense * y_tree;
  VectorXd slack_sn = b_affine - A_dense * y_sn;
  EXPECT_GE(slack_tree.minCoeff(), -eps);
  EXPECT_GE(slack_sn.minCoeff(), -eps);
}

// Same test but with overlapping (banded) sparsity pattern.
GTEST_TEST(SparseLinearConstraintAssembler, ConeProgramBanded) {
  srand(7);
  double eps = 1e-6;
  int num_vars = 8;
  int bandwidth = 3;
  int rows_per_group = 3;
  int num_groups = num_vars - bandwidth + 1;
  int num_rows = rows_per_group * num_groups;

  std::vector<Eigen::Triplet<double>> triplets;
  for (int g = 0; g < num_groups; g++) {
    for (int r = 0; r < rows_per_group; r++) {
      int row = g * rows_per_group + r;
      for (int j = 0; j < bandwidth; j++) {
        triplets.emplace_back(row, g + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
      }
    }
  }
  Eigen::SparseMatrix<double> A_sparse(num_rows, num_vars);
  A_sparse.setFromTriplets(triplets.begin(), triplets.end());

  VectorXd b_affine = VectorXd::Ones(num_rows) * 2.0;
  MatrixXd A_dense(A_sparse);

  VectorXd x0 = VectorXd::Random(num_rows).cwiseAbs() * 0.01;
  VectorXd cost = A_dense.transpose() * x0;

  // --- Reference ---
  SolverConfiguration config = DefaultTestConfiguration();
  config.prepare_dual_variables = true;
  config.inv_sqrt_mu_max = 5e3;
  config.final_centering_tolerance = 1.01;

  SparseLinearConstraint slc_ref(A_sparse, b_affine);
  Program prog_ref(num_vars);
  slc_ref.AddToProgram(prog_ref);
  VectorXd y_ref(num_vars);
  Solve(cost, prog_ref, config, y_ref.data());

  // --- Tree solver ---
  VectorXd y_tree = SolveWithAssembler(
      A_sparse, b_affine, cost, CONEX_KKT_SOLVER_TREE);
  EXPECT_NEAR((y_tree - y_ref).norm(), 0, eps);
  EXPECT_GE((b_affine - A_dense * y_tree).minCoeff(), -eps);

  // --- Supernodal solver ---
  VectorXd y_sn = SolveWithAssembler(
      A_sparse, b_affine, cost, CONEX_KKT_SOLVER_SUPERNODAL);
  EXPECT_NEAR((y_sn - y_ref).norm(), 0, eps);
  EXPECT_GE((b_affine - A_dense * y_sn).minCoeff(), -eps);
}

// Compare solver timings between tree and supernodal on sparse LPs.
GTEST_TEST(SparseLinearConstraintAssembler, SolverTimingComparison) {
  using clock = std::chrono::high_resolution_clock;

  struct TestCase {
    std::string name;
    int num_vars;
    int bandwidth;
    int rows_per_group;
  };

  std::vector<TestCase> cases = {
      {"small_banded", 20, 3, 5},
      {"medium_banded", 50, 5, 10},
      {"large_banded", 100, 5, 10},
      {"wide_band", 80, 10, 8},
      {"large_wide", 200, 10, 20},
  };

  std::cout << "\n=== Solver Timing Comparison (tree vs supernodal) ===\n";
  std::cout << "name                  | vars | rows | nnz    | tree (ms)  | supernodal (ms)\n";
  std::cout << "-------------------   | ---- | ---- | ------ | ---------- | ---------------\n";

  for (const auto& tc : cases) {
    srand(42);
    int num_groups = tc.num_vars - tc.bandwidth + 1;
    int num_rows = tc.rows_per_group * num_groups;

    std::vector<Eigen::Triplet<double>> triplets;
    for (int g = 0; g < num_groups; g++) {
      for (int r = 0; r < tc.rows_per_group; r++) {
        int row = g * tc.rows_per_group + r;
        for (int j = 0; j < tc.bandwidth; j++) {
          triplets.emplace_back(row, g + j,
                                0.5 + static_cast<double>(rand()) / RAND_MAX);
        }
      }
    }
    Eigen::SparseMatrix<double> A_sparse(num_rows, tc.num_vars);
    A_sparse.setFromTriplets(triplets.begin(), triplets.end());

    VectorXd b_affine = VectorXd::Ones(num_rows) * 2.0;
    MatrixXd A_dense(A_sparse);
    VectorXd x0 = VectorXd::Random(num_rows).cwiseAbs() * 0.01;
    VectorXd cost = A_dense.transpose() * x0;

    // Print group counts for the two decomposition strategies.
    SparseLinearConstraint slc(A_sparse, b_affine);
    int containment_groups = slc.num_groups();
    // Containment-merged groups have no merging for banded patterns
    // (no support is a subset of another), so this equals the number
    // of unique supports.
    int unique_supports = static_cast<int>(slc.row_supports().size());

    // Tree solver with precomputed Gram.
    auto t0 = clock::now();
    VectorXd y_tree = SolveWithAssembler(
        A_sparse, b_affine, cost, CONEX_KKT_SOLVER_TREE,
        /*precompute_gram=*/true);
    auto t1 = clock::now();

    // Supernodal solver.
    auto t2 = clock::now();
    VectorXd y_sn = SolveWithAssembler(
        A_sparse, b_affine, cost, CONEX_KKT_SOLVER_SUPERNODAL);
    auto t3 = clock::now();

    double tree_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double sn_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // Verify both give same answer.
    EXPECT_NEAR((y_tree - y_sn).norm(), 0, 1e-5);

    printf("%-21s | %4d | %4d | %6ld | %4d supports, %4d containment groups | %10.3f | %10.3f\n",
           tc.name.c_str(), tc.num_vars, num_rows,
           static_cast<long>(A_sparse.nonZeros()),
           unique_supports, containment_groups,
           tree_ms, sn_ms);
  }
}

// Examine clique tree structures for both solver paths.
GTEST_TEST(SparseLinearConstraintAssembler, CliqueTreeComparison) {
  srand(42);
  int num_vars = 80;
  int bandwidth = 10;
  int rows_per_group = 8;
  int num_groups = num_vars - bandwidth + 1;
  int num_rows = rows_per_group * num_groups;

  std::vector<Eigen::Triplet<double>> triplets;
  for (int g = 0; g < num_groups; g++) {
    for (int r = 0; r < rows_per_group; r++) {
      int row = g * rows_per_group + r;
      for (int j = 0; j < bandwidth; j++) {
        triplets.emplace_back(row, g + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
      }
    }
  }
  Eigen::SparseMatrix<double> A_sparse(num_rows, num_vars);
  A_sparse.setFromTriplets(triplets.begin(), triplets.end());
  VectorXd b_affine = VectorXd::Ones(num_rows) * 2.0;

  SparseLinearConstraint slc(A_sparse, b_affine);

  // --- Supernodal path: containment-merged cliques → MakePrimalDualCliqueTree ---
  std::vector<std::vector<int>> sn_cliques;
  for (const auto& g : slc.groups()) {
    sn_cliques.push_back(g.variables);
  }
  CliqueTree sn_tree = MakeCliqueTree(sn_cliques);

  std::cout << "\n=== Supernodal solver clique tree ===\n";
  std::cout << "Input cliques: " << sn_cliques.size() << "\n";
  std::cout << "Tree nodes: " << sn_tree.supernodes.size() << "\n";
  int sn_with_supernodes = 0;
  int sn_total_fill = 0;
  for (size_t i = 0; i < sn_tree.supernodes.size(); i++) {
    int sn_size = sn_tree.supernodes[i].size();
    int sep_size = sn_tree.separators[i].size();
    sn_total_fill += sn_size * (sn_size + sep_size);
    if (sn_size > 0) sn_with_supernodes++;
    if (i < 20 || sn_size == 0) {
      std::cout << "  node " << i << ": sn=" << sn_size
                << " sep=" << sep_size
                << " total=" << sn_size + sep_size
                << (sn_size == 0 ? " *** NO SUPERNODE ***" : "")
                << "\n";
    }
  }
  if (sn_tree.supernodes.size() > 20) {
    std::cout << "  ... (" << sn_tree.supernodes.size() - 20 << " more nodes)\n";
  }
  std::cout << "Nodes with supernodes: " << sn_with_supernodes
            << " / " << sn_tree.supernodes.size() << "\n";
  std::cout << "Total fill (sum of sn*(sn+sep)): " << sn_total_fill << "\n";

  // --- Tree solver path: row supports → MakeCliqueTreeMinDegreeFromRowSupports ---
  std::vector<std::vector<int>> maximal_cliques;
  CliqueTree tree_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      slc.row_supports(), &maximal_cliques,
      /*max_merge_supernode_size=*/0, SUPERNODE_REORDER_BFS_GREEDY, {});

  std::cout << "\n=== Tree solver clique tree ===\n";
  std::cout << "Input supports: " << slc.row_supports().size() << "\n";
  std::cout << "Maximal cliques: " << maximal_cliques.size() << "\n";
  std::cout << "Tree nodes: " << tree_tree.supernodes.size() << "\n";
  int tree_with_supernodes = 0;
  int tree_total_fill = 0;
  for (size_t i = 0; i < tree_tree.supernodes.size(); i++) {
    int sn_size = tree_tree.supernodes[i].size();
    int sep_size = tree_tree.separators[i].size();
    tree_total_fill += sn_size * (sn_size + sep_size);
    if (sn_size > 0) tree_with_supernodes++;
    if (i < 20 || sn_size == 0) {
      std::cout << "  node " << i << ": sn=" << sn_size
                << " sep=" << sep_size
                << " total=" << sn_size + sep_size
                << (sn_size == 0 ? " *** NO SUPERNODE ***" : "")
                << "\n";
    }
  }
  if (tree_tree.supernodes.size() > 20) {
    std::cout << "  ... (" << tree_tree.supernodes.size() - 20 << " more nodes)\n";
  }
  std::cout << "Nodes with supernodes: " << tree_with_supernodes
            << " / " << tree_tree.supernodes.size() << "\n";
  std::cout << "Total fill (sum of sn*(sn+sep)): " << tree_total_fill << "\n";
}

}  // namespace conex
