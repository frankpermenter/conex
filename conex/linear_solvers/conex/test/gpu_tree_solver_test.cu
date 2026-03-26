#include "conex/gpu_tree_solver/gpu_tree_solver.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/clique_ordering.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/kkt_solver_factory.h"
#include "conex/tree_solver/kkt_tree_solver.h"

#include <set>

namespace conex {
namespace {

// Helper: build a CliqueTree and CPU solver, then compare GPU vs CPU.
struct TestProblem {
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd rhs;
  Eigen::VectorXd x_true;  // A^T A x_true = rhs

  CliqueTree clique_tree;
  std::vector<std::vector<int>> maximal_cliques;

  // CPU reference solution.
  Eigen::VectorXd x_cpu;
};

TestProblem MakeBlockDiagonal(int blocks, int rows_per_block, int cols_per_block,
                               int seed = 42) {
  srand(seed);
  TestProblem p;
  int total_rows = blocks * rows_per_block;
  int total_cols = blocks * cols_per_block;

  std::vector<Eigen::Triplet<double>> trips;
  for (int b = 0; b < blocks; ++b) {
    for (int i = 0; i < rows_per_block; ++i) {
      for (int j = 0; j < cols_per_block; ++j) {
        double val = static_cast<double>(rand()) / RAND_MAX - 0.5;
        trips.emplace_back(b * rows_per_block + i, b * cols_per_block + j, val);
      }
    }
  }
  p.A.resize(total_rows, total_cols);
  p.A.setFromTriplets(trips.begin(), trips.end());

  // x_true random, rhs = A^T A x_true.
  p.x_true = Eigen::VectorXd::Random(total_cols);
  Eigen::MatrixXd Ad(p.A);
  p.rhs = Ad.transpose() * (Ad * p.x_true);

  // Build clique tree.
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(total_rows);
  auto slc = std::make_unique<SparseLinearConstraint>(p.A, b_zero);
  std::set<int> vs;
  for (const auto& s : slc->row_supports()) vs.insert(s.begin(), s.end());
  std::vector<int> av(vs.begin(), vs.end());

  ConstraintManager cm(total_cols);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), av);
  cm.AddCustomAssembler(assembler.get());

  SolverConfiguration cfg;
  auto cpu_solver = MakeTreeSolver(&cm, cfg);
  EXPECT_TRUE(cpu_solver->AssembleAndFactor());
  p.x_cpu = cpu_solver->Solve(p.rhs);

  // Extract clique tree from the CPU solver's internal state.
  // We rebuild it here since the CPU solver doesn't expose it directly.
  auto slc2 = std::make_unique<SparseLinearConstraint>(p.A, b_zero);
  std::vector<std::vector<int>> cliques;
  for (const auto& s : slc2->row_supports()) cliques.push_back(s);

  p.clique_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      cliques, &p.maximal_cliques);

  return p;
}

TestProblem MakeBanded(int n, int bandwidth, int rows_per_group, int seed = 42) {
  srand(seed);
  TestProblem p;
  int num_groups = n - bandwidth + 1;
  int total_rows = rows_per_group * num_groups;

  std::vector<Eigen::Triplet<double>> trips;
  for (int g = 0; g < num_groups; ++g) {
    for (int i = 0; i < rows_per_group; ++i) {
      for (int j = 0; j < bandwidth; ++j) {
        double val = static_cast<double>(rand()) / RAND_MAX - 0.5;
        trips.emplace_back(g * rows_per_group + i, g + j, val);
      }
    }
  }
  p.A.resize(total_rows, n);
  p.A.setFromTriplets(trips.begin(), trips.end());

  p.x_true = Eigen::VectorXd::Random(n);
  Eigen::MatrixXd Ad(p.A);
  p.rhs = Ad.transpose() * (Ad * p.x_true);

  // CPU solve.
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(total_rows);
  auto slc = std::make_unique<SparseLinearConstraint>(p.A, b_zero);
  std::set<int> vs;
  for (const auto& s : slc->row_supports()) vs.insert(s.begin(), s.end());
  std::vector<int> av(vs.begin(), vs.end());

  ConstraintManager cm(n);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), av);
  cm.AddCustomAssembler(assembler.get());

  SolverConfiguration cfg;
  auto cpu_solver = MakeTreeSolver(&cm, cfg);
  EXPECT_TRUE(cpu_solver->AssembleAndFactor());
  p.x_cpu = cpu_solver->Solve(p.rhs);

  // Clique tree.
  auto slc2 = std::make_unique<SparseLinearConstraint>(p.A, b_zero);
  std::vector<std::vector<int>> cliques;
  for (const auto& s : slc2->row_supports()) cliques.push_back(s);
  p.clique_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      cliques, &p.maximal_cliques);

  return p;
}

// Build and factor a GpuTreeSolver from a CliqueTree and A matrix.
// Assembles A^T A into per-supernode dense blocks on the host,
// then factors on GPU and solves.
Eigen::VectorXd SolveOnGpu(const TestProblem& prob) {
  const auto& ct = prob.clique_tree;
  const int n = prob.A.cols();
  const int num_cliques = static_cast<int>(ct.supernodes.size());

  GpuTreeSolver gpu;
  gpu.Finalize(ct);

  // Assemble A^T A into per-supernode dense blocks.
  // Each supernode ci has variables = supernodes[ci] ∪ separators[ci].
  // The dense block is (A_clique)^T * (A_clique) where A_clique has
  // columns corresponding to the clique's variables.
  Eigen::MatrixXd AtA(prob.A.transpose() * prob.A);

  for (int ci = 0; ci < num_cliques; ++ci) {
    std::vector<int> vars = ct.supernodes[ci];
    vars.insert(vars.end(), ct.separators[ci].begin(), ct.separators[ci].end());
    int total = static_cast<int>(vars.size());

    Eigen::MatrixXd block(total, total);
    for (int i = 0; i < total; ++i)
      for (int j = 0; j < total; ++j)
        block(i, j) = AtA(vars[i], vars[j]);

    gpu.SetSupernodeData(ci, block);
  }

  EXPECT_TRUE(gpu.AssembleAndFactor());
  return gpu.Solve(prob.rhs);
}

// --- Tests ---

TEST(GpuTreeSolver, Finalize) {
  auto prob = MakeBlockDiagonal(3, 6, 4);

  GpuTreeSolver gpu;
  gpu.Finalize(prob.clique_tree);

  EXPECT_EQ(gpu.number_of_variables(), prob.A.cols());
  EXPECT_GT(gpu.num_levels(), 0);
  EXPECT_EQ(static_cast<int>(gpu.descriptors().size()),
            static_cast<int>(prob.clique_tree.supernodes.size()));
}

TEST(GpuTreeSolver, BlockDiagonal) {
  auto prob = MakeBlockDiagonal(3, 8, 4);
  Eigen::VectorXd x_gpu = SolveOnGpu(prob);

  double rel_err = (x_gpu - prob.x_cpu).norm() / prob.x_cpu.norm();
  EXPECT_LT(rel_err, 1e-10)
      << "GPU vs CPU relative error: " << rel_err;

  double true_err = (x_gpu - prob.x_true).norm() / prob.x_true.norm();
  EXPECT_LT(true_err, 1e-10)
      << "GPU vs ground truth relative error: " << true_err;
}

TEST(GpuTreeSolver, Banded) {
  auto prob = MakeBanded(30, 5, 6);
  Eigen::VectorXd x_gpu = SolveOnGpu(prob);

  double rel_err = (x_gpu - prob.x_cpu).norm() / prob.x_cpu.norm();
  EXPECT_LT(rel_err, 1e-10)
      << "GPU vs CPU relative error: " << rel_err;
}

TEST(GpuTreeSolver, LargerBanded) {
  auto prob = MakeBanded(100, 10, 8);
  Eigen::VectorXd x_gpu = SolveOnGpu(prob);

  double rel_err = (x_gpu - prob.x_cpu).norm() / prob.x_cpu.norm();
  EXPECT_LT(rel_err, 1e-10)
      << "GPU vs CPU relative error: " << rel_err;
}

TEST(GpuTreeSolver, RepeatedSolve) {
  // Factor once, solve with multiple RHS.
  auto prob = MakeBlockDiagonal(4, 6, 3);
  const auto& ct = prob.clique_tree;
  const int n = prob.A.cols();
  const int num_cliques = static_cast<int>(ct.supernodes.size());

  GpuTreeSolver gpu;
  gpu.Finalize(ct);

  Eigen::MatrixXd AtA(prob.A.transpose() * prob.A);
  for (int ci = 0; ci < num_cliques; ++ci) {
    std::vector<int> vars = ct.supernodes[ci];
    vars.insert(vars.end(), ct.separators[ci].begin(), ct.separators[ci].end());
    int total = static_cast<int>(vars.size());
    Eigen::MatrixXd block(total, total);
    for (int i = 0; i < total; ++i)
      for (int j = 0; j < total; ++j)
        block(i, j) = AtA(vars[i], vars[j]);
    gpu.SetSupernodeData(ci, block);
  }

  ASSERT_TRUE(gpu.AssembleAndFactor());

  // Solve with 3 different RHS vectors.
  for (int trial = 0; trial < 3; ++trial) {
    srand(100 + trial);
    Eigen::VectorXd x_true = Eigen::VectorXd::Random(n);
    Eigen::VectorXd rhs = AtA * x_true;
    Eigen::VectorXd x_gpu = gpu.Solve(rhs);

    double rel_err = (x_gpu - x_true).norm() / x_true.norm();
    EXPECT_LT(rel_err, 1e-10)
        << "trial " << trial << " GPU error: " << rel_err;
  }
}

TEST(GpuTreeSolver, PartitionInterface) {
  auto prob = MakeBlockDiagonal(2, 6, 4);

  GpuTreeSolver gpu;
  gpu.Finalize(prob.clique_tree);

  // Verify partition is accessible through KKTSolverBase.
  KKTSolverBase* base = &gpu;
  EXPECT_EQ(base->number_of_variables(), prob.A.cols());

  auto& partition = base->partition();
  EXPECT_EQ(partition.num_variables(), prob.A.cols());
}

}  // namespace
}  // namespace conex
