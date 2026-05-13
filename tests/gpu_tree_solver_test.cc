#include "conex/gpu_tree_solver/gpu_tree_solver.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/clique_ordering.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/linear_solvers/kkt_solver_factory.h"
#include "conex/linear_solvers/kkt_tree_solver.h"
#include "conex/common/kkt_solver_dense.h"

#include <set>
#include <unordered_map>

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
  p.x_cpu = KKTSolve(*cpu_solver, p.rhs);

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
  p.x_cpu = KKTSolve(*cpu_solver, p.rhs);

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
  gpu.FinalizeStructure(ct);

  // Build elimination ordering (same as GpuTreeSolver::FinalizeStructure).
  Eigen::VectorXi perm = Eigen::VectorXi::Constant(n, -1);
  int epos = 0;
  for (int ci : ct.post_order_position_to_clique) {
    for (int v : ct.supernodes[ci]) {
      if (v >= 0 && v < n) perm(v) = epos++;
    }
  }

  // Build per-clique variable sets for lookup.
  std::vector<std::set<int>> clique_vars(num_cliques);
  for (int ci = 0; ci < num_cliques; ++ci) {
    for (int v : ct.supernodes[ci]) clique_vars[ci].insert(v);
    for (int v : ct.separators[ci]) clique_vars[ci].insert(v);
  }

  // For each clique, build a map from variable to local index.
  std::vector<std::vector<int>> clique_var_list(num_cliques);
  std::vector<std::unordered_map<int, int>> var_to_local(num_cliques);
  for (int ci = 0; ci < num_cliques; ++ci) {
    auto& vars = clique_var_list[ci];
    vars = ct.supernodes[ci];
    vars.insert(vars.end(), ct.separators[ci].begin(), ct.separators[ci].end());
    for (int k = 0; k < static_cast<int>(vars.size()); ++k) {
      var_to_local[ci][vars[k]] = k;
    }
  }

  // Assemble per-row contributions: each row of A contributes A_r^T A_r
  // to the clique that owns the first-eliminated variable of the row's support.
  std::vector<Eigen::MatrixXd> blocks(num_cliques);
  for (int ci = 0; ci < num_cliques; ++ci) {
    int total = static_cast<int>(clique_var_list[ci].size());
    blocks[ci] = Eigen::MatrixXd::Zero(total, total);
  }

  // For each row of A, find its support and assign to the owning clique.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row(prob.A);
  for (int r = 0; r < A_row.rows(); ++r) {
    // Collect the row's support (nonzero columns).
    std::vector<std::pair<int, double>> row_entries;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r);
         it; ++it) {
      row_entries.push_back({static_cast<int>(it.col()), it.value()});
    }
    if (row_entries.empty()) continue;

    // Find the clique whose supernode contains the first-eliminated variable.
    int first_elim_var = -1;
    int first_elim_pos = n;
    for (const auto& e : row_entries) {
      if (perm(e.first) < first_elim_pos) {
        first_elim_pos = perm(e.first);
        first_elim_var = e.first;
      }
    }

    // Find the clique whose supernode contains first_elim_var.
    int owner = -1;
    for (int ci = 0; ci < num_cliques; ++ci) {
      for (int v : ct.supernodes[ci]) {
        if (v == first_elim_var) { owner = ci; break; }
      }
      if (owner >= 0) break;
    }
    EXPECT_GE(owner, 0) << "No clique owns variable " << first_elim_var;

    // Verify the row's support is a subset of the owning clique's variables.
    // If not, find the ancestor clique that contains all support variables.
    const auto& owner_vars = clique_vars[owner];
    bool all_in_owner = true;
    for (const auto& e : row_entries) {
      if (owner_vars.find(e.first) == owner_vars.end()) {
        all_in_owner = false;
        break;
      }
    }

    if (!all_in_owner) {
      // Search for a clique that contains ALL of this row's support.
      std::set<int> row_support;
      for (const auto& e : row_entries) row_support.insert(e.first);

      owner = -1;
      for (int ci = 0; ci < num_cliques; ++ci) {
        bool contains_all = true;
        for (int v : row_support) {
          if (clique_vars[ci].find(v) == clique_vars[ci].end()) {
            contains_all = false;
            break;
          }
        }
        if (contains_all) {
          // Among cliques that contain all vars, pick the one with
          // the earliest-eliminated supernode variable.
          if (owner < 0) {
            owner = ci;
          } else {
            int min_elim_ci = n, min_elim_owner = n;
            for (int v : ct.supernodes[ci])
              min_elim_ci = std::min(min_elim_ci, (int)perm(v));
            for (int v : ct.supernodes[owner])
              min_elim_owner = std::min(min_elim_owner, (int)perm(v));
            if (min_elim_ci < min_elim_owner) owner = ci;
          }
        }
      }
      EXPECT_GE(owner, 0) << "No clique covers row " << r << "'s support";
    }

    // Add A_r^T * A_r to the owning clique's block.
    const auto& local_map = var_to_local[owner];
    for (const auto& ei : row_entries) {
      auto it_i = local_map.find(ei.first);
      EXPECT_NE(it_i, local_map.end());
      if (it_i == local_map.end()) continue;
      for (const auto& ej : row_entries) {
        auto it_j = local_map.find(ej.first);
        EXPECT_NE(it_j, local_map.end());
        if (it_j == local_map.end()) continue;
        blocks[owner](it_i->second, it_j->second) += ei.second * ej.second;
      }
    }
  }

  for (int ci = 0; ci < num_cliques; ++ci) {
    gpu.SetSupernodeData(ci, blocks[ci]);
  }

  EXPECT_TRUE(gpu.AssembleAndFactor());
  return KKTSolve(gpu,prob.rhs);
}

// --- Tests ---

TEST(GpuTreeSolver, FinalizeStructure) {
  auto prob = MakeBlockDiagonal(3, 6, 4);

  GpuTreeSolver gpu;
  gpu.FinalizeStructure(prob.clique_tree);

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
  gpu.FinalizeStructure(ct);

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
    Eigen::VectorXd x_gpu = KKTSolve(gpu,rhs);

    double rel_err = (x_gpu - x_true).norm() / x_true.norm();
    EXPECT_LT(rel_err, 1e-10)
        << "trial " << trial << " GPU error: " << rel_err;
  }
}

TEST(GpuTreeSolver, PartitionInterface) {
  auto prob = MakeBlockDiagonal(2, 6, 4);

  GpuTreeSolver gpu;
  gpu.FinalizeStructure(prob.clique_tree);

  // Verify partition is accessible through KKTSolverBase.
  KKTSolverBase* base = &gpu;
  EXPECT_EQ(base->number_of_variables(), prob.A.cols());

  EXPECT_EQ(base->number_of_variables(), prob.A.cols());
}

}  // namespace
}  // namespace conex
