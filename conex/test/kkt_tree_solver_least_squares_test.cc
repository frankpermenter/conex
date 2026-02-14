#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <thread>
#include <vector>

#include "conex/clique_ordering.h"
#include "conex/kkt_tree_solver.h"
#include "conex/static_subsystem.h"
#include "conex/workspace.h"
#include <Eigen/SparseQR>
#include <gtest/gtest.h>

namespace conex {
namespace {

using RowSparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;

class RowPartitionAssembler final : public SupernodalAssemblerBase {
 public:
  RowPartitionAssembler(const RowSparseMatrix& A, const std::vector<int>& rows,
                        const std::vector<int>& clique_variables)
      : SupernodalAssemblerBase(clique_variables),
        local_matrix_(clique_variables.size(), clique_variables.size()) {
    CONEX_DEMAND(A.cols() > 0, "A must have non-zero columns.");
    local_matrix_.setZero();

    std::vector<int> column_to_local(A.cols(), -1);
    for (size_t i = 0; i < clique_variables.size(); ++i) {
      column_to_local.at(clique_variables.at(i)) = static_cast<int>(i);
    }

    for (int row : rows) {
      std::vector<std::pair<int, double>> local_entries;
      for (RowSparseMatrix::InnerIterator it(A, row); it; ++it) {
        const int local = column_to_local.at(it.col());
        if (local >= 0) {
          local_entries.emplace_back(local, it.value());
        }
      }
      for (size_t i = 0; i < local_entries.size(); ++i) {
        for (size_t j = 0; j <= i; ++j) {
          const int r = local_entries.at(i).first;
          const int c = local_entries.at(j).first;
          const double v =
              local_entries.at(i).second * local_entries.at(j).second;
          local_matrix_(r, c) += v;
          if (r != c) {
            local_matrix_(c, r) += v;
          }
        }
      }
    }

    Workspace workspace(&submatrix_data_);
    memory_.resize(SizeOf(workspace));
    Initialize(&workspace, memory_.data());
  }

  void SetDenseData() override {
    submatrix_data_.G = local_matrix_;
  }

 private:
  Eigen::MatrixXd local_matrix_;
  Eigen::VectorXd memory_;
};

class StaticMatrixAssembler final : public SupernodalAssemblerBase {
 public:
  StaticMatrixAssembler(const std::vector<int>& variables,
                        const Eigen::MatrixXd& local_matrix)
      : SupernodalAssemblerBase(variables), local_matrix_(local_matrix) {
    CONEX_DEMAND(local_matrix_.rows() == local_matrix_.cols(),
                 "Local matrix must be square.");
    CONEX_DEMAND(local_matrix_.rows() == static_cast<int>(variables.size()),
                 "Local matrix size must match variables.");
    Workspace workspace(&submatrix_data_);
    memory_.resize(SizeOf(workspace));
    Initialize(&workspace, memory_.data());
  }

  void SetDenseData() override { submatrix_data_.G = local_matrix_; }

 private:
  Eigen::MatrixXd local_matrix_;
  Eigen::VectorXd memory_;
};

std::vector<int> RowSupport(const RowSparseMatrix& A, int row) {
  std::vector<int> support;
  for (RowSparseMatrix::InnerIterator it(A, row); it; ++it) {
    support.push_back(it.col());
  }
  std::sort(support.begin(), support.end());
  support.erase(std::unique(support.begin(), support.end()), support.end());
  return support;
}

std::vector<std::vector<int>> PartitionRows(int num_rows, int num_parts) {
  std::vector<std::vector<int>> partitions;
  partitions.resize(num_parts);
  for (int r = 0; r < num_rows; ++r) {
    partitions.at(r % num_parts).push_back(r);
  }
  return partitions;
}

RowSparseMatrix BuildSparseLeastSquaresMatrix(int n) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(5 * n);
  std::mt19937 rng(17);
  std::uniform_real_distribution<double> noise(0.01, 0.04);

  const int m = 3 * n - 2;
  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, 2.0 + noise(rng));
  }
  for (int i = 0; i < n - 1; ++i) {
    triplets.emplace_back(n + i, i, -1.0);
    triplets.emplace_back(n + i, i + 1, 1.0);
  }
  for (int i = 0; i < n - 1; ++i) {
    triplets.emplace_back(2 * n - 1 + i, i, 0.5);
    triplets.emplace_back(2 * n - 1 + i, i + 1, 0.5);
  }

  RowSparseMatrix A(m, n);
  A.setFromTriplets(triplets.begin(), triplets.end());
  A.makeCompressed();
  return A;
}

TEST(KKTTreeSolver, SparseLeastSquaresFromRowPartitions) {
  const int n = 120;
  const RowSparseMatrix A = BuildSparseLeastSquaresMatrix(n);
  const int row_partitions = 6;
  const auto partitions = PartitionRows(A.rows(), row_partitions);

  std::vector<std::vector<int>> cliques;
  std::vector<std::vector<int>> rows_per_clique;
  for (const auto& partition_rows : partitions) {
    std::map<std::vector<int>, std::vector<int>> grouped_rows;
    for (int row : partition_rows) {
      auto support = RowSupport(A, row);
      if (!support.empty()) {
        grouped_rows[support].push_back(row);
      }
    }
    for (const auto& entry : grouped_rows) {
      cliques.push_back(entry.first);
      rows_per_clique.push_back(entry.second);
    }
  }

  CliqueTree clique_tree = MakeCliqueTree(cliques);
  SymmetricLinearSystemTreeSolver tree_solver;
  tree_solver.SetNumThreads(
      std::min<int>(4, std::max(2u, std::thread::hardware_concurrency())));
  tree_solver.EnableAutoUpdateAtAssemble(true);
  tree_solver.SetFactorizationMode(true);

  std::vector<std::unique_ptr<RowPartitionAssembler>> assemblers;
  std::vector<std::unique_ptr<KKTAssemblerToSubsystemAdapter>> adapters;
  assemblers.reserve(cliques.size());
  adapters.reserve(cliques.size());
  for (size_t i = 0; i < cliques.size(); ++i) {
    assemblers.emplace_back(std::make_unique<RowPartitionAssembler>(
        A, rows_per_clique.at(i), cliques.at(i)));
    auto adapter = std::make_unique<KKTAssemblerToSubsystemAdapter>(
        assemblers.back().get());
    auto* subsystem =
        adapter->create_subsystem(SubsystemType::kPositiveDefinite);
    tree_solver.AddSubsystem(subsystem);
    tree_solver.push_back(std::move(adapter));
  }
  tree_solver.Finalize(clique_tree);

  Eigen::VectorXd x_true = Eigen::VectorXd::LinSpaced(n, -1.0, 1.0);
  Eigen::VectorXd b = A * x_true;
  Eigen::VectorXd rhs = A.transpose() * b;

  Eigen::MatrixXd rhs_mat(rhs.rows(), 1);
  rhs_mat.col(0) = rhs;

  const auto tree_start = std::chrono::steady_clock::now();
  tree_solver.Assemble();
  ASSERT_TRUE(tree_solver.Factor());
  Eigen::MatrixXd x_tree = tree_solver.Solve(rhs_mat, true);
  const auto tree_elapsed = std::chrono::steady_clock::now() - tree_start;

  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
  const Eigen::SparseMatrix<double> A_col_major = A;
  const auto qr_start = std::chrono::steady_clock::now();
  qr.compute(A_col_major);
  ASSERT_EQ(qr.info(), Eigen::Success);
  Eigen::VectorXd x_qr = qr.solve(b);
  ASSERT_EQ(qr.info(), Eigen::Success);
  const auto qr_elapsed = std::chrono::steady_clock::now() - qr_start;

  const Eigen::VectorXd x_tree_vec = x_tree.col(0);
  const double rel_diff =
      (x_tree_vec - x_qr).norm() / std::max(1.0, x_qr.norm());
  const double tree_residual =
      (A * x_tree_vec - b).norm() / std::max(1.0, b.norm());
  const double qr_residual = (A * x_qr - b).norm() / std::max(1.0, b.norm());

  std::cout << "least_squares tree_ms="
            << std::chrono::duration<double, std::milli>(tree_elapsed).count()
            << " qr_ms="
            << std::chrono::duration<double, std::milli>(qr_elapsed).count()
            << " rel_diff=" << rel_diff << " tree_residual=" << tree_residual
            << " qr_residual=" << qr_residual << "\n";

  EXPECT_LT(rel_diff, 2e-5);
  EXPECT_LT(tree_residual, 2e-5);
  EXPECT_LT(qr_residual, 2e-8);
}

TEST(KKTTreeSolver, RelabelsNonContiguousGlobalCliquesAndSolves) {
  // Two independent cliques in global labels: {0,2} and {1,3}.
  // Even though labels are non-contiguous globally, solver should relabel and
  // solve correctly.
  Eigen::MatrixXd local_02(2, 2);
  local_02 << 4.0, 1.0, 1.0, 3.0;
  Eigen::MatrixXd local_13(2, 2);
  local_13 << 5.0, 2.0, 2.0, 6.0;

  SymmetricLinearSystemTreeSolver tree_solver;
  tree_solver.SetNumThreads(1);
  tree_solver.EnableAutoUpdateAtAssemble(true);
  tree_solver.SetFactorizationMode(true);

  std::vector<std::unique_ptr<StaticMatrixAssembler>> assemblers;
  std::vector<std::unique_ptr<KKTAssemblerToSubsystemAdapter>> adapters;

  assemblers.emplace_back(
      std::make_unique<StaticMatrixAssembler>(std::vector<int>{0, 2}, local_02));
  {
    auto adapter = std::make_unique<KKTAssemblerToSubsystemAdapter>(
        assemblers.back().get());
    auto* subsystem =
        adapter->create_subsystem(SubsystemType::kPositiveDefinite);
    tree_solver.AddSubsystem(subsystem);
    tree_solver.push_back(std::move(adapter));
  }

  assemblers.emplace_back(
      std::make_unique<StaticMatrixAssembler>(std::vector<int>{1, 3}, local_13));
  {
    auto adapter = std::make_unique<KKTAssemblerToSubsystemAdapter>(
        assemblers.back().get());
    auto* subsystem =
        adapter->create_subsystem(SubsystemType::kPositiveDefinite);
    tree_solver.AddSubsystem(subsystem);
    tree_solver.push_back(std::move(adapter));
  }

  CliqueTree clique_tree;
  clique_tree.supernodes = {{0, 2}, {1, 3}};
  clique_tree.separators = {{}, {}};
  clique_tree.node_to_parent = {-1, -1};
  tree_solver.Finalize(clique_tree);

  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(4, 4);
  K(0, 0) = local_02(0, 0);
  K(0, 2) = local_02(0, 1);
  K(2, 0) = local_02(1, 0);
  K(2, 2) = local_02(1, 1);
  K(1, 1) = local_13(0, 0);
  K(1, 3) = local_13(0, 1);
  K(3, 1) = local_13(1, 0);
  K(3, 3) = local_13(1, 1);

  Eigen::VectorXd rhs(4);
  rhs << 1.0, -2.0, 3.0, 4.0;
  Eigen::MatrixXd rhs_mat(4, 1);
  rhs_mat.col(0) = rhs;

  tree_solver.Assemble();
  ASSERT_TRUE(tree_solver.Factor());

  Eigen::MatrixXd x_tree;
  EXPECT_NO_THROW(x_tree = tree_solver.Solve(rhs_mat, true));

  Eigen::LDLT<Eigen::MatrixXd> ldlt(K);
  ASSERT_EQ(ldlt.info(), Eigen::Success);
  const Eigen::VectorXd x_ref = ldlt.solve(rhs);
  ASSERT_EQ(ldlt.info(), Eigen::Success);

  const double rel_error =
      (x_tree.col(0) - x_ref).norm() / std::max(1.0, x_ref.norm());
  EXPECT_LT(rel_error, 1e-10);
}

}  // namespace
}  // namespace conex
