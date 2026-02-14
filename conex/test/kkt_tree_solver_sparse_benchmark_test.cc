#include <chrono>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "conex/kkt_tree_solver.h"
#include "conex/static_subsystem.h"
#include "conex/workspace.h"
#include <Eigen/SparseQR>
#include <gtest/gtest.h>

namespace conex {
namespace {

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

  void SetDenseData() override {
    submatrix_data_.G = local_matrix_;
  }

 private:
  Eigen::MatrixXd local_matrix_;
  Eigen::VectorXd memory_;
};

struct MatrixCase {
  std::string name;
  Eigen::SparseMatrix<double> matrix;
  std::vector<std::vector<int>> cliques;
  std::vector<std::vector<int>> supernodes;
  std::vector<std::vector<int>> separators;
  std::vector<int> node_to_parent;
};

MatrixCase BuildBlockDiagonalCase(const std::string& name,
                                  const std::vector<int>& block_sizes,
                                  int seed) {
  const int n = std::accumulate(block_sizes.begin(), block_sizes.end(), 0);
  std::vector<Eigen::Triplet<double>> triplets;
  int reserve_nnz = 0;
  for (int bs : block_sizes) {
    reserve_nnz += bs * bs;
  }
  triplets.reserve(reserve_nnz);

  std::mt19937 rng(seed);
  std::normal_distribution<double> coeff(0.0, 1.0);

  std::vector<std::vector<int>> blocks;
  int offset = 0;
  for (const int block_size : block_sizes) {
    std::vector<int> vars(block_size);
    std::iota(vars.begin(), vars.end(), offset);
    blocks.push_back(vars);

    // Build a dense SPD block: B = R^T R + lambda I.
    Eigen::MatrixXd R(block_size, block_size);
    for (int i = 0; i < block_size; ++i) {
      for (int j = 0; j < block_size; ++j) {
        R(i, j) = coeff(rng);
      }
    }
    Eigen::MatrixXd B = R.transpose() * R;
    B.diagonal().array() += static_cast<double>(block_size);
    for (int i = 0; i < block_size; ++i) {
      for (int j = 0; j < block_size; ++j) {
        const int gi = offset + i;
        const int gj = offset + j;
        triplets.emplace_back(gi, gj, B(i, j));
      }
    }
    offset += block_size;
  }

  Eigen::SparseMatrix<double> matrix(n, n);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();
  std::vector<std::vector<int>> separators(blocks.size());
  std::vector<int> parent(blocks.size(), -1);
  return MatrixCase{name, matrix, blocks, blocks, separators, parent};
}

MatrixCase BuildArrowCase(const std::string& name, int num_leaves, int seed) {
  const int n = 1 + num_leaves;
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(3 * n);

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> off_noise(0.02, 0.05);
  const int center = 0;

  triplets.emplace_back(center, center, 10.0);
  for (int i = 1; i < n; ++i) {
    const double off = 0.25 + off_noise(rng);
    triplets.emplace_back(i, i, 2.0 + off_noise(rng));
    triplets.emplace_back(center, i, off);
    triplets.emplace_back(i, center, off);
  }

  Eigen::SparseMatrix<double> matrix(n, n);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();

  std::vector<std::vector<int>> cliques;
  std::vector<std::vector<int>> supernodes;
  std::vector<std::vector<int>> separators;
  std::vector<int> parent;
  cliques.reserve(n);
  supernodes.reserve(n);
  separators.reserve(n);
  parent.reserve(n);

  cliques.push_back({center});
  supernodes.push_back({center});
  separators.push_back({});
  parent.push_back(-1);
  for (int i = 1; i < n; ++i) {
    cliques.push_back({center, i});
    supernodes.push_back({i});
    separators.push_back({center});
    parent.push_back(0);
  }
  return MatrixCase{name, matrix, cliques, supernodes, separators, parent};
}

MatrixCase BuildSingleDenseCase(const std::string& name, int n, int seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> coeff(0.0, 1.0);
  Eigen::MatrixXd R(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      R(i, j) = coeff(rng);
    }
  }
  Eigen::MatrixXd B = R.transpose() * R;
  B.diagonal().array() += static_cast<double>(n);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(n * n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      triplets.emplace_back(i, j, B(i, j));
    }
  }
  Eigen::SparseMatrix<double> matrix(n, n);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  return MatrixCase{name, matrix, {vars}, {vars}, {{}}, {-1}};
}

std::vector<MatrixCase> SparseMatrixLibrary() {
  return {
      BuildBlockDiagonalCase("block_diag_4x80", {80, 80, 80, 80}, 7),
      BuildBlockDiagonalCase("block_diag_8x50",
                             {50, 50, 50, 50, 50, 50, 50, 50}, 11),
      BuildBlockDiagonalCase("block_diag_mixed", {120, 70, 40, 90}, 19),
      BuildArrowCase("arrow_star_1x160", 160, 23),
      BuildSingleDenseCase("single_dense_320", 320, 29),
  };
}

struct BenchmarkResult {
  double update_ms = 0;
  double assemble_tree_ms = 0;
  double assemble_ms = 0;
  double factor_ms = 0;
  double tree_ms = 0;
  double sparse_ms = 0;
  double dense_ms = 0;
};

BenchmarkResult BenchmarkCase(const MatrixCase& matrix_case, int num_threads) {
  const Eigen::MatrixXd dense = Eigen::MatrixXd(matrix_case.matrix);

  SymmetricLinearSystemTreeSolver tree_solver;
  tree_solver.SetNumThreads(num_threads);
  tree_solver.EnableAutoUpdateAtAssemble(true);
  tree_solver.SetFactorizationMode(true);

  std::vector<std::unique_ptr<StaticMatrixAssembler>> assemblers;
  std::vector<std::unique_ptr<KKTAssemblerToSubsystemAdapter>> adapters;
  assemblers.reserve(matrix_case.cliques.size());
  adapters.reserve(matrix_case.cliques.size());

  CliqueTree clique_tree;
  clique_tree.supernodes = matrix_case.supernodes;
  clique_tree.separators = matrix_case.separators;
  clique_tree.node_to_parent = matrix_case.node_to_parent;

  for (const auto& clique : matrix_case.cliques) {
    Eigen::MatrixXd local(clique.size(), clique.size());
    for (size_t i = 0; i < clique.size(); ++i) {
      for (size_t j = 0; j < clique.size(); ++j) {
        local(i, j) = dense(clique[i], clique[j]);
      }
    }
    assemblers.emplace_back(
        std::make_unique<StaticMatrixAssembler>(clique, local));
    auto adapter = std::make_unique<KKTAssemblerToSubsystemAdapter>(
        assemblers.back().get());
    auto* subsystem =
        adapter->create_subsystem(SubsystemType::kPositiveDefinite);
    tree_solver.AddSubsystem(subsystem);
    tree_solver.push_back(std::move(adapter));
  }
  tree_solver.Finalize(clique_tree);

  const int repeats = 6;

  double assemble_total_ms = 0.0;
  double update_total_ms = 0.0;
  double assemble_tree_total_ms = 0.0;
  double factor_total_ms = 0.0;
  tree_solver.EnableAutoUpdateAtAssemble(false);
  for (int i = 0; i < repeats; ++i) {
    const auto update_start = std::chrono::steady_clock::now();
    tree_solver.UpdateAssemblerData();
    const auto update_elapsed = std::chrono::steady_clock::now() - update_start;
    update_total_ms +=
        std::chrono::duration<double, std::milli>(update_elapsed).count();

    const auto assemble_tree_start = std::chrono::steady_clock::now();
    tree_solver.Assemble();
    const auto assemble_tree_elapsed =
        std::chrono::steady_clock::now() - assemble_tree_start;
    const double assemble_tree_ms =
        std::chrono::duration<double, std::milli>(assemble_tree_elapsed).count();
    assemble_tree_total_ms += assemble_tree_ms;
    const double assemble_ms =
        std::chrono::duration<double, std::milli>(update_elapsed).count() +
        assemble_tree_ms;
    assemble_total_ms +=
        assemble_ms;

    const auto factor_start = std::chrono::steady_clock::now();
    if (!tree_solver.Factor()) {
      ADD_FAILURE() << "Tree solver factorization failed for case "
                    << matrix_case.name;
      return BenchmarkResult{};
    }
    const auto factor_elapsed = std::chrono::steady_clock::now() - factor_start;
    factor_total_ms +=
        std::chrono::duration<double, std::milli>(factor_elapsed).count();
  }

  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
      sparse_solver;
  auto sparse_start = std::chrono::steady_clock::now();
  for (int i = 0; i < repeats; ++i) {
    sparse_solver.compute(matrix_case.matrix.selfadjointView<Eigen::Lower>());
    EXPECT_EQ(sparse_solver.info(), Eigen::Success);
  }
  auto sparse_elapsed = std::chrono::steady_clock::now() - sparse_start;

  auto dense_start = std::chrono::steady_clock::now();
  for (int i = 0; i < repeats; ++i) {
    Eigen::LDLT<Eigen::MatrixXd> dense_solver(dense);
    EXPECT_EQ(dense_solver.info(), Eigen::Success);
  }
  auto dense_elapsed = std::chrono::steady_clock::now() - dense_start;

  BenchmarkResult result;
  result.update_ms = update_total_ms;
  result.assemble_tree_ms = assemble_tree_total_ms;
  result.assemble_ms = assemble_total_ms;
  result.factor_ms = factor_total_ms;
  result.tree_ms = result.assemble_ms + result.factor_ms;
  result.sparse_ms =
      std::chrono::duration<double, std::milli>(sparse_elapsed).count();
  result.dense_ms =
      std::chrono::duration<double, std::milli>(dense_elapsed).count();
  return result;
}

TEST(KKTTreeSolver, SparseLibraryBenchmarkAgainstEigen) {
  const unsigned int cores = std::thread::hardware_concurrency();
  if (cores < 2) {
    SUCCEED()
        << "Skipping benchmark check: requires at least 2 hardware threads.";
    return;
  }
  const int num_threads = std::min<int>(4, static_cast<int>(cores));
  auto library = SparseMatrixLibrary();

  int tree_faster_cases = 0;
  for (const auto& matrix_case : library) {
    const auto result = BenchmarkCase(matrix_case, num_threads);
    std::cout << "Case=" << matrix_case.name
              << " update_ms=" << result.update_ms
              << " assemble_tree_ms=" << result.assemble_tree_ms
              << " assemble_ms=" << result.assemble_ms
              << " factor_ms=" << result.factor_ms << " tree_ms="
              << result.tree_ms << " sparse_ms=" << result.sparse_ms
              << " dense_ms=" << result.dense_ms << " sparse/tree="
              << (result.sparse_ms / result.tree_ms) << " dense/tree="
              << (result.dense_ms / result.tree_ms) << "\n";

    if (result.tree_ms < result.sparse_ms) {
      ++tree_faster_cases;
    }
  }
  EXPECT_GE(tree_faster_cases, 1);
}

}  // namespace
}  // namespace conex
