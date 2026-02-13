#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "conex/clique_ordering.h"
#include "conex/kkt_tree_solver.h"
#include "conex/static_subsystem.h"
#include "conex/workspace.h"
#include <Eigen/SparseQR>

namespace conex {
namespace {

using RowSparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;

class RowPartitionAssembler final : public SupernodalAssemblerBase {
 public:
  RowPartitionAssembler(const RowSparseMatrix& A, const std::vector<int>& rows,
                        const std::vector<int>& clique_variables)
      : SupernodalAssemblerBase(clique_variables),
        local_matrix_(clique_variables.size(), clique_variables.size()) {
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
    submatrix_data_.G.setZero();
    submatrix_data_.G.triangularView<Eigen::Lower>() =
        local_matrix_.triangularView<Eigen::Lower>();
  }

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
  std::vector<std::vector<int>> partitions(num_parts);
  for (int r = 0; r < num_rows; ++r) {
    partitions.at(r % num_parts).push_back(r);
  }
  return partitions;
}

RowSparseMatrix BuildSparseLeastSquaresMatrix(int n, int bandwidth) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(12 * n);
  int row = 0;

  // Identity rows guarantee rank.
  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(row, i, 2.0);
    ++row;
  }

  // Overlapping rows control induced clique width.
  for (int i = 0; i + bandwidth < n; ++i) {
    for (int j = 0; j <= bandwidth; ++j) {
      triplets.emplace_back(row, i + j, (j % 2 == 0 ? 0.6 : -0.4));
    }
    ++row;
  }
  for (int i = 0; i + bandwidth < n; ++i) {
    for (int j = 0; j <= bandwidth; ++j) {
      triplets.emplace_back(row, i + j, 0.2 + 0.05 * j);
    }
    ++row;
  }

  RowSparseMatrix A(row, n);
  A.setFromTriplets(triplets.begin(), triplets.end());
  A.makeCompressed();
  return A;
}

int ComputeTreeWidth(const CliqueTree& clique_tree) {
  int tree_width = 0;
  for (size_t i = 0; i < clique_tree.supernodes.size(); ++i) {
    int clique_size = static_cast<int>(clique_tree.supernodes.at(i).size() +
                                       clique_tree.separators.at(i).size());
    tree_width = std::max(tree_width, clique_size - 1);
  }
  return tree_width;
}

struct Metrics {
  std::string family;
  int n = 0;
  int m = 0;
  int nnz = 0;
  int nnz_ata = 0;
  double ata_density = 0;
  int bandwidth = 0;
  int tree_width = 0;
  int num_cliques = 0;
  double tree_ms = 0;
  double qr_ms = 0;
  double dense_ms = 0;
  double speedup_qr_over_tree = 0;
  double speedup_dense_over_tree = 0;
  double rel_diff_tree_vs_qr = 0;
  double rel_diff_dense_vs_qr = 0;
  double residual_tree = 0;
  double residual_qr = 0;
  double residual_dense = 0;
};

RowSparseMatrix BuildStarLeastSquaresMatrix(int n) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(3 * n);
  int row = 0;
  // Identity rows.
  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(row, i, 1.8);
    ++row;
  }
  // Star couplings centered at variable 0.
  for (int i = 1; i < n; ++i) {
    triplets.emplace_back(row, 0, 0.7);
    triplets.emplace_back(row, i, 0.5);
    ++row;
  }
  RowSparseMatrix A(row, n);
  A.setFromTriplets(triplets.begin(), triplets.end());
  A.makeCompressed();
  return A;
}

Metrics BenchmarkInstance(const std::string& family, int n, int bandwidth,
                          int row_partitions, int repeats) {
  Metrics metrics;
  metrics.family = family;
  metrics.n = n;
  metrics.bandwidth = bandwidth;

  RowSparseMatrix A =
      (family == "star" ? BuildStarLeastSquaresMatrix(n)
                        : BuildSparseLeastSquaresMatrix(n, bandwidth));
  metrics.m = A.rows();
  metrics.nnz = A.nonZeros();
  const Eigen::SparseMatrix<double> A_col_major = A;
  const Eigen::SparseMatrix<double> AtA_sparse =
      (A_col_major.transpose() * A_col_major).pruned(1e-12);
  metrics.nnz_ata = AtA_sparse.nonZeros();
  metrics.ata_density = static_cast<double>(metrics.nnz_ata) /
                        static_cast<double>(n) / static_cast<double>(n);

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
  metrics.tree_width = ComputeTreeWidth(clique_tree);
  metrics.num_cliques = static_cast<int>(cliques.size());
  if (family == "star") {
    if (metrics.tree_width > 1) {
      throw std::runtime_error("Star sanity check failed: tree width > 1.");
    }
    if (metrics.ata_density > 0.1) {
      throw std::runtime_error("Star sanity check failed: A^T A too dense.");
    }
  }

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
  const Eigen::MatrixXd AtA = Eigen::MatrixXd(A.transpose() * A);
  Eigen::MatrixXd rhs_mat(rhs.rows(), 1);
  rhs_mat.col(0) = rhs;

  double best_tree_ms = std::numeric_limits<double>::infinity();
  Eigen::VectorXd x_tree_best;
  for (int i = 0; i < repeats; ++i) {
    auto start = std::chrono::steady_clock::now();
    tree_solver.Assemble();
    if (!tree_solver.Factor()) {
      throw std::runtime_error("Tree solver factorization failed.");
    }
    const Eigen::MatrixXd x_tree = tree_solver.Solve(rhs_mat, true);
    auto elapsed = std::chrono::steady_clock::now() - start;
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(elapsed).count();
    if (elapsed_ms < best_tree_ms) {
      best_tree_ms = elapsed_ms;
      x_tree_best = x_tree.col(0);
    }
  }
  metrics.tree_ms = best_tree_ms;

  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
  double best_qr_ms = std::numeric_limits<double>::infinity();
  Eigen::VectorXd x_qr_best;
  for (int i = 0; i < repeats; ++i) {
    auto start = std::chrono::steady_clock::now();
    qr.compute(A_col_major);
    if (qr.info() != Eigen::Success) {
      throw std::runtime_error("SparseQR factorization failed.");
    }
    const Eigen::VectorXd x_qr = qr.solve(b);
    if (qr.info() != Eigen::Success) {
      throw std::runtime_error("SparseQR solve failed.");
    }
    auto elapsed = std::chrono::steady_clock::now() - start;
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(elapsed).count();
    if (elapsed_ms < best_qr_ms) {
      best_qr_ms = elapsed_ms;
      x_qr_best = x_qr;
    }
  }
  metrics.qr_ms = best_qr_ms;

  double best_dense_ms = std::numeric_limits<double>::infinity();
  Eigen::VectorXd x_dense_best;
  for (int i = 0; i < repeats; ++i) {
    auto start = std::chrono::steady_clock::now();
    Eigen::LDLT<Eigen::MatrixXd> ldlt(AtA);
    if (ldlt.info() != Eigen::Success) {
      throw std::runtime_error("Dense LDLT factorization failed.");
    }
    const Eigen::VectorXd x_dense = ldlt.solve(rhs);
    if (ldlt.info() != Eigen::Success) {
      throw std::runtime_error("Dense LDLT solve failed.");
    }
    auto elapsed = std::chrono::steady_clock::now() - start;
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(elapsed).count();
    if (elapsed_ms < best_dense_ms) {
      best_dense_ms = elapsed_ms;
      x_dense_best = x_dense;
    }
  }
  metrics.dense_ms = best_dense_ms;

  metrics.speedup_qr_over_tree = metrics.qr_ms / metrics.tree_ms;
  metrics.speedup_dense_over_tree = metrics.dense_ms / metrics.tree_ms;

  metrics.rel_diff_tree_vs_qr =
      (x_tree_best - x_qr_best).norm() / std::max(1.0, x_qr_best.norm());
  metrics.rel_diff_dense_vs_qr =
      (x_dense_best - x_qr_best).norm() / std::max(1.0, x_qr_best.norm());

  metrics.residual_tree =
      (A * x_tree_best - b).norm() / std::max(1.0, b.norm());
  metrics.residual_qr = (A * x_qr_best - b).norm() / std::max(1.0, b.norm());
  metrics.residual_dense =
      (A * x_dense_best - b).norm() / std::max(1.0, b.norm());

  if (!(metrics.rel_diff_tree_vs_qr < 1e-4 &&
        metrics.rel_diff_dense_vs_qr < 1e-4 && metrics.residual_tree < 1e-5 &&
        metrics.residual_qr < 1e-8 && metrics.residual_dense < 1e-5)) {
    throw std::runtime_error("Solver answer mismatch across tree/qr/dense.");
  }
  return metrics;
}

Metrics RunBenchmarkInstance(int n, int bandwidth, int row_partitions,
                             int repeats) {
  return BenchmarkInstance("band", n, bandwidth, row_partitions, repeats);
}

Metrics RunStarBenchmarkInstance(int n, int row_partitions, int repeats) {
  return BenchmarkInstance("star", n, 1, row_partitions, repeats);
}

}  // namespace
}  // namespace conex

int main(int argc, char** argv) {
  std::string output_csv = "conex/test/least_squares_speedup.csv";
  if (argc > 1) {
    output_csv = argv[1];
  }

  const std::vector<int> n_values = {80, 120, 160, 220, 300, 420, 600, 900};
  const std::vector<int> bandwidth_values = {1, 2, 3, 5, 8};
  const int row_partitions = 6;
  const int repeats = 4;

  std::ofstream out(output_csv);
  if (!out.is_open()) {
    std::cerr << "Failed to open output file: " << output_csv << "\n";
    return 1;
  }
  out << "family,n,m,nnz,nnz_ata,ata_density,bandwidth,tree_width,num_cliques,"
      << "tree_ms,qr_ms,dense_ms,"
      << "speedup_qr_over_tree,speedup_dense_over_tree,rel_diff_tree_vs_qr,"
      << "rel_diff_dense_vs_qr,residual_tree,residual_qr,residual_dense\n";

  std::cout << std::fixed << std::setprecision(4);
  try {
    for (int n : n_values) {
      for (int bw : bandwidth_values) {
        if (bw >= n) {
          continue;
        }
        auto metrics =
            conex::RunBenchmarkInstance(n, bw, row_partitions, repeats);
        out << metrics.family << "," << metrics.n << "," << metrics.m << ","
            << metrics.nnz << "," << metrics.nnz_ata << ","
            << metrics.ata_density << "," << metrics.bandwidth << ","
            << metrics.tree_width << "," << metrics.num_cliques << ","
            << metrics.tree_ms << "," << metrics.qr_ms << ","
            << metrics.dense_ms << "," << metrics.speedup_qr_over_tree << ","
            << metrics.speedup_dense_over_tree << ","
            << metrics.rel_diff_tree_vs_qr << ","
            << metrics.rel_diff_dense_vs_qr << "," << metrics.residual_tree
            << "," << metrics.residual_qr << "," << metrics.residual_dense
            << "\n";
        std::cout << metrics.family << " n=" << metrics.n
                  << " bw=" << metrics.bandwidth << " nnz=" << metrics.nnz
                  << " nnz(AtA)=" << metrics.nnz_ata
                  << " dens(AtA)=" << metrics.ata_density
                  << " tree_width=" << metrics.tree_width
                  << " qr/tree=" << metrics.speedup_qr_over_tree
                  << " dense/tree=" << metrics.speedup_dense_over_tree
                  << " rel(tree,qr)=" << metrics.rel_diff_tree_vs_qr
                  << " rel(dense,qr)=" << metrics.rel_diff_dense_vs_qr << "\n";
      }
      auto star_metrics =
          conex::RunStarBenchmarkInstance(n, row_partitions, repeats);
      out << star_metrics.family << "," << star_metrics.n << ","
          << star_metrics.m << "," << star_metrics.nnz << ","
          << star_metrics.nnz_ata << "," << star_metrics.ata_density << ","
          << star_metrics.bandwidth << "," << star_metrics.tree_width << ","
          << star_metrics.num_cliques << "," << star_metrics.tree_ms << ","
          << star_metrics.qr_ms << "," << star_metrics.dense_ms << ","
          << star_metrics.speedup_qr_over_tree << ","
          << star_metrics.speedup_dense_over_tree << ","
          << star_metrics.rel_diff_tree_vs_qr << ","
          << star_metrics.rel_diff_dense_vs_qr << ","
          << star_metrics.residual_tree << "," << star_metrics.residual_qr
          << "," << star_metrics.residual_dense << "\n";
      std::cout << star_metrics.family << " n=" << star_metrics.n
                << " nnz=" << star_metrics.nnz
                << " nnz(AtA)=" << star_metrics.nnz_ata
                << " dens(AtA)=" << star_metrics.ata_density
                << " tree_width=" << star_metrics.tree_width
                << " qr/tree=" << star_metrics.speedup_qr_over_tree
                << " dense/tree=" << star_metrics.speedup_dense_over_tree
                << " rel(tree,qr)=" << star_metrics.rel_diff_tree_vs_qr
                << " rel(dense,qr)=" << star_metrics.rel_diff_dense_vs_qr
                << "\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "Benchmark failed: " << e.what() << "\n";
    return 2;
  }

  std::cout << "Wrote: " << output_csv << "\n";
  return 0;
}
