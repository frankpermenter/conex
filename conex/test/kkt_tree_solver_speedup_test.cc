#include <algorithm>
#include <chrono>
#include <memory>
#include <numeric>
#include <thread>
#include <vector>

#include "conex/kkt_tree_solver.h"
#include <gtest/gtest.h>

namespace conex {
namespace {

class SleepySubsystem final : public KKTSubsystemBase {
 public:
  explicit SleepySubsystem(std::chrono::milliseconds sleep_time,
                           std::chrono::milliseconds separator_read_sleep =
                               std::chrono::milliseconds(0))
      : sleep_time_(sleep_time), separator_read_sleep_(separator_read_sleep) {}

  Eigen::Ref<Eigen::MatrixXd> supernode_submatrix() override {
    return supernode_submatrix_;
  }
  Eigen::Ref<Eigen::MatrixXd> separator_schur_complement() override {
    if (separator_read_sleep_.count() > 0) {
      std::this_thread::sleep_for(separator_read_sleep_);
    }
    return separator_schur_complement_;
  }
  Eigen::Ref<Eigen::MatrixXd> separator_rows() override {
    return separator_rows_;
  }
  Eigen::Ref<const Eigen::MatrixXd> supernode_submatrix() const override {
    return supernode_submatrix_;
  }
  Eigen::Ref<const Eigen::MatrixXd> separator_schur_complement()
      const override {
    if (separator_read_sleep_.count() > 0) {
      std::this_thread::sleep_for(separator_read_sleep_);
    }
    return separator_schur_complement_;
  }
  Eigen::Ref<const Eigen::MatrixXd> separator_rows() const override {
    return separator_rows_;
  }

 private:
  void DoInitialize() override {
    supernode_submatrix_.setZero(supernodes().size(), supernodes().size());
    separator_rows_.setZero(separators().size(), supernodes().size());
    separator_schur_complement_.setZero(separators().size(),
                                        separators().size());
  }

  bool DoEliminateSupernodeColumns() override {
    std::this_thread::sleep_for(sleep_time_);
    return true;
  }

  void DoComputeSeparatorSchurComplement() override {}

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd>) const override {}

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd>) const override {}

  std::chrono::milliseconds sleep_time_;
  std::chrono::milliseconds separator_read_sleep_;
  Eigen::MatrixXd separator_schur_complement_;
  Eigen::MatrixXd supernode_submatrix_;
  Eigen::MatrixXd separator_rows_;
};

long MeasureElapsedMs(int num_roots, int num_threads,
                      std::chrono::milliseconds work_per_root) {
  SymmetricLinearSystemTreeSolver solver;
  solver.SetNumThreads(num_threads);

  std::vector<std::unique_ptr<SleepySubsystem>> subsystems;
  subsystems.reserve(num_roots);
  for (int i = 0; i < num_roots; ++i) {
    auto subsystem = std::make_unique<SleepySubsystem>(work_per_root);
    subsystem->SetSupernodes({i});
    subsystem->SetSeparators({});
    solver.AddSubsystem(subsystem.get());
    subsystems.emplace_back(std::move(subsystem));
  }

  solver.SetEliminationTree(std::vector<int>(num_roots, -1));

  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(solver.AssembleAndFactor());
  const auto elapsed = std::chrono::steady_clock::now() - start;
  return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
}

long MeasureStarAssembleMs(int num_leaves, int separator_size, int num_threads,
                           int repeats,
                           std::chrono::milliseconds separator_read_sleep) {
  SymmetricLinearSystemTreeSolver solver;
  solver.SetNumThreads(num_threads);

  std::vector<std::unique_ptr<SleepySubsystem>> subsystems;
  subsystems.reserve(num_leaves + 1);

  auto root = std::make_unique<SleepySubsystem>(std::chrono::milliseconds(0));
  std::vector<int> root_supernodes(separator_size);
  std::iota(root_supernodes.begin(), root_supernodes.end(), 0);
  root->SetSupernodes(root_supernodes);
  root->SetSeparators({});
  solver.AddSubsystem(root.get());
  subsystems.emplace_back(std::move(root));

  for (int i = 0; i < num_leaves; ++i) {
    auto leaf = std::make_unique<SleepySubsystem>(std::chrono::milliseconds(0),
                                                  separator_read_sleep);
    leaf->SetSupernodes({separator_size + i});
    leaf->SetSeparators(root_supernodes);
    solver.AddSubsystem(leaf.get());
    subsystems.emplace_back(std::move(leaf));
  }

  std::vector<int> parent(num_leaves + 1, 0);
  parent[0] = -1;
  solver.SetEliminationTree(parent);
  solver.ComputeSeparatorOffsets();

  long elapsed_total_ms = 0;
  for (int i = 0; i < repeats; ++i) {
    const auto start = std::chrono::steady_clock::now();
    solver.Assemble();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    elapsed_total_ms +=
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  }
  return elapsed_total_ms;
}

TEST(KKTTreeSolver, MultiThreadAssembleAndFactorSpeedup) {
  const unsigned int cores = std::thread::hardware_concurrency();
  if (cores < 2) {
    SUCCEED()
        << "Skipping speedup check: requires at least 2 hardware threads.";
    return;
  }

  const int num_roots = 8;
  const auto work_per_root = std::chrono::milliseconds(40);
  const int parallel_threads = std::min<int>(4, static_cast<int>(cores));

  const long single_thread_ms = MeasureElapsedMs(num_roots, 1, work_per_root);
  const long multi_thread_ms =
      MeasureElapsedMs(num_roots, parallel_threads, work_per_root);

  EXPECT_LT(multi_thread_ms, static_cast<long>(single_thread_ms * 0.85));
  EXPECT_GT(single_thread_ms - multi_thread_ms, work_per_root.count());
}

TEST(KKTTreeSolver, MultiThreadSingleTreeStarSpeedup) {
  const unsigned int cores = std::thread::hardware_concurrency();
  if (cores < 2) {
    SUCCEED()
        << "Skipping speedup check: requires at least 2 hardware threads.";
    return;
  }

  const int parallel_threads = std::min<int>(4, static_cast<int>(cores));
  const int num_leaves = 10;
  const int separator_size = 120;
  const int repeats = 3;
  const auto per_read_sleep = std::chrono::milliseconds(8);

  const long single_thread_ms = MeasureStarAssembleMs(
      num_leaves, separator_size, 1, repeats, per_read_sleep);
  const long multi_thread_ms = MeasureStarAssembleMs(
      num_leaves, separator_size, parallel_threads, repeats, per_read_sleep);

  EXPECT_LT(multi_thread_ms, static_cast<long>(single_thread_ms * 0.8));
  EXPECT_GT(single_thread_ms - multi_thread_ms, 30);
}

TEST(KKTTreeSolver, RejectsNonContiguousSupernodesInSolve) {
  SymmetricLinearSystemTreeSolver solver;
  solver.SetNumThreads(1);

  auto subsystem = std::make_unique<SleepySubsystem>(std::chrono::milliseconds(0));
  subsystem->SetSupernodes({0, 2});
  subsystem->SetSeparators({});
  solver.AddSubsystem(subsystem.get());

  solver.SetEliminationTree({-1});
  ASSERT_TRUE(solver.AssembleAndFactor());

  Eigen::MatrixXd rhs(3, 1);
  rhs.setZero();
  EXPECT_THROW((void)solver.Solve(rhs, false), std::runtime_error);
}

}  // namespace
}  // namespace conex
