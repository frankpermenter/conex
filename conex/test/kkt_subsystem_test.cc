#include "conex/kkt_subsystem.h"
#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
#include "conex/kkt_solver_interface.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {
#define CONEX_NOOP(x) (void) x;
namespace {
MatrixXd IncrementSubmatrix(const MatrixXd& full_matrix,
                            const MatrixXd& sub_matrix,
                            const std::vector<int>& c) {
  MatrixXd y = full_matrix;
  int i = 0;
  for (auto ci : c) {
    int j = 0;
    for (auto cj : c) {
      y(ci, cj) += sub_matrix(i, j);
      j++;
    }
    i++;
  }
  return y;
}
}

class TreeSolver : public KKTSolverBase {
 public:
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool permute_to_elimination_order) const {
    for (auto root : roots_) {
      root->ApplyInverseOfLeftFactor(b);
      root->ApplyInverseOfRightFactor(b);
    }
  }

  void DoAssemble() override { 
    for (auto root : roots_) {
      root->Assemble(); 
    }
  }

  bool DoFactor() override {
    for (auto root : roots_) {
      root->AssembleAndFactor();
    }
    return true;
  }

  void MakeTree(std::vector<int> parent) {
    for (size_t i = 0; i < parent.size(); ++i) {
      if (parent[i] > 0) {
        subsystems_.at(parent[i])->AddChild(subsystems_.at(i));
      } else {
        roots_.push_back(subsystems_.at(i));
      }
    }
    // Post-order
    std::vector<int> variable_to_elimination_position(number_of_variables());
    int first = 0;
    for (auto r : roots_) {
      r->ComputePostOrdering(first, &variable_to_elimination_position);
    }
    for (auto s : subsystems_) {
      s->SetVariableOrdering(variable_to_elimination_position);
    }
  }

  int number_of_variables() const {
    int max = 0;
    for (auto s : subsystems_) {
      const auto& sn =  s->supernodes();
      double max_s = *std::max_element(sn.begin(), sn.end());
      if (max_s > max) {
        max = max_s;
      }
    }
    return max + 1;
  }
  Eigen::MatrixXd DoKKTMatrix(bool permute_to_elimination_order = true) const {
    int num_vars = number_of_variables(); 
    MatrixXd M(num_vars, num_vars);
    M.setZero();
    for (auto root : roots_) {
      root->MakeKKTMatrix(&M);
    }
    return M.selfadjointView<Eigen::Lower>();
  }

  void AddSubsystem(KKTSubsystem* system) { 
    subsystems_.push_back(system);
  }
  std::vector<KKTSubsystem*> roots_;
  std::vector<KKTSubsystem*> subsystems_;
};

using Eigen::MatrixXd;

template<typename FactorizationMethod, bool schur_complement_mode>
class CholeskySolver : public KKTSubsystem {
 public:
  CholeskySolver(std::vector<int> vars) : KKTSubsystem(vars, 0) {}

  void DoEliminateSupernodeColumns() override {
    llt_.compute(supernode_submatrix_);
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    if constexpr (schur_complement_mode) {
      llt_.solveInPlace(y);
    } else {
      llt_.matrixL().solveInPlace(y);
    }
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    if constexpr (schur_complement_mode) {
      CONEX_NOOP(y);
      return;
    } else {
      llt_.matrixL().transpose().solveInPlace(y);
    }
  }

  void DoComputeSeparatorSchurComplement() override {
    separator_schur_complement_ -=
        separator_rows_ * llt_.solve(separator_rows_.transpose());
  }

  FactorizationMethod llt_;
};

using LLTSolver = CholeskySolver<Eigen::LLT<Eigen::MatrixXd>, true>;
class QuadraticCost : public LLTSolver {
 public:
  QuadraticCost(Eigen::MatrixXd Q, std::vector<int> vars)
      : LLTSolver(vars), Q_(Q) {}

  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    int n1 = supernode_submatrix_.rows();
    int n2 = separator_rows_.rows();
    Q_in_elimination_order_.resize(n1 + n2, n1 + n2);
    AssignSubmatrix(Q_, Q_in_elimination_order_, variable_to_local_elimination_rank());
    DoAssemble();
  }

 private:
  void DoAssemble() {
    int n1 = supernode_submatrix_.rows();
    int n2 = separator_rows_.rows();
    supernode_submatrix_ = Q_in_elimination_order_.topLeftCorner(n1, n1);
    separator_rows_ = Q_in_elimination_order_.bottomLeftCorner(n2, n1);
    separator_schur_complement_ =
        Q_in_elimination_order_.bottomRightCorner(n2, n2);
  }
  void AssignSubmatrix(const Eigen::MatrixXd& source, 
                       Eigen::Ref<Eigen::MatrixXd> destination,
                       const std::vector<int>& destination_to_source_index) {
    DUMP(destination_to_source_index);
    for (int i = 0; i < source.rows(); i++) {
      for (int j = 0; j < source.cols(); j++) {
        destination(i, j) = source(destination_to_source_index.at(i),  
                                   destination_to_source_index.at(j));
      }
    }
  }
 private:
  Eigen::MatrixXd Q_in_elimination_order_;
  Eigen::MatrixXd Q_;
};

// 1 1 1
// 1 1 1
// 1 1 1 1 1
//     1 1 1
//     1 1 1
#if 0
GTEST_TEST(KKTSubsystem, TestConstruction) {

  std::vector<int> vars{0, 1, 2};
  Eigen::MatrixXd Q(3, 3);
  Q << 10, 2, 3,
       2, 10, 4,
       3, 4, 10;
  QuadraticCost q1(Q, vars); q1.SetSeparators({2}); q1.SetSupernodes({0, 1});

  std::vector<int> vars_2{2, 3, 4};
  QuadraticCost q2(Q, vars); q2.SetSupernodes({2, 3, 4});

  q1.DoInitialize();
  q2.DoInitialize();

  q2.AddChild(&q1);
  q2.AssembleAndFactor(true);
  return;

  Eigen::MatrixXd Q_full(5, 5);
  Q_full.setZero();
  Q_full.topLeftCorner(3, 3) = Q;
  Q_full.bottomRightCorner(3, 3) += Q;
  DUMP(Q_full);
  Eigen::LLT<Eigen::MatrixXd> llt(Q_full);
  Eigen::MatrixXd L = llt.matrixL();
  DUMP(L);
  VectorXd x_ref(5);
  x_ref.setLinSpaced(5, -1, 1);
  MatrixXd b = Q_full * x_ref;
  DUMP(L.triangularView<Eigen::Lower>().solve(b));
  KKTSystem system; system.root = &q2;
  system.SolveInPlace(&b);
  DUMP(b);
}
#endif

GTEST_TEST(KKTSubsystem, TestTrivialExample) {
  int num_vars = 5;
  MatrixXd full_matrix = MatrixXd::Zero(num_vars, num_vars);
  std::vector<int> vars{0, 1, 2};
  Eigen::MatrixXd Q1(3, 3);
  // clang-format off
  Q1 << 50, 2, 3,
        2, 10, 4,
        3, 4, 10;
  // clang-format on
  QuadraticCost q1(Q1, vars);
  q1.SetSeparators({2});
  q1.SetSupernodes({0, 1});
  full_matrix = IncrementSubmatrix(full_matrix, Q1, vars);

  std::vector<int> vars_2{2, 3};
  Eigen::MatrixXd Q2(2, 2);
  // clang-format off
  Q2 << 5, 2, 
        2, 5;
  // clang-format on
  QuadraticCost q2(Q2, vars_2);
  q2.SetSupernodes({2});
  q2.SetSeparators({3});
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars_2);

  std::vector<int> vars_3{3, 4};
  QuadraticCost q3(Q2, vars_3);
  q3.SetSupernodes({3, 4});
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars_3);

  TreeSolver system;
  system.AddSubsystem(&q1);
  system.AddSubsystem(&q2);
  system.AddSubsystem(&q3);
  std::vector<int> parent{1, 2, -1};
  system.MakeTree(parent);

  q1.DoInitialize();
  q2.DoInitialize();
  q3.DoInitialize();

  EXPECT_EQ(q1.parent(), &q2);
  EXPECT_EQ(q2.parent(), &q3);

  system.Assemble();
  DUMP(system.KKTMatrix());
  EXPECT_NEAR((system.KKTMatrix() - full_matrix).norm(), 0, 1e-14);

  Eigen::LLT<Eigen::MatrixXd> llt(full_matrix);
  Eigen::MatrixXd L = llt.matrixL();
  VectorXd x_ref(num_vars);
  x_ref.setLinSpaced(5, -1, 1);
  MatrixXd b = full_matrix * x_ref;
  system.Factor();
  system.SolveInPlace(b);
  EXPECT_NEAR((x_ref - b).norm(), 0, 1e-12);
}

}  // namespace conex
