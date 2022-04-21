#include "conex/kkt_subsystem.h"
#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
#include "conex/kkt_solver_interface.h"
#include "conex/kkt_tree_solver.h"
#include "conex/RLDLT.h"
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

class LUSolver : public KKTSubsystem {
 public:
  LUSolver(std::vector<int> vars) : KKTSubsystem(vars, 0) {}

  void DoEliminateSupernodeColumns() override {
    lu_.compute(supernode_submatrix_);
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
      y = lu_.solve(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
      CONEX_NOOP(y);
  }

  void DoComputeSeparatorSchurComplement() override {
    separator_schur_complement_ -=
        separator_rows_ * lu_.solve(separator_rows_.transpose());
  }

  Eigen::PartialPivLU<Eigen::MatrixXd> lu_;
};

using LLTSolver = CholeskySolver<Eigen::RLDLT<Eigen::MatrixXd>, true>;

template<bool is_positive_definite>
using FactorizationMethod = typename std::conditional<is_positive_definite, LLTSolver, LUSolver>::type;

template<bool is_positive_definite>
class StaticSubsystem : public FactorizationMethod<is_positive_definite>  {
  using Base = FactorizationMethod<is_positive_definite>;
 public:
  StaticSubsystem(Eigen::MatrixXd Q, std::vector<int> vars)
      : Base(vars), Q_(Q) {}

  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    int n1 = Base::supernode_submatrix_.rows();
    int n2 = Base::separator_rows_.rows();
    Q_in_elimination_order_.resize(n1 + n2, n1 + n2);
    AssignSubmatrix(Q_, Q_in_elimination_order_, Base::variable_to_local_elimination_rank());
    DoAssemble();
  }

 private:
  void DoAssemble() {
    int n1 = Base::supernode_submatrix_.rows();
    int n2 = Base::separator_rows_.rows();
    Base::supernode_submatrix_ = Q_in_elimination_order_.topLeftCorner(n1, n1);
    Base::separator_rows_ = Q_in_elimination_order_.bottomLeftCorner(n2, n1);
    Base::separator_schur_complement_ =
        Q_in_elimination_order_.bottomRightCorner(n2, n2);
  }
  void AssignSubmatrix(const Eigen::MatrixXd& source, 
                       Eigen::Ref<Eigen::MatrixXd> destination,
                       const std::vector<int>& source_to_dest_index) {
    destination.setZero();
    for (int i = 0; i < source.rows(); i++) {
      for (int j = 0; j < source.cols(); j++) {
            destination(source_to_dest_index.at(i), 
                        source_to_dest_index.at(j)) = source(i, j);
      }
    }
  }
 private:
  Eigen::MatrixXd Q_in_elimination_order_;
  Eigen::MatrixXd Q_;
};

template<typename StaticAssemblerType>
void DoTestTrivalExample(const std::vector<int>& v) {
  int num_vars = 5;
  std::vector<int> vars{v[0], v[1], v[2]};
  MatrixXd full_matrix = MatrixXd::Zero(num_vars, num_vars);
  Eigen::MatrixXd Q1(3, 3);
  // clang-format off
  Q1 << 50, 2, 3,
        2, 10, 4,
        3, 4, 10;
  Q1 << 1, 1, 1,
        1, 1, 1,
        1, 1, 0;
  // clang-format on
  StaticAssemblerType q1(Q1, vars);
  q1.SetSeparators({v[1]});
  q1.SetSupernodes({v[0], v[2]});
  full_matrix = IncrementSubmatrix(full_matrix, Q1, vars);

  std::vector<int> vars_2{v[1], v[3]};
  Eigen::MatrixXd Q2(2, 2);
  // clang-format off
  Q2 << 5, 2, 
        2, 5;
  // clang-format on
  StaticAssemblerType q2(Q2, vars_2);
  q2.SetSupernodes({v[1]});
  q2.SetSeparators({v[3]});
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars_2);

  std::vector<int> vars_3{v[3], v[4]};
  StaticAssemblerType q3(Q2, vars_3);
  q3.SetSupernodes({v[3], v[4]});
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars_3);

  SymmetricLinearSystemTreeSolver system;
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

template<typename StaticAssemblerType>
void DoFailLDLT(bool expect_fail) {
  int num_vars = 3;
  std::vector<int> vars{0, 1, 2};
  MatrixXd full_matrix = MatrixXd::Zero(num_vars, num_vars);
  Eigen::MatrixXd Q1(3, 3);
  // clang-format off
  Q1 << 1, 0,  1,
        0, 0, -1,
        1, -1, 0;
  // clang-format on
  StaticAssemblerType q1(Q1, vars);
  q1.SetSupernodes({0,1,2});
  full_matrix = IncrementSubmatrix(full_matrix, Q1, vars);

  SymmetricLinearSystemTreeSolver system;
  system.AddSubsystem(&q1);
  std::vector<int> parent{-1};
  system.MakeTree(parent);

  q1.DoInitialize();

  VectorXd x_ref(num_vars);
  x_ref.setLinSpaced(num_vars, -1, 1);
  MatrixXd b = full_matrix * x_ref;
  system.Factor();
  system.SolveInPlace(b);

  if (expect_fail) {
    EXPECT_TRUE((x_ref - b).norm() > 1e-7);
  } else {
    EXPECT_NEAR((x_ref - b).norm(), 0, 1e-12);
  }
}

GTEST_TEST(KKTSubsystem, FailLDLT) {
  DoFailLDLT<StaticSubsystem<true>>(true /*expect_fail*/);
  DoFailLDLT<StaticSubsystem<false>>(false /*expect_fail*/);
}


GTEST_TEST(KKTSubsystem, TestTrivialExampleNominalOrder) {
  std::vector<int> v{0, 1, 2, 3, 4};
  //DoTestTrivalExample<StaticSubsystem<true>>(v);
  DoTestTrivalExample<StaticSubsystem<false>>(v);
  DoTestTrivalExample<StaticSubsystem<true>>(v);

}
GTEST_TEST(KKTSubsystem, TestTrivialExampleArbitrarilyPermutedOrder) {
  std::vector<int> v{2, 0, 1, 4, 3};
  DoTestTrivalExample<StaticSubsystem<true>>(v);
  DoTestTrivalExample<StaticSubsystem<false>>(v);
}

GTEST_TEST(KKTSubsystem, TestTrivialExampleReverseOrder) {
  std::vector<int> v{4, 3, 2, 1, 0};
  DoTestTrivalExample<StaticSubsystem<true>>(v);
  DoTestTrivalExample<StaticSubsystem<false>>(v);
}

}  // namespace conex
