#include "conex/kkt_subsystem.h"

#include <iostream>
#include <map>
#include <tuple>

#include "conex/RLDLT.h"
#include "conex/debug_macros.h"
#include "conex/kkt_solver_interface.h"
#include "conex/cholesky_solvers.h"
#include "conex/kkt_tree_solver.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {
#define CONEX_NOOP(x) (void)x;
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
}  // namespace

using Eigen::MatrixXd;


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

template <bool is_positive_definite>
using FactorizationMethod =
    typename std::conditional<is_positive_definite, LLTSolver, LUSolver>::type;

template <bool is_positive_definite>
class StaticSubsystem : public FactorizationMethod<is_positive_definite> {
  using Base = FactorizationMethod<is_positive_definite>;

 public:
  StaticSubsystem(Eigen::MatrixXd Q, std::vector<int> vars)
      : Base(vars), Q_(Q) {}

  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    int n1 = Base::supernode_submatrix_.rows();
    int n2 = Base::separator_rows_.rows();
    Q_in_elimination_order_.resize(n1 + n2, n1 + n2);
    AssignSubmatrix(Q_, Q_in_elimination_order_,
                    Base::variable_to_local_elimination_rank());
    DoAssemble();
  }

 private:
  bool DoIsValidLeaf() override { return Q_.diagonal().norm() > 0; }
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
        destination(source_to_dest_index.at(i), source_to_dest_index.at(j)) =
            source(i, j);
      }
    }
  }

 private:
  Eigen::MatrixXd Q_in_elimination_order_;
  Eigen::MatrixXd Q_;
};

template <typename StaticAssemblerType>
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
        1, 2, 1,
        1, 1, 3;
  // clang-format on
  StaticAssemblerType q1(Q1, vars);
  full_matrix = IncrementSubmatrix(full_matrix, Q1, vars);

  std::vector<int> vars_2{v[1], v[3]};
  Eigen::MatrixXd Q2(2, 2);
  // clang-format off
  Q2 << 5, 2, 
        2, 5;
  // clang-format on
  StaticAssemblerType q2(Q2, vars_2);
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars_2);

  std::vector<int> vars_3{v[3], v[4]};
  StaticAssemblerType q3(Q2, vars_3);
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars_3);

  SymmetricLinearSystemTreeSolver system;
  system.AddSubsystem(&q1);
  system.AddSubsystem(&q2);
  system.AddSubsystem(&q3);
  std::vector<int> parent{1, 2, -1};
  system.Finalize(parent);

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

template <typename StaticAssemblerType>
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
  full_matrix = IncrementSubmatrix(full_matrix, Q1, vars);

  SymmetricLinearSystemTreeSolver system;
  system.AddSubsystem(&q1);
  std::vector<int> parent{-1};
  Options options; 
  system.Finalize(parent, false);

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
  // DoTestTrivalExample<StaticSubsystem<true>>(v);
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

std::vector<std::unique_ptr<KKTSubsystem>> MakeTestSystem(
    std::vector<Eigen::MatrixXd>& matrices,
    const std::vector<std::vector<int>>& vars) {
  std::vector<std::unique_ptr<KKTSubsystem>> subsystems;
  for (size_t i = 0; i < matrices.size(); i++) {
    subsystems.emplace_back(
        std::make_unique<StaticSubsystem<false>>(matrices.at(i), vars.at(i)));
  }
  return subsystems;
}

template <typename StaticAssemblerType>
void DoBadRoot() {
  int num_vars = 3;
  Eigen::MatrixXd full_matrix(num_vars, num_vars);
  full_matrix.setZero();

  // clang-format off
  std::vector<int> vars1{0, 1};
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(vars1.size(), vars1.size());
  StaticAssemblerType q1(Q1, vars1);
  full_matrix = IncrementSubmatrix(full_matrix, Q1, vars1);

  std::vector<int> vars2{0, 1, 2};
  Eigen::MatrixXd Q2(3, 3);
  Q2 << 0, 0, 1,
        0, 0, 1,
        1, 1, 0;
  StaticAssemblerType q2(Q2, vars2);
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars2);
  // clang-format on

  SymmetricLinearSystemTreeSolver system;
  system.AddSubsystem(&q1);
  system.AddSubsystem(&q2);

  std::vector<int> parent_zero_pivot_error{-1, 0};
  EXPECT_THROW({ system.Finalize(parent_zero_pivot_error); },
               std::runtime_error);

  std::vector<int> parent_self_parent_error{0, 1};
  EXPECT_THROW({ system.Finalize(parent_self_parent_error); },
               std::runtime_error);

  std::vector<int> parent_valid{1, -1};
  EXPECT_NO_THROW({ system.Finalize(parent_valid); });

  q1.DoInitialize();
  q2.DoInitialize();

  VectorXd x_ref(num_vars);
  x_ref.setLinSpaced(num_vars, -1, 1);
  MatrixXd b = full_matrix * x_ref;
  system.Factor();
  system.SolveInPlace(b);

  EXPECT_NEAR((x_ref - b).norm(), 0, 1e-12);
}

GTEST_TEST(KKTSubsystem, DoBadRootNode) { DoBadRoot<StaticSubsystem<false>>(); }


GTEST_TEST(KKTSubsystem, TreeFillIn) {
  //    {4, 5, 6}
  //    {3, 4, 6}
  //    {0, 1, 2, 3, 5}        

  std::vector<int> supernode_reference_1{4, 5, 6};
  std::vector<int> supernode_reference_2{3};
  std::vector<int> supernode_reference_3{0, 1, 2};

  std::vector<int> separator_reference_1{};
  std::vector<int> separator_reference_2{4, 5, 6};
  std::vector<int> separator_reference_3{3, 5};

  int num_vars = 7;
  Eigen::MatrixXd full_matrix(num_vars, num_vars);
  full_matrix.setZero();

  std::vector<int> vars1{4, 5, 6};
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(vars1.size(), vars1.size());
  full_matrix = IncrementSubmatrix(full_matrix, Q1, vars1);
  StaticSubsystem<false> q1(Q1, vars1);

  std::vector<int> vars2{3, 4, 6};
  Eigen::MatrixXd Q2(3, 3);
  //clang-format off
  Q2 << 0, 0, 0,
        0, 0, 0,
        0, 0, 0;
  // clang-format on
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars2);
  StaticSubsystem<false> q2(Q2, vars2);

  Eigen::MatrixXd Q3(5, 5);
  //clang-format off
  Q3 << 1, 1, 1, 1, 1,
        1, 2, 1, 1, 1,
        1, 1, 3, 1, 1,
        1, 1, 1, 4, 1,
        1, 1, 1, 1, 5;
  // clang-format on
  std::vector<int> vars3{0, 1, 2, 3, 5};
  StaticSubsystem<false> q3(Q3, vars3);
  full_matrix = IncrementSubmatrix(full_matrix, Q3, vars3);

  SymmetricLinearSystemTreeSolver system;
  system.AddSubsystem(&q1);
  system.AddSubsystem(&q2);
  system.AddSubsystem(&q3);
  Options options;
  options.validate_leaf_nodes = true;
  options.root_node = 0;

  EXPECT_NO_THROW({ system.Finalize(options); });

  EXPECT_EQ(q1.parent(), nullptr);
  EXPECT_EQ(q2.parent(), &q1);
  EXPECT_EQ(q3.parent(), &q2);

  EXPECT_EQ(q1.supernodes(), supernode_reference_1);
  EXPECT_EQ(q2.supernodes(), supernode_reference_2);
  EXPECT_EQ(q3.supernodes(), supernode_reference_3);

  EXPECT_EQ(q1.separators(), separator_reference_1);
  EXPECT_EQ(q2.separators(), separator_reference_2);
  EXPECT_EQ(q3.separators(), separator_reference_3);

  options.check_for_zero_pivots = false;
  options.validate_leaf_nodes = true;
  system.Finalize(options);
  EXPECT_NO_THROW({ system.Finalize(options); });
  system.Assemble();
  EXPECT_NEAR((system.KKTMatrix() - full_matrix).norm(), 0, 1e-14);
}


GTEST_TEST(KKTSubsystem, TreeRepair) {
/*

The initial spanning tree of the clique
intersection graph is:

           {0, 1, 2}
   {1, 2, 6}      {2, 3, 4, 5}

The node {1, 2, 6} is an invalid leaf.  This triggers
a reorganization

          {1, 2, 6} 
          {0, 1, 2}
        {2, 3, 4, 5}

*/

  std::vector<int> vars1{0, 1, 2};
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(vars1.size(), vars1.size());
  StaticSubsystem<false> q1(Q1, vars1);

  std::vector<int> vars2{1, 2, 6};
  Eigen::MatrixXd Q2(3, 3);
  Q2 << 0, 0, 1,
        0, 0, 1,
        1, 1, 0;
  StaticSubsystem<false> invalid_leaf(Q2, vars2);

  Eigen::MatrixXd Q3(4, 4);
  Q3 << 1, 0, 1, 1,
        0, 1, 1, 1,
        1, 1, 1, 1,
        1, 1, 1, 1;
  StaticSubsystem<false> q3(Q3, {0, 3, 4, 5});
  // clang-format on

  SymmetricLinearSystemTreeSolver system;
  system.AddSubsystem(&q1);
  system.AddSubsystem(&invalid_leaf);
  system.AddSubsystem(&q3);
  Options options;
  options.validate_leaf_nodes = false;
  options.check_for_zero_pivots = true;
  options.root_node = 0;
  EXPECT_THROW({ system.Finalize(options); }, std::runtime_error);
  EXPECT_EQ(invalid_leaf.parent(),  &q1);
  EXPECT_EQ(q3.parent(), &q1);
  EXPECT_EQ(q1.parent(), nullptr);

  options.check_for_zero_pivots = false;
  EXPECT_NO_THROW({ system.Finalize(options); });

  options.check_for_zero_pivots = true;
  options.validate_leaf_nodes = true;
  system.Finalize(options);

/* Verify tree has be properly reorganized */
  EXPECT_EQ(invalid_leaf.parent(),  nullptr);
  EXPECT_EQ(q1.parent(), &invalid_leaf);
  EXPECT_EQ(q3.parent(), &q1);

  EXPECT_NO_THROW({ system.Finalize(options); });
}


}  // namespace conex
