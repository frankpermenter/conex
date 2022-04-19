
#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
#include "conex/test/kkt_subsystem.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;
namespace conex {


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


class KKTSystem {
 public:
  void SolveInPlace(Eigen::MatrixXd* x) {
    root_->ApplyInverseOfLeftFactor(x);
    root_->ApplyInverseOfRightFactor(x);
  }
  void Factor() {
    root_->AssembleAndFactor();
  }

  Eigen::MatrixXd KKTMatrix() {
    int num_vars = root_->supernodes().back() + 1;
    MatrixXd M(num_vars, num_vars); M.setZero();
    root_->MakeKKTMatrix(&M);
    return M.selfadjointView<Eigen::Lower>();
  }
  KKTSubsystem* root_;
};

using Eigen::MatrixXd;

class QuadraticCost : public KKTSubsystem {
 public:
  QuadraticCost(Eigen::MatrixXd Q, std::vector<int> vars) : KKTSubsystem(vars), 
      Q_(Q)  {}

  void DoInitialize() override {
     Q_in_elimination_order_  = Q_;
     int n1 = supernodes_.size();
     int n2 = separators_.size();
     supernode_submatrix_ = Q_in_elimination_order_.topLeftCorner(n1, n1);
     separator_rows_ = Q_in_elimination_order_.bottomLeftCorner(n2, n1);
     separator_schur_complement_ = Q_in_elimination_order_.bottomRightCorner(n2, n2);
  }

  void DoEliminateSupernodeColumns() override {
     llt_.compute(supernode_submatrix_);
  }

  MatrixXd DoGetSupernodeColumns() override {
    MatrixXd cols(supernodes_.size() + separators_.size(),
                  supernodes_.size());
    cols << supernode_submatrix_, 
             separator_rows_;
    return cols;
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(Eigen::Ref<MatrixXd> y) override {
    if (schur_complement_mode_) {
     llt_.solveInPlace(y);
    } else {
     llt_.matrixL().solveInPlace(y);
    }
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(Eigen::Ref<MatrixXd> y) override {
    if (schur_complement_mode_) {
      return;
    }
    llt_.matrixL().transpose().solveInPlace(y);
  }

  void DoComputeSeparatorSchurComplement() override {
    int n2 = separators_.size();
    separator_schur_complement_ -=  separator_rows_ * llt_.solve(separator_rows_.transpose());
  }

  bool schur_complement_mode_ = true;
  Eigen::LDLT<Eigen::MatrixXd> llt_;
  Eigen::MatrixXd Q_in_elimination_order_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd separator_rows_left_factor_;
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



// 1 1 1 
// 1 1 1
// 1 1 1 1 0
//   0 1 1 1
// 0 0 0 1 1
GTEST_TEST(KKTSubsystem, TestTrivialExample) {

  int num_vars = 5;
  MatrixXd full_matrix = MatrixXd::Zero(num_vars, num_vars);
  std::vector<int> vars{0, 1, 2};
  Eigen::MatrixXd Q1(3, 3);
  Q1 << 50, 2, 3, 
        2, 10, 4, 
        3, 4, 10;
  QuadraticCost q1(Q1, vars); q1.SetSeparators({2}); q1.SetSupernodes({0, 1});
  full_matrix = IncrementSubmatrix(full_matrix, Q1, vars);

  std::vector<int> vars_2{2, 3};
  Eigen::MatrixXd Q2(2, 2);
  Q2 << 5, 2,
        2, 5;
  QuadraticCost q2(Q2, vars_2); q2.SetSupernodes({2}); q2.SetSeparators({3});
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars_2);

  std::vector<int> vars_3{3, 4};
  QuadraticCost q3(Q2, vars_3); q3.SetSupernodes({3, 4});
  full_matrix = IncrementSubmatrix(full_matrix, Q2, vars_3);

  DUMP(full_matrix);

  q1.DoInitialize();
  q2.DoInitialize();
  q3.DoInitialize();

  q2.AddChild(&q1);
  q3.AddChild(&q2);

  EXPECT_EQ(q1.parent(), &q2);
  EXPECT_EQ(q2.parent(), &q3);

  KKTSystem system;
  system.root_ = &q3;

  //EXPECT_NEAR( (system.KKTMatrix() - full_matrix).norm(), 0, 1e-14);


  Eigen::LLT<Eigen::MatrixXd> llt(full_matrix);
  Eigen::MatrixXd L = llt.matrixL();
  VectorXd x_ref(num_vars);
  x_ref.setLinSpaced(5, -1, 1);
  MatrixXd b = full_matrix * x_ref;
  system.Factor();
  system.SolveInPlace(&b);
  EXPECT_NEAR((x_ref - b).norm(), 0, 1e-12);

}










} // namespace conex
