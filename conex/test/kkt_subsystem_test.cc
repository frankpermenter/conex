
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

class KKTSystem {
 public:
  Eigen::VectorXd SolveInPlace(Eigen::MatrixXd* x) {
    root->ApplyInverseOfLeftFactor(x);
    DUMP(*x);
    root->ApplyInverseOfRightFactor(x);
  }
  KKTSubsystem* root;
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

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(Eigen::Ref<MatrixXd> y) override {
    DUMP(y);
     llt_.matrixL().solveInPlace(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(Eigen::Ref<MatrixXd> y) override {
    DUMP(y);
     llt_.matrixL().transpose().solveInPlace(y);
  }

  void DoComputeSeparatorSchurComplement() override {
    int n2 = separators_.size();
    separator_schur_complement_ -=  separator_rows_ * llt_.solve(separator_rows_.transpose());
  }

  Eigen::LLT<Eigen::MatrixXd> llt_;
  Eigen::MatrixXd Q_in_elimination_order_;
  Eigen::MatrixXd Q_;
};



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
  q2.AssembleAndFactor();

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
  KKTSystem system;
  system.root = &q2;
  system.SolveInPlace(&b);
  DUMP(b);
}

} // namespace conex
