
#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
#include "conex/test/kkt_subsystem.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

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
     int n1 = supernodes_.size();
     llt_.compute(supernode_submatrix_);
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

  q2.AssembleAndFactor(true);

  Eigen::MatrixXd Q_full(5, 5);
  Q_full.setZero();
  Q_full.topLeftCorner(3, 3) = Q;
  Q_full.bottomRightCorner(3, 3) += Q;
  DUMP(Q_full);
  Eigen::LLT<Eigen::MatrixXd> llt(Q_full);
  Eigen::MatrixXd L = llt.matrixL();
  DUMP(L);
  //DUMP(MatrixXd(q1.llt_.matrixL()));
  //DUMP(MatrixXd(q2.llt_.matrixL()));;


}

};
