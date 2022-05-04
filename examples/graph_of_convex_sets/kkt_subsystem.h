#pragma once
#include <Eigen/Dense>

#include "conex/kkt_subsystem.h"
#include "conex/RLDLT.h"
//#include "directed_graph.h"

namespace conex {
using Eigen::MatrixXd;
constexpr int schur_complement_mode = true;
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
      (void) y;
      return;
    } else {
      llt_.matrixL().transpose().solveInPlace(y);
    }
  }

  void DoComputeSeparatorSchurComplement() override;

  Eigen::RLDLT<Eigen::MatrixXd> llt_;
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> temp_row_major_;
};


class LUSolver : public KKTSubsystem {
 public:
  LUSolver(std::vector<int> vars) : KKTSubsystem(vars, 0) {}

  void DoEliminateSupernodeColumns() override {
    lu_.compute(supernode_submatrix_.selfadjointView<Eigen::Lower>());
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const override {
    y = lu_.solve(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const override {
    (void)y;  // NOOP
  }

  void DoComputeSeparatorSchurComplement() override {
    separator_schur_complement_ -=
        separator_rows_ * lu_.solve(separator_rows_.transpose());
  }

  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
  }

  Eigen::PartialPivLU<Eigen::MatrixXd> lu_;
};

class StaticSubsystem : public LUSolver {
  using Base = LUSolver; 

 public:
  StaticSubsystem(Eigen::MatrixXd Q, std::vector<int> vars)
      : Base(vars), Q_(Q.selfadjointView<Eigen::Lower>()) {}

  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    int n1 = Base::supernode_submatrix_.rows();
    int n2 = Base::separator_rows_.rows();
    Q_in_elimination_order_.resize(n1 + n2, n1 + n2);
    AssignSubmatrix(Q_, Q_in_elimination_order_,
                    Base::variable_to_local_elimination_rank());
    DoAssemble();
  }

  Eigen::MatrixXd submatrix() const { return Q_; }

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
  const Eigen::MatrixXd Q_;
};



// Variables for node v with incoming edges {e} and outgoing
// edges {f}:
//
// Supernodes:
//
//    outgoing spatial y_e,
//    incoming spatial z_f,
//    incoming flow variable: phi_f
//
// Separators:
//
//    outgoing flow variable: phi_e
//    upstream incoming spatial z_e,
//
//
// Sparsity of KKT submatrix:
//
// ye      *                         I
// ye         *                      I
// zf              *                -1
// zf                 *             -1
// phif            *  *    *  *        -1
// phif            *  *    *  *        -1
//
// lam    I   I  -I  -I               0
// lam                    -1  -1      0             1
//
// ze     *   0    0
// ze     0   *       0
// phie   *   *             *  *      1
//
//  We eliminate incoming spatial z,
//  outgoing spatial y, outgoing flows phi.
//

//        ye  ye  zf  zf  phif phif  lam  ze  ze   phie
//
// ye      *
// ye         *
// zf              *
// zf                 *
// phif            *  *    *  *
// phif            *  *    *  *
//
// lam    I   I  -I  -I               0
// lam                    -1  -1      0
//
// ze     *   0    0
// ze     0   *       0
// phie   *   *             *  *      1

// Cliques:
//
//  y z


/*
 Given node elimination sequence
  (Out-going y)  (In coming z) (Incoming Flow)

  Elimination order:

   (ye, ze, phie)_{incoming edges}, conservation of flow multipliers,

  Equations: grad_{ye, ze, phie} = 0,
  conservation of flow: ze + yf = 0. phie + phif = 0

  Separators: yf and phif of out-going edges.

  We can interpret each block as optimality
  conditions for the node sub-problem:
 
  min \sum_{e \in Incoming} f(z_e, y_e, ph_e)
   
  \sum_{e \in In} y_e + \sum_{f \in Out} z_{f} = 0
  \sum_{e \in In} phi_e + \sum_{f \in Out} phi_{f} = 0
  \sum_{e \in In} phi_e <= 1.
*/

/*
 Construct graph of form
  0 -> 1 -> 2 -> 3 -> 4

 min \sum_{e \in Incoming} f(z_e, y_e)
   
  \ y_e +  z_{f} = 0
  phi_e +  phi_{f} = 0
  phi_e <= 1.

*/

struct ConvexSetNodeParameters {
  int spatial_dimension;
  int num_incoming;
  int num_outgoing;
  std::vector<int> outgoing_edge_start_positions;
};

class ConvexSetNode : public CholeskySolver {
 public:

  ConvexSetNode(const std::vector<int>& scalar_variables, 
                const ConvexSetNodeParameters& parameters);

  int num_supernodes() { return supernodes().size(); }
  int num_separators() { return separators().size(); }

  Eigen::MatrixXd MakeSuperNodeSubmatrix() {
    Eigen::MatrixXd Q(num_supernodes(), num_supernodes());
    Q.setZero();
    int offset = 0;

    // Fill y_e, z_e, phi_e all incoming e.
    for (int i = 0; i < num_incoming; i++) {
      Q.block(offset, offset, spatial_dim * 2 + 1, spatial_dim * 2 + 1)
          .setConstant(.01);
      Q.block(offset, offset, spatial_dim * 2 + 1, spatial_dim * 2 + 1)
          .diagonal()
          .setConstant(1);
      offset += 2 * spatial_dim + 1;
      Q(offset - 1, offset - 1) = 100;
    }

    // Spatial flow
    int offset_row = offset;
    int offset_col = spatial_dim;
    for (int i = 0; i < num_incoming; i++) {
      Q.block(offset_row, offset_col, spatial_dim, spatial_dim).setIdentity();
      offset_col += 2 * spatial_dim + 1;
    }

    offset_row += spatial_dim;
    offset_col = 2 * spatial_dim;
    // Flow conservation
    for (int i = 0; i < num_incoming; i++) {
      Q(offset_row, offset_col) = 10 + i;
      offset_col += 2 * spatial_dim + 1;
    }
    return Q;
  }

  Eigen::MatrixXd Submatrix() {
    Eigen::MatrixXd m2 = MakeSuperNodeSubmatrix();
    Eigen::MatrixXd m1 = MakeSeperatorMatrixNoFill();
    Eigen::MatrixXd Q(m1.rows() + m2.rows(), m1.rows() + m2.rows());
    Q << m2, m1.transpose(),
         m1, Eigen::MatrixXd::Zero(m1.rows(), m1.rows());
    return Q;
  }

  //          ye  ze phie ye  ze  phie  lam_spatial  lam_flow    yf  pf  yf pf
  //
  // lam_s        I           I                                  -I      -I
  //                  I            I                                 -1     -1
  //                                        -I
  //                                                  -1
  //                                        -I
  //                                                  -1
  Eigen::MatrixXd MakeSeperatorMatrix() {
    Eigen::MatrixXd Q(num_separators(), num_supernodes());
    Q.setZero();
    // Set col to spatial flow multiplier.
    int offset_col = (2 * spatial_dim + 1) * num_incoming;
    for (int i = 0; i < num_outgoing; i++) {
      Q.block(params_.outgoing_edge_start_positions.at(i), offset_col, spatial_dim, spatial_dim)
          .diagonal()
          .setConstant(-2);
    }

    // Update col to flow multiplier.
    offset_col += spatial_dim;

    for (int i = 0; i < num_outgoing; i++) {
      int offset_row = params_.outgoing_edge_start_positions.at(i) + spatial_dim;
      Q(offset_row, offset_col) = -1;
      offset_row += spatial_dim;
    }
    return Q;
  }

  Eigen::MatrixXd MakeSeperatorMatrixNoFill() {
    Eigen::MatrixXd Q(num_separators(), num_supernodes());
    Q.setZero();
    int offset_row = 0;
    int offset_col = (2 * spatial_dim + 1) * num_incoming;
    for (int i = 0; i < num_outgoing; i++) {
      Q.block(offset_row, offset_col, spatial_dim, spatial_dim)
          .diagonal()
          .setConstant(-2);
      offset_row += spatial_dim + 1;
    }
    offset_col += spatial_dim;
    offset_row = spatial_dim;
    for (int i = 0; i < num_outgoing; i++) {
      Q(offset_row, offset_col) = -1;
      offset_row += spatial_dim + 1;
    }
    return Q;
  }


 private:
  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    auto data = MakeSeperatorMatrix();
    CONEX_CHECK(data.rows() == separator_rows_.rows());
    CONEX_CHECK(data.cols() == separator_rows_.cols());
    separator_rows_ = data;

    data = MakeSuperNodeSubmatrix();
    CONEX_CHECK(data.rows() == supernode_submatrix_.rows());
    CONEX_CHECK(data.cols() == supernode_submatrix_.cols());
    supernode_submatrix_ = data;
    separator_schur_complement_.setZero();
  }

  int spatial_dim = 0;
  int num_incoming = 0;
  int num_outgoing = 0;
  ConvexSetNodeParameters params_;
};

} // namespace conex
