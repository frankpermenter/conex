#pragma once
#include <Eigen/Dense>

#include "conex/kkt_subsystem.h"
#include "conex/cholesky_solvers.h"
#include "conex/RLDLT.h"
//#include "directed_graph.h"

namespace conex {
using Eigen::MatrixXd;



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
  // Local variable positions.
  std::vector<int> outgoing_spatial_flow_start_positions;
  std::vector<int> outgoing_flow_start_positions;
  std::vector<int> incoming_spatial_flow_start_positions;
  std::vector<int> outgoing_spatial_flow_of_incoming_edge_start_positions;
  std::vector<int> incoming_flow_start_positions;
  int conservation_of_flow_multiplier_position;
  int conservation_of_spatial_flow_multiplier_position;
};

#define CONEX_NO_COPY_NO_MOVE(T)\
T(const T&) = delete;\
T(T&&) = delete;\
T& operator=(const T&) = delete;\
T& operator=(T&&) = delete;\





class ConvexSetNode : public KKTSubsystem {
 public:
  CONEX_NO_COPY_NO_MOVE(ConvexSetNode)

  ConvexSetNode(const std::vector<int>& scalar_variables, 
                const ConvexSetNodeParameters& parameters);


  int num_supernodes() { return supernodes().size(); }
  int num_separators() { return separators().size(); }

  bool DoEliminateSupernodeColumns() override {
    return factorization_->DoEliminateSupernodeColumns();
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    factorization_->DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    factorization_->DoApplyInverseOfRightFactorOfSupernodeSubmatrix(y);
  }

  void DoComputeSeparatorSchurComplement() override {
    factorization_->DoComputeSeparatorSchurComplement();
  }

  Eigen::MatrixXd MakeSuperNodeSubmatrix();

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
  Eigen::MatrixXd MakeSeperatorMatrix();

  Eigen::MatrixXd MakeSeperatorMatrixNoFill();

 private:
  using FactorizationType = CholeskySolver<Eigen::RLDLT<Eigen::MatrixXd>, true>;
  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    auto data = MakeSeperatorMatrix();
    CONEX_CHECK(data.rows() == separator_rows().rows());
    CONEX_CHECK(data.cols() == separator_rows().cols());
    separator_rows() = data;

    data = MakeSuperNodeSubmatrix();
    CONEX_CHECK(data.rows() == supernode_submatrix().rows());
    CONEX_CHECK(data.cols() == supernode_submatrix().cols());
    supernode_submatrix() = data;
    separator_schur_complement().setZero();
    factorization_ = std::make_unique<FactorizationType>(supernode_submatrix(), separator_rows(), separator_schur_complement());
  }

  int spatial_dim = 0;
  int num_incoming = 0;
  int num_outgoing = 0;
  ConvexSetNodeParameters params_;
  std::unique_ptr<FactorizationType> factorization_;
};

} // namespace conex
