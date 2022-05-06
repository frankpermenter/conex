#include "kkt_subsystem.h"
#include "conex/debug_macros.h"
#define CONEX_ENABLE_TIMER 1

namespace conex {
namespace {

  bool is_sequential(const std::vector<int>& n) {
    for (size_t i = 1; i < n.size(); i++) {
      if (n[i] - n[i-1] != 1) {
        return false;
      }
    }
    return true;
  }
  bool is_sorted_ascending(const std::vector<int>& n) {
    for (size_t i = 1; i < n.size(); i++) {
      if (n[i] - n[i-1] <= 0) {
        return false;
      }
    }
    return true;
  }
} // namespace
using T = ConvexSetNode;

T::ConvexSetNode(const std::vector<int>& variables, 
                 const ConvexSetNodeParameters& params) : 
                 KKTSubsystem(variables, 0), params_(params) {
  int num_supernodes =
      params.num_incoming * (2 * params.spatial_dimension + 1) +
      params.spatial_dimension + 1;

  std::vector<int> supernodes;
  supernodes.insert(supernodes.begin(), variables.begin(),
                    variables.begin() + num_supernodes);

  std::vector<int> separators;
  separators.insert(separators.begin(), variables.begin() + num_supernodes,
                    variables.end());
  SetSupernodes(supernodes);
  SetSeparators(separators);
  CONEX_CHECK(is_sequential(supernodes));
  CONEX_CHECK(is_sorted_ascending(separators));
  num_incoming = params.num_incoming; 
  num_outgoing = params.num_outgoing;
  spatial_dim = params.spatial_dimension;
}



  //          ye  ze phie ye  ze  phie  lam_spatial  lam_flow    yf  pf  yf pf
  //
  // lam_s        I           I                                  -I      -I
  //                  I            I                                 -1     -1
  //                                        -I
  //                                                  -1
  //                                        -I
  //                                                  -1
  Eigen::MatrixXd T::MakeSeperatorMatrix() {
    Eigen::MatrixXd Q(num_separators(), num_supernodes());
    Q.setZero();
    // Set col to spatial flow multiplier.
    for (int i = 0; i < num_outgoing; i++) {
      Q.block(params_.outgoing_spatial_flow_start_positions.at(i), 
         params_.conservation_of_spatial_flow_multiplier_position, spatial_dim, spatial_dim)
          .diagonal()
          .setConstant(-2);
    }

    for (int i = 0; i < num_outgoing; i++) {
      int offset_row = params_.outgoing_flow_start_positions.at(i); 
      Q(offset_row, params_.conservation_of_flow_multiplier_position) = -1;
      offset_row += spatial_dim;
    }
//    Q.setConstant(-.01);
    return Q;
  }

  Eigen::MatrixXd T::MakeSeperatorMatrixNoFill() {
  throw std::runtime_error("Obsolete function.");
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
 //   Q.setConstant(-.01);
    return Q;
  }

  Eigen::MatrixXd T::MakeSuperNodeSubmatrix() {
    Eigen::MatrixXd Q(num_supernodes(), num_supernodes());
    Q.setZero();
    int offset = 0;

    // Fill y_e, z_e, phi_e all incoming e.
    MatrixXd edge_hessian(spatial_dim, spatial_dim);
    edge_hessian.setConstant(.01);
    edge_hessian.diagonal().array() += 1;
    for (int i = 0; i < num_incoming; i++) {
      Q.block(params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(i), 
              params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(i), 
      spatial_dim, spatial_dim) = edge_hessian;

      Q.block(params_.incoming_spatial_flow_start_positions.at(i), 
              params_.incoming_spatial_flow_start_positions.at(i),
      spatial_dim, spatial_dim) = edge_hessian;

      Q.block(params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(i), 
              params_.incoming_spatial_flow_start_positions.at(i), 
      spatial_dim, spatial_dim).setConstant(.01);

      Q.block(params_.incoming_spatial_flow_start_positions.at(i), 
              params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(i),
      spatial_dim, spatial_dim).setConstant(.01); 

      Q(params_.incoming_flow_start_positions.at(i), 
        params_.incoming_flow_start_positions.at(i)) = 100;


      Q.block(params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(i), 
              params_.incoming_flow_start_positions.at(i), 
              spatial_dim, 1).setConstant(0.01);

      Q.block(params_.incoming_spatial_flow_start_positions.at(i), 
              params_.incoming_flow_start_positions.at(i), 
              spatial_dim, 1).setConstant(0.01);

      Q.block(params_.incoming_flow_start_positions.at(i), 
              params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(i), 
              1, spatial_dim).setConstant(0.01);

      Q.block(params_.incoming_flow_start_positions.at(i), 
              params_.incoming_spatial_flow_start_positions.at(i), 
              1,  spatial_dim).setConstant(0.01);

    }
    

    // Spatial flow
    for (int i = 0; i < num_incoming; i++) {
      Q.block(params_.conservation_of_spatial_flow_multiplier_position, 
              params_.incoming_spatial_flow_start_positions.at(i), spatial_dim, spatial_dim).setIdentity();
    }

    // Flow conservation
    for (int i = 0; i < num_incoming; i++) {
      Q(params_.conservation_of_flow_multiplier_position,  
        params_.incoming_flow_start_positions.at(i)) = 10 + i;
    }
    return Q;
  }

} // namespace conex
