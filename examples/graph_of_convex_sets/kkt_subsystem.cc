#include "kkt_subsystem.h"
#include "conex/debug_macros.h"
#define CONEX_ENABLE_TIMER 1

namespace conex {
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
    int offset_col = (2 * spatial_dim + 1) * num_incoming;
    for (int i = 0; i < num_outgoing; i++) {
      Q.block(params_.outgoing_spatial_flow_start_positions.at(i), offset_col, spatial_dim, spatial_dim)
          .diagonal()
          .setConstant(-2);
    }

    // Update col to flow multiplier.
    offset_col += spatial_dim;

    for (int i = 0; i < num_outgoing; i++) {
      int offset_row = params_.outgoing_flow_start_positions.at(i); 
      Q(offset_row, offset_col) = -1;
      offset_row += spatial_dim;
    }
//    Q.setConstant(-.01);
    return Q;
  }

  Eigen::MatrixXd T::MakeSeperatorMatrixNoFill() {
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

} // namespace conex
