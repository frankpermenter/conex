#define CONEX_ENABLE_TIMER 1
#include "kkt_subsystem.h"
#include "conex/debug_macros.h"
#include "supernode_inverse.h"

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

SupernodeSubmatrix::Parameters MakeParams(const ConvexSetNodeParameters& input) {
  SupernodeSubmatrix::Parameters params;
  params.num_edges = input.num_incoming;
  params.spatial_dimension = input.spatial_dimension;
  return params;
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
  use_custom_supernode_inverse_ = false;
}

  void T::DoInitialize() {
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

    supernode_submatrix_ = std::make_unique<SupernodeSubmatrix>(MakeParams(params_), 
    supernode_submatrix());
  }





  //          ye  ze phie ye  ze  phie  lam_spatial  lam_flow    yf  pf  yf pf
  //
  // lam_s        I           I                                  -I      -I
  //                  I            I                                 -1     -1
  //                                        -I
  //                                                  -1
  //                                        -I
  //                                                  -1

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
   Q.setConstant(-.01);
    return Q;
  }

  Eigen::MatrixXd T::MakeSuperNodeSubmatrix() {
    Eigen::MatrixXd Q(num_supernodes(), num_supernodes());
    Q.setZero();

    // Fill y_e, z_e, phi_e all incoming e.
    MatrixXd edge_hessian(spatial_dim, spatial_dim);
    edge_hessian.setConstant(.01);
    edge_hessian.diagonal().array() += 1;
    for (int i = 0; i < num_incoming; i++) {
      for (int j = 0; j < num_incoming; j++) {
        if (!fill_in_ && i != j) {
          continue; 
        }
        Q.block(params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(i), 
                params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(j), 
        spatial_dim, spatial_dim) = edge_hessian;

        Q.block(params_.incoming_spatial_flow_start_positions.at(i), 
                params_.incoming_spatial_flow_start_positions.at(j),
        spatial_dim, spatial_dim) = edge_hessian;

        Q.block(params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(i), 
                params_.incoming_spatial_flow_start_positions.at(j), 
        spatial_dim, spatial_dim).setConstant(.01);

        Q.block(params_.incoming_spatial_flow_start_positions.at(i), 
                params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(j),
        spatial_dim, spatial_dim).setConstant(.01); 

        Q(params_.incoming_flow_start_positions.at(i), 
          params_.incoming_flow_start_positions.at(j)) = .01;


        Q.block(params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(i), 
                params_.incoming_flow_start_positions.at(j), 
                spatial_dim, 1).setConstant(0.01);

        Q.block(params_.incoming_spatial_flow_start_positions.at(i), 
                params_.incoming_flow_start_positions.at(j), 
                spatial_dim, 1).setConstant(0.01);

        Q.block(params_.incoming_flow_start_positions.at(i), 
                params_.outgoing_spatial_flow_of_incoming_edge_start_positions.at(j), 
                1, spatial_dim).setConstant(0.01);

        Q.block(params_.incoming_flow_start_positions.at(i), 
                params_.incoming_spatial_flow_start_positions.at(j), 
                1,  spatial_dim).setConstant(0.01);
      }
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
    int rows = Q.rows() - 1 - params_.spatial_dimension;

    //Q.setConstant(.1);
    if (fill_in_) {
      Q.diagonal().head(rows).setConstant(100);
    }
    return Q;
  }

  Eigen::MatrixXd T::MakeSeperatorMatrix() {
    Eigen::MatrixXd Q(num_separators(), num_supernodes());
    Q.setZero();
    if (fill_in_) {    
      for (int i = 0; i < num_outgoing; i++) {
        for (int j = 0; j < num_incoming; j++) {
          Q.block(params_.outgoing_spatial_flow_start_positions.at(i), 
                  params_.incoming_spatial_flow_start_positions.at(j), spatial_dim, spatial_dim).setConstant(.000001);
        }
      }

      for (int i = 0; i < num_outgoing; i++) {
        for (int j = 0; j < num_incoming; j++) {
          Q.block(params_.outgoing_flow_start_positions.at(i), 
                  params_.incoming_flow_start_positions.at(j), spatial_dim, spatial_dim).setConstant(.000001);
        }
      }

      for (int i = 0; i < num_outgoing; i++) {
        for (int j = 0; j < num_incoming; j++) {
          Q.block(params_.outgoing_flow_start_positions.at(i), 
                  params_.incoming_spatial_flow_start_positions.at(j),  1, spatial_dim).setConstant(.000001);
        }
      }
      for (int i = 0; i < num_outgoing; i++) {
        for (int j = 0; j < num_incoming; j++) {
          Q.block(params_.outgoing_spatial_flow_start_positions.at(i), 
                  params_.incoming_flow_start_positions.at(j), spatial_dim, 1).setConstant(.000001);
        }
      }
    }

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
    return Q;
  }

  bool T::DoEliminateSupernodeColumns() {
    bool success = true;
    if (use_custom_supernode_inverse_) {
      supernode_submatrix_->SetData(supernode_submatrix());
      success = supernode_submatrix_->AssembleAndFactor();
    } else {
      success = factorization_->DoEliminateSupernodeColumns();
    }
    return success;
  }

  void T::DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const {
    if (use_custom_supernode_inverse_) {
      supernode_submatrix_->SolveInPlace(y);
    } else {
      factorization_->DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(y);
    }
  }

  void T::DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const {
    if (use_custom_supernode_inverse_) {
      (void) y; // NOOP
    } else  {
      factorization_->DoApplyInverseOfRightFactorOfSupernodeSubmatrix(y);
    }
  }
  void T::DoComputeSeparatorSchurComplement() {
    if (separator_rows().rows() == 0) {
      return;
    }
    MatrixXd temp = separator_rows().transpose();
    if (use_custom_supernode_inverse_) {
        supernode_submatrix_->SolveInPlace(temp);
        int n = temp.cols();
        for (int j = 0; j < n; j++) {
          separator_schur_complement().col(j).tail(n - j).noalias() -= separator_rows().bottomRows(n - j) * temp.col(j);
        }
    } else {
      factorization_->DoComputeSeparatorSchurComplement();
    }
  }



} // namespace conex
