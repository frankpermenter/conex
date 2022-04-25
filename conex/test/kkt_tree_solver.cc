#include "gtest/gtest.h"
#include "conex/kkt_subsystem.h"
#include "conex/error_checking_macros.h"
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

using Eigen::MatrixXd;

class LUSolver : public KKTSubsystem {
 public:
  LUSolver(std::vector<int> vars) : KKTSubsystem(vars, 0) {}

  void DoEliminateSupernodeColumns() override { lu_.compute(supernode_submatrix_); }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    y = lu_.solve(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
   (void) y;// NOOP
  }

  void DoComputeSeparatorSchurComplement() override {
    separator_schur_complement_ -=
        separator_rows_ * lu_.solve(separator_rows_.transpose());
  }

  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    int n1 = supernode_submatrix_.rows();
    int n2 = separator_rows_.rows();
  }

  Eigen::PartialPivLU<Eigen::MatrixXd> lu_;
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
//        ye  ye  zf   zf  phif phif  lam  ze  ze   phie
//
//        *                            
//            *
//                *                   
//                     *              
//                *    0    *  *
//                0    *    *  *
//
// lam    I   I   -I   -I              0   
// lam                    -1  -1       0       
//
// ze     *   0    0                
// ze     0   *         0
// phie                     *  *      1
//
//  We eliminate incoming spatial z,
//  outgoing spatial y, outgoing flows phi. 
//

struct VariableIDs {
  std::vector<int> incoming_flows;
  std::vector<int> outgoing_flows;
  std::vector<std::vector<int>> outgoing_spatial_flow;
  std::vector<std::vector<int>> outgoing_spatial_flow_separator;
  std::vector<std::vector<int>> incoming_spatial_flow;
  std::vector<int> conversation_of_spatial_flow_multiplier; 
  int conversation_of_flow_multiplier; 
};

struct NodeData {
  int spatial_dim;
  int node_id;
  VariableIDs variable_ids;
};

class ConvexSetNode : public LUSolver {
 public:
  static std::vector<int> ConcatenateVariablesInLocalOrdering(const VariableIDs& ids) {
    std::vector<int> variables;
    for (const auto& y_e : ids.outgoing_spatial_flow) {
      variables.insert(variables.end(), y_e.begin(), y_e.end());
    }
    for (const auto& y_e : ids.incoming_spatial_flow) {
      variables.insert(variables.end(), y_e.begin(), y_e.end());
    }
    variables.insert(variables.end(), ids.incoming_flows.begin(), 
                     ids.incoming_flows.end()); 
    variables.insert(variables.end(), ids.conversation_of_spatial_flow_multiplier.begin(), 
                     ids.conversation_of_spatial_flow_multiplier.end()); 
    variables.push_back(ids.conversation_of_flow_multiplier);
    for (const auto& z_e : ids.outgoing_spatial_flow_separator) {
      variables.insert(variables.end(), z_e.begin(), z_e.end());
    }
    variables.insert(variables.end(), ids.outgoing_flows.begin(), 
                     ids.outgoing_flows.end()); 
    return variables;
  }

  ConvexSetNode(const NodeData& data) : LUSolver(ConcatenateVariablesInLocalOrdering(data.variable_ids)) {
    const auto& ids = data.variable_ids;
    num_incoming = ids.incoming_flows.size();
    num_outgoing = ids.outgoing_flows.size();
    spatial_dim = ids.conversation_of_spatial_flow_multiplier.size();

    std::vector<int> supernodes;
    std::vector<int> separators;

    for (auto e : ids.outgoing_spatial_flow) {
      supernodes.insert(supernodes.end(), e.begin(), e.end());
    }
    for (auto e : ids.incoming_spatial_flow) {
      supernodes.insert(supernodes.end(), e.begin(), e.end());
    }
    supernodes.insert(supernodes.end(), ids.incoming_flows.begin(), ids.incoming_flows.end());
    supernodes.insert(supernodes.end(), ids.conversation_of_spatial_flow_multiplier.begin(), 
                                        ids.conversation_of_spatial_flow_multiplier.end());
    supernodes.push_back(ids.conversation_of_flow_multiplier);


    for (auto e : ids.outgoing_spatial_flow_separator) {
      separators.insert(separators.end(), e.begin(), e.end());
    }
    separators.insert(separators.end(), ids.outgoing_flows.begin(), ids.outgoing_flows.end());


    SetSeparators(separators);
    SetSupernodes(supernodes);

    CONEX_CHECK(ids.incoming_flows.size() == ids.incoming_spatial_flow.size());
    CONEX_CHECK(ids.outgoing_flows.size() == ids.outgoing_spatial_flow_separator.size());
    CONEX_CHECK(ids.outgoing_flows.size() == ids.outgoing_spatial_flow.size());
  }

  int num_variables() {
      return spatial_dim * (num_incoming + num_outgoing) + // z incoming, y outgoing
             + num_incoming
             + (spatial_dim + 1) // conservation multipliers
             + num_outgoing +  // outgoing flows
             + spatial_dim * num_outgoing; // z_e for each outgoing y_e 
  } 

  int num_supernodes() {
      return spatial_dim * (num_incoming + num_outgoing) + // z incoming, y outgoing
             num_incoming +  // incoming_flows
             spatial_dim + 1; // conservation multipliers
  } // downstream incoming

  int num_separators() {
    return num_outgoing*(1+spatial_dim); // z_e for each outgoing y_e 
  } // downstream incoming


  Eigen::MatrixXd MakeSuperNodeSubmatrix() {
    Eigen::MatrixXd Q(num_supernodes(), num_supernodes());
    //Eigen::MatrixXd Q(num_variables(), num_variables());
    Q.setZero();
    int offset = 0;

    // Fill y_e
    for (int i = 0; i < num_outgoing; i++) {
      Q.block(offset, offset, spatial_dim, spatial_dim).setConstant(.01);
      Q.block(offset, offset, spatial_dim, spatial_dim).diagonal().setConstant(1);
      offset += spatial_dim;
    }

    int offset_z = offset;
    // Fill z_f
    for (int i = 0; i < num_incoming; i++) {
      Q.block(offset, offset, spatial_dim, spatial_dim).setConstant(-.01);
      Q.block(offset, offset, spatial_dim, spatial_dim).diagonal().setConstant(2);
      offset += spatial_dim;
    }

    // Fill phi_f / z_f
    if (0) {
      for (int i = 0; i < num_incoming; i++) {
        Q.block(offset + i, offset_z + i * spatial_dim, 1, spatial_dim).setRandom();
      }
    } else {
      Q.block(offset, offset_z, num_incoming, spatial_dim * num_incoming).setRandom();
    }

    // Fill incoming flows phif
    Q.block(offset, offset, num_incoming, num_incoming).setConstant(4);
    offset += num_incoming;

    // Spatial flow 
    int offset_row = offset;
    int offset_col = 0;
    for (int i = 0; i < num_outgoing; i++) {
      Q.block(offset_row, offset_col, spatial_dim, spatial_dim).setIdentity();
      offset_col += spatial_dim;
    }
    for (int i = 0; i < num_incoming; i++) {
      Q.block(offset_row, offset_col, spatial_dim, spatial_dim).diagonal().setConstant(-1);
      offset_col += spatial_dim;
    }

    // Flow conservation
    offset_row += spatial_dim;
    Q.block(offset_row, offset_col, 1, num_incoming).setConstant(-10);
    return Q;
  }

//       ye  ye   zf   zf  phif  lam  lam
// ze     *   0    0                
// ze     0   *         0
// phie                     *  *  0    1
// phie                     *  *  0    1

  Eigen::MatrixXd MakeSeperatorMatrix() {
    Eigen::MatrixXd Q(num_separators(), num_supernodes());
    Q.diagonal().head(num_outgoing * spatial_dim).setConstant(1);
    // Phie
    Q.block(num_outgoing * spatial_dim,  (num_incoming+num_outgoing) * spatial_dim, 
            num_outgoing, num_incoming ).setConstant(1);

    // Flow equation transpose
    Q.rightCols(1).bottomRows(num_outgoing).setConstant(1); 
    return Q;
  }



 private:

  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    auto data = MakeSeperatorMatrix();
    DUMP(data.cols());
    DUMP(data.rows());
    DUMP(separator_rows_.rows());
    DUMP(separator_rows_.cols());
    CONEX_CHECK(data.rows() == separator_rows_.rows());
    CONEX_CHECK(data.cols() == separator_rows_.cols());
    separator_rows_ = data;

    data = MakeSuperNodeSubmatrix();
    CONEX_CHECK(data.rows() == supernode_submatrix_.rows());
    CONEX_CHECK(data.cols() == supernode_submatrix_.cols());
    supernode_submatrix_ = data; 
  }

  int spatial_dim = 0;
  int num_incoming = 0;
  int num_outgoing = 0;
};

GTEST_TEST(GraphOfConvexSets, PrintSparsity) {
  //Eigen::MatrixXd M(5, 5);
  //M << 1, 0, 0, 0, 1, 
  //     0, 1, 0, 0, 1, 
  //     0, 0, 1, 0, -1, 
  //     0, 0, 0, 1, -1,
  //    1, 1, -1, -1, 0;

  //Eigen::MatrixXd B(2, 5);

  //B << 0, 0, 1, 0, 0,
  //     0, 0, 0, 1, 0;

  //Eigen::LDLT<Eigen::MatrixXd> llt(M);
  //DUMP(B* MatrixXd(llt.solve(B.transpose())));

  NodeData node_data;

  node_data.variable_ids.incoming_flows = std::vector{0, 1};
  node_data.variable_ids.outgoing_flows = std::vector{2, 3, 4};
  node_data.variable_ids.outgoing_spatial_flow = std::vector<std::vector<int>>{{5, 6, 7}, {8, 9, 10}, {11, 12, 13}};
  node_data.variable_ids.incoming_spatial_flow = std::vector<std::vector<int>>{ {14, 15, 16}, {17, 18, 19}  };
  node_data.variable_ids.conversation_of_spatial_flow_multiplier = std::vector<int>{  {20, 21, 22}  };
  node_data.variable_ids.outgoing_spatial_flow_separator = std::vector<std::vector<int>>{  {23, 24, 25}, {26, 27, 28}, {29, 30, 31}  };
  node_data.variable_ids.conversation_of_flow_multiplier = 32; 


  ConvexSetNode node(node_data);
  node.Assemble();
  MatrixXd supernode_submatrix = node.MakeSuperNodeSubmatrix();

  MatrixXd temp(33, 33);
  node.MakeKKTMatrix(&temp);
  DUMP(temp);
  return;

  DUMP(supernode_submatrix);
  Eigen::LDLT<Eigen::MatrixXd> llt(supernode_submatrix);
  Eigen::MatrixXd factor = llt.matrixL();
  DUMP(node.MakeSeperatorMatrix());
  Eigen::MatrixXd seperator = node.MakeSeperatorMatrix();
  DUMP(seperator * MatrixXd(llt.solve(seperator.transpose())));
}




} // namespace conex
