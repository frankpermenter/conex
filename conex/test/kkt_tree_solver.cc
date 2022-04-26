#include "gtest/gtest.h"
#include "conex/kkt_subsystem.h"
#include "conex/error_checking_macros.h"
#include <Eigen/Dense>
#include <map>
#include <stack>

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


struct Variables {
  std::vector<int> flow_variable;
  // Map incoming edge label to spatial variable
  std::vector<std::vector<int>> incoming_spatial_flow_variable;
  std::vector<std::vector<int>> outgoing_spatial_flow_variable;
  std::vector<std::vector<int>> node_to_conversation_of_spatial_flow_multiplier; 
  std::vector<int> node_to_conversation_of_flow_multiplier; 
};


struct Node {
  std::vector<int> incoming_edges;
  std::vector<int> outgoing_edges;
  int spatial_dimension;

  int parent_in_spanning_tree = 0;
  std::vector<int> children_in_spanning_tree;
  Variables ids;
};

class Graph {
 public:
  Graph(std::vector<Node> nodes) : nodes_(std::move(nodes)) {}


  void AssignEliminationOrder() {
    int offset = 0;
    for (auto& root : roots_) {
      offset = AssignEliminationOrderHelper(root, offset);
    }
  }

  // Do a pass to assign elimination position to each variable
  int AssignEliminationOrderHelper(int node_index,  int offset) {

    for (auto& child : nodes_.at(node_index).children_in_spanning_tree) {
      offset = AssignEliminationOrderHelper(child, offset);
    }

    // Assign variable 
    for (auto e : nodes_.at(node_index).incoming_edges) {
      // Spatial y_e
      for (int i = 0; i < nodes_.at(node_index).spatial_dimension; ++i) {
        ids_.outgoing_spatial_flow_variable.at(e).push_back(offset);
        offset++;
      }
      // Spatial z_e
      for (int i = 0; i < nodes_.at(node_index).spatial_dimension; ++i) {
        ids_.incoming_spatial_flow_variable.at(e).push_back(offset);
        offset++;
      }
      ids_.flow_variable.at(node_index) = e;
      offset++; // phi_e
    }

    for (int i = 0; i < nodes_.at(node_index).spatial_dimension; ++i) {
      ids_.node_to_conversation_of_spatial_flow_multiplier.at(node_index).push_back(offset);
      offset++; 
    }
    ids_.node_to_conversation_of_flow_multiplier.at(node_index) = offset;
  }

  void BuildSpanningTree(int root) {
    std::vector<int> visited(nodes_.size(), 0);

    int parent = root;
    roots_.push_back(root);
    visited.at(parent) = 1;
    nodes_[parent].parent_in_spanning_tree = -1;

    std::stack<int> nodes_to_visit;
    while (nodes_to_visit.size() > 0) {
      parent = nodes_to_visit.top(); nodes_to_visit.pop();
      for (auto& child : nodes_.at(parent).outgoing_edges) {
        if (visited.at(child) == 0) {
          visited.at(child) = 1;
          nodes_.at(child).parent_in_spanning_tree = parent;
          nodes_.at(parent).children_in_spanning_tree.push_back(child);
          nodes_to_visit.push(child);
        }
      }
    }
  }

  NodeData BuildNodeData(int node_index)  {
    NodeData data;
  }

  std::vector<int> roots_;
  std::vector<Node> nodes_;
  Variables ids_;
};









class ConvexSetNode : public LUSolver {
 public:
  static std::vector<int> ConcatenateVariablesInLocalOrdering(const Graph& graph, const int node_index) {
    auto ids = graph.ids_;
    std::vector<int> variables;
    auto& node = graph.nodes_.at(node_index);
    for (auto e : node.incoming_edges) {
      auto& y_e = ids.outgoing_spatial_flow_variable.at(e);
      variables.insert(variables.end(), y_e.begin(), y_e.end());
      auto& z_e = ids.incoming_spatial_flow_variable.at(e);
      variables.insert(variables.end(), z_e.begin(), z_e.end());
      variables.push_back(ids.flow_variable.at(e));
    }
    auto& lam_1 = ids.node_to_conversation_of_spatial_flow_multiplier.at(node_index);
    variables.insert(variables.end(), lam_1.begin(), lam_1.end());
    variables.push_back(ids.node_to_conversation_of_flow_multiplier.at(node_index));

    for (auto e : node.outgoing_edges) {
      auto& y_e = ids.outgoing_spatial_flow_variable.at(e);
      variables.insert(variables.end(), y_e.begin(), y_e.end());
      variables.push_back(ids.flow_variable.at(e));
    }
    return variables;
  }

  ConvexSetNode(const Graph& graph, const int node_index) : LUSolver(ConcatenateVariablesInLocalOrdering(graph, node_index)) {

    auto& node = graph.nodes_.at(node_index);
    const auto& variables = shared_variables();
    int num_supernodes = node.incoming_edges.size() * (2 * node.spatial_dimension + 1) + node.spatial_dimension + 1;
    int num_separator_no_fill = node.outgoing_edges.size() * (node.spatial_dimension + 1);

    std::vector<int> supernodes;
    supernodes.insert(supernodes.begin(), variables.begin(), variables.begin()  + num_supernodes);

    std::vector<int> separators;
    separators.insert(separators.begin(), variables.begin() + num_supernodes, variables.end());

    SetSupernodes(supernodes);
    SetSeparators(separators);

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

//  
//  Given node elimination sequence
//   (Out-going y)  (In coming z) (Incoming Flow) 
//   
//   Elimination order:
//
//    (ye, ze, phie)_{incoming edges}, conservation of flow multipliers, 
//
//   Equations: grad_{ye, ze, phie} = 0, 
//   conservation of flow: ze + yf = 0. phie + phif = 0
//
//   Separators: yf and phif of out-going edges.
//

// To assign variables, use DFS. 
//
//



Graph MakeGraph() {
  Node n1;
  Node n2;
  Node n3;

  std::vector<Node> nodes(3);
  nodes[0].outgoing_edges.push_back(1); 
  nodes[1].outgoing_edges.push_back(2);

  int i = 0;
  for (auto& n : nodes) {
    for (auto& e : n.outgoing_edges) {
      nodes[e].incoming_edges.push_back(i);
    }
    i++;
  }

  Graph graph(nodes);
  graph.BuildSpanningTree(0);
  graph.AssignEliminationOrder();
  return graph;
}

GTEST_TEST(GraphOfConvexSets, PrintSparsity) {
  //NodeData node_data;
  //// The 
  //node_data.variable_ids.outgoing_spatial_flow = std::vector<std::vector<int>>{{5, 6, 7}, {8, 9, 10}, {11, 12, 13}};
  //node_data.variable_ids.incoming_spatial_flow = std::vector<std::vector<int>>{{14, 15, 16}, {17, 18, 19}};
  //node_data.variable_ids.incoming_flows = std::vector{0, 1};

  //node_data.variable_ids.conversation_of_spatial_flow_multiplier = std::vector<int>{{20, 21, 22}};
  //node_data.variable_ids.conversation_of_flow_multiplier = 32; 

  //node_data.variable_ids.outgoing_flows = std::vector{2, 3, 4};
  //node_data.variable_ids.outgoing_spatial_flow_separator = std::vector<std::vector<int>>{  {23, 24, 25}, {26, 27, 28}, {29, 30, 31}  };

  Graph graph = MakeGraph();

  ConvexSetNode node(graph, 0);
  node.Assemble();
  MatrixXd supernode_submatrix = node.MakeSuperNodeSubmatrix();

  MatrixXd temp(33, 33);
  node.MakeKKTMatrix(&temp);
}




} // namespace conex
