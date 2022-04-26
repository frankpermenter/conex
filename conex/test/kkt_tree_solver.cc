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

struct Edge {
  int source;
  int sink;
};

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
};


struct Variables {
  std::vector<int> edge_to_flow_variable;
  // Map incoming edge label to spatial variable
  std::vector<std::vector<int>> edge_to_incoming_spatial_flow_variable;
  std::vector<std::vector<int>> edge_to_outgoing_spatial_flow_variable;
  std::vector<std::vector<int>> node_to_conversation_of_spatial_flow_multiplier; 
  std::vector<int> node_to_conversation_of_flow_multiplier; 
};


struct Node {
  std::vector<int> incoming_edges;
  std::vector<int> outgoing_edges;
  int spatial_dimension;

  int parent_in_spanning_tree = 0;
  std::vector<int> children_in_spanning_tree;
};

class Graph {
 public:
  Graph(std::vector<Node> nodes, std::vector<Edge> edges) : nodes_(std::move(nodes)),  edges_(std::move(edges)) { 
    int num_nodes = nodes_.size();
    int num_edges = edges_.size();
    ids_.edge_to_outgoing_spatial_flow_variable.resize(num_edges); 
    ids_.edge_to_incoming_spatial_flow_variable.resize(num_edges); 
    ids_.node_to_conversation_of_spatial_flow_multiplier.resize(num_nodes); 
    ids_.node_to_conversation_of_flow_multiplier.resize(num_nodes); 
    ids_.edge_to_flow_variable.resize(num_edges);
  }

  void AssignEliminationOrder() {
    int offset = 0;
    for (auto& root : roots_) {
      offset = AssignEliminationOrderHelper(root, offset);
    }
  }

  // Do a pass to assign elimination position to each variable
  int AssignEliminationOrderHelper(int node_index,  int offset) {
    CONEX_CHECK(node_index < nodes_.size());

    for (auto& child : nodes_.at(node_index).children_in_spanning_tree) {
      offset = AssignEliminationOrderHelper(child, offset);
    }

    // Assign variable 
    for (auto e : nodes_.at(node_index).incoming_edges) {
      // Spatial y_e
      for (int i = 0; i < nodes_.at(node_index).spatial_dimension; ++i) {
        ids_.edge_to_outgoing_spatial_flow_variable.at(e).push_back(offset);
        offset++;
      }
      // Spatial z_e
      for (int i = 0; i < nodes_.at(node_index).spatial_dimension; ++i) {
        ids_.edge_to_incoming_spatial_flow_variable.at(e).push_back(offset);
        offset++;
      }
      ids_.edge_to_flow_variable.at(e) = offset;
      offset++; // phi_e
    }

    for (int i = 0; i < nodes_.at(node_index).spatial_dimension; ++i) {
      ids_.node_to_conversation_of_spatial_flow_multiplier.at(node_index).push_back(offset);
      offset++; 
    }
    ids_.node_to_conversation_of_flow_multiplier.at(node_index) = offset;
    offset++;
    return offset;
  }

  void BuildSpanningTree(int root) {
    std::vector<int> visited(nodes_.size(), 0);

    int parent = root;
    roots_.push_back(root);
    visited.at(parent) = 1;
    nodes_[parent].parent_in_spanning_tree = -1;

    std::stack<int> nodes_to_visit;
    nodes_to_visit.push(parent);
    while (nodes_to_visit.size() > 0) {
      parent = nodes_to_visit.top(); nodes_to_visit.pop();
      for (auto& e : nodes_.at(parent).outgoing_edges) {
        CONEX_CHECK(edges_.at(e).source == parent);
        int child = edges_.at(e).sink;
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
  std::vector<Edge> edges_;
  Variables ids_;
};

class ConvexSetNode : public LUSolver {
 public:
  static std::vector<int> ConcatenateVariablesInLocalOrdering(const Graph& graph, const int node_index) {
    auto ids = graph.ids_;
    std::vector<int> variables;
    auto& node = graph.nodes_.at(node_index);
    for (auto e : node.incoming_edges) {
      auto& y_e = ids.edge_to_outgoing_spatial_flow_variable.at(e);
      variables.insert(variables.end(), y_e.begin(), y_e.end());
      auto& z_e = ids.edge_to_incoming_spatial_flow_variable.at(e);
      variables.insert(variables.end(), z_e.begin(), z_e.end());
      variables.push_back(ids.edge_to_flow_variable.at(e));
    }
    auto& lam_1 = ids.node_to_conversation_of_spatial_flow_multiplier.at(node_index);
    variables.insert(variables.end(), lam_1.begin(), lam_1.end());
    variables.push_back(ids.node_to_conversation_of_flow_multiplier.at(node_index));

    for (auto e : node.outgoing_edges) {
      auto& y_e = ids.edge_to_outgoing_spatial_flow_variable.at(e);
      variables.insert(variables.end(), y_e.begin(), y_e.end());
      variables.push_back(ids.edge_to_flow_variable.at(e));
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
    num_incoming = node.incoming_edges.size();
    num_outgoing = node.outgoing_edges.size();
    spatial_dim = node.spatial_dimension;

  }


  int num_supernodes() {
      return supernodes().size();
  } 

  int num_separators() {
      return separators().size();
  } 
  


  Eigen::MatrixXd MakeSuperNodeSubmatrix() {
    Eigen::MatrixXd Q(num_supernodes(), num_supernodes());
    Q.setZero();
    int offset = 0;

    // Fill y_e, z_e, phi_e all incoming e.
    for (int i = 0; i < num_incoming; i++) {
      Q.block(offset, offset, spatial_dim*2 + 1, spatial_dim*2 + 1).setConstant(.01);
      Q.block(offset, offset, spatial_dim*2 + 1, spatial_dim*2 + 1).diagonal().setConstant(1);
      offset += 2 * spatial_dim + 1;
    }

    // Spatial flow 
    //
    int offset_row = offset;
    int offset_col = 0;
    for (int i = 0; i < num_incoming; i++) {
      for (int i = 0; i < num_outgoing; i++) {
        Q.block(offset_row, offset_col, spatial_dim, spatial_dim).setIdentity();
        offset_col += 2 * spatial_dim + 1;
      }
    }

    offset_row += spatial_dim;
    offset_col = 2*spatial_dim;
    // Flow conservation
    for (int i = 0; i < num_incoming; i++) {
      Q.block(offset_row, offset_col, 1, num_incoming).setConstant(-10);
      offset_col += 2 * spatial_dim + 1;
    }
    return Q;
    DUMP(Q);
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
    int offset_row = 0;
    int offset_col = (2*spatial_dim + 1) * num_incoming;
    for (int i = 0; i < num_outgoing; i++) {
      Q.block(offset_row, offset_col, spatial_dim, spatial_dim).diagonal().setConstant(-2);
      offset_row += spatial_dim + 1;
    }
    offset_col += spatial_dim;
    offset_row = spatial_dim;
    for (int i = 0; i < num_outgoing; i++) {
      Q(offset_row, offset_col) = -1;
      offset_row += spatial_dim;
    }
    return Q;
  }



 private:

  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    auto data = MakeSeperatorMatrix();
    DUMP(data);
    DUMP(separator_rows_);
    DUMP(separators());
    DUMP(supernodes());
    CONEX_CHECK(data.rows() == separator_rows_.rows());
    CONEX_CHECK(data.cols() == separator_rows_.cols());
    separator_rows_ = data;

    data = MakeSuperNodeSubmatrix();
    CONEX_CHECK(data.rows() == supernode_submatrix_.rows());
    CONEX_CHECK(data.cols() == supernode_submatrix_.cols());
    supernode_submatrix_ = data; 
    Eigen::LDLT<MatrixXd> llt(supernode_submatrix_);
    CONEX_CHECK(llt.info() == Eigen::Success);
    DUMP(separator_rows_ * llt.solve(separator_rows_.transpose()));
    DUMP(supernode_submatrix_);
    DUMP(separator_rows_);
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



Graph MakePath(int num_edges, int spatial_dim) {
  std::vector<Node> nodes(num_edges+1);
  std::vector<Edge> edges(num_edges);
  int i = 0;
  for (auto& e : edges) {
    e.source = i;
    e.sink = i + 1;
    i++;
  }

  for (auto& n : nodes) {
    n.spatial_dimension = spatial_dim;
  }

  i = 0;
  for (auto& e : edges) {
    nodes.at(e.source).outgoing_edges.push_back(i);
    nodes.at(e.sink).incoming_edges.push_back(i);
    i++;
  }

  Graph graph(nodes, edges);
  graph.BuildSpanningTree(0);
  graph.AssignEliminationOrder();
  return graph;
}

GTEST_TEST(GraphOfConvexSets, PrintSparsity) {

  int num_edges = 3;
  int dim = 2;
  Graph graph = MakePath(num_edges, dim);
  std::vector<ConvexSetNode> nodes;
  for (int i = 0; i < num_edges + 1; i++) {
    nodes.emplace_back(graph, i);
  }

  int i = 0;
  std::vector<int> roots;
  for (auto& n : graph.nodes_) {
    int parent = n.parent_in_spanning_tree;
    if (parent > 0) {
      nodes.at(parent).AddChild(&nodes.at(i));
    } else {
      roots.push_back(i);
    }
    i++;
  }

  nodes.at(roots.at(0)).Assemble();


  int vars_per_node = (2*dim + 1) + (dim + 1); 
  MatrixXd temp(vars_per_node  * num_edges, vars_per_node  * num_edges);

  nodes.at(roots.at(0)).MakeKKTMatrix(&temp);
  DUMP(temp);
}




} // namespace conex
