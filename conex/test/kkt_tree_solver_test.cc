#include <map>
#include <memory>
#include <numeric>
#include <stack>

#include "conex/error_checking_macros.h"
#include "conex/kkt_subsystem.h"
#include "conex/RLDLT.h"
#include "conex/test/directed_graph.h"
#include "conex/kkt_tree_solver.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

MatrixXd Sparsity(const Eigen::MatrixXd& d) {
  MatrixXd y = d;
  for (int i = 0; i < d.rows(); i++) {
    for (int j = 0; j < d.cols(); j++) {
      if (d(i, j) != 0) {
        y(i, j) = 1;
      }
    }
  }
  return y;
}

using Eigen::MatrixXd;


class LUSolver : public KKTSubsystem {
 public:
  LUSolver(std::vector<int> vars) : KKTSubsystem(vars, 0) {}

  void DoEliminateSupernodeColumns() override {
    lu_.compute(supernode_submatrix());
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    y = lu_.solve(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
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
    int n1 = Base::supernode_submatrix().rows();
    int n2 = Base::separator_rows_.rows();
    Q_in_elimination_order_.resize(n1 + n2, n1 + n2);
    AssignSubmatrix(Q_, Q_in_elimination_order_,
                    Base::variable_to_local_elimination_rank());
    DoAssemble();
  }

  MatrixXd submatrix() const { return Q_; }

 private:
  bool DoIsValidLeaf() override { return Q_.diagonal().norm() > 0; }
  void DoAssemble() {
    int n1 = Base::supernode_submatrix().rows();
    int n2 = Base::separator_rows_.rows();
    Base::supernode_submatrix() = Q_in_elimination_order_.topLeftCorner(n1, n1);
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


class ConvexSetNode : public LUSolver {
 public:
  static std::vector<int> ConcatenateVariablesInLocalOrdering(
      const Graph& graph, const int node_index) {
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
    auto& lam_1 =
        ids.node_to_conversation_of_spatial_flow_multiplier.at(node_index);
    variables.insert(variables.end(), lam_1.begin(), lam_1.end());
    variables.push_back(
        ids.node_to_conversation_of_flow_multiplier.at(node_index));

    for (auto e : node.outgoing_edges) {
      auto& y_e = ids.edge_to_outgoing_spatial_flow_variable.at(e);
      variables.insert(variables.end(), y_e.begin(), y_e.end());
      variables.push_back(ids.edge_to_flow_variable.at(e));
    }
    return variables;
  }

  ConvexSetNode(const Graph& graph, const int node_index)
      : LUSolver(ConcatenateVariablesInLocalOrdering(graph, node_index)) {
    auto& node = graph.nodes_.at(node_index);
    const auto& variables = shared_variables();
    int num_supernodes =
        node.incoming_edges.size() * (2 * node.spatial_dimension + 1) +
        node.spatial_dimension + 1;
    int num_separator_no_fill =
        node.outgoing_edges.size() * (node.spatial_dimension + 1);

    std::vector<int> supernodes;
    supernodes.insert(supernodes.begin(), variables.begin(),
                      variables.begin() + num_supernodes);

    std::vector<int> separators;
    separators.insert(separators.begin(), variables.begin() + num_supernodes,
                      variables.end());

    SetSupernodes(supernodes);
    SetSeparators(separators);
    num_incoming = node.incoming_edges.size();
    num_outgoing = node.outgoing_edges.size();
    spatial_dim = node.spatial_dimension;
  }

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
    Eigen::MatrixXd m1 = MakeSeperatorMatrix();
    Eigen::MatrixXd m2 = MakeSuperNodeSubmatrix();
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
      offset_row += spatial_dim;
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
    CONEX_CHECK(data.rows() == supernode_submatrix().rows());
    CONEX_CHECK(data.cols() == supernode_submatrix().cols());
    supernode_submatrix() = data;
  }

  int spatial_dim = 0;
  int num_incoming = 0;
  int num_outgoing = 0;
};

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

struct GraphData {
  std::vector<Node> nodes;
  std::vector<Edge> edges;
  int source = 0;
};

GraphData MakePath(int num_edges, int spatial_dim) {
  GraphData data; 
  data.edges.resize(num_edges + 1);
  data.nodes.resize(num_edges + 1);

  int i = -1;
  for (auto& e : data.edges) {
    e.sink = i + 1;
    e.source = i;
    i++;
  }

  for (auto& n : data.nodes) {
    n.spatial_dimension = spatial_dim;
  }

  return data;
}

GraphData MakeCycle(int num_edges, int spatial_dim) {
  GraphData data = MakePath(num_edges, spatial_dim);
  Edge edge;
  edge.source = data.nodes.size() - 1;
  edge.sink = 0;
  data.edges.push_back(edge);
  return data;
}

void Verify(const GraphData& data,
  std::vector<int> node_to_parent_in_spanning_tree_reference,
            int spatial_dim) {

  Graph graph(data.nodes, data.edges);

  graph.BuildSpanningTree();
  graph.AssignEliminationOrder();

  int num_nodes = graph.nodes_.size();
  int num_edges = graph.edges_.size();
  std::vector<std::unique_ptr<ConvexSetNode>> nodes(num_nodes);
  std::vector<std::unique_ptr<StaticSubsystem>> static_subsystems(num_nodes);

  for (int i = 0; i < num_nodes; i++) {
    nodes.at(i) = std::make_unique<ConvexSetNode>(graph, i);
  }

  for (int i = 0; i < num_nodes; i++) {
    static_subsystems.at(i) = std::make_unique<StaticSubsystem>(nodes.at(i)->Submatrix(), nodes.at(i)->shared_variables());
  }

  SymmetricLinearSystemTreeSolver system;
  #if 0
  for (auto& n : nodes) {
    system.AddSubsystem(n.get());
  }
  #else
  for (auto& n : static_subsystems) {
    system.AddSubsystem(n.get());
  }
  #endif

 EXPECT_EQ(graph.node_to_parent_in_spanning_tree(), node_to_parent_in_spanning_tree_reference);

  system.Finalize(graph.node_to_parent_in_spanning_tree(), true);
  system.Assemble();

  Eigen::RLDLT<MatrixXd> llt(system.KKTMatrix(true));
  DUMP((system.KKTMatrix(true)));
  DUMP(Sparsity(MatrixXd(llt.matrixL())));
}

#if 0
GTEST_TEST(GraphOfConvexSets, Path) {
  int num_edges = 3;
  int spatial_dim = 2;

  std::vector<int> node_to_parent_in_spanning_tree_reference(num_edges + 1);
  std::iota(
      node_to_parent_in_spanning_tree_reference.begin(), 
      node_to_parent_in_spanning_tree_reference.end(),
      1);

  GraphData graph = MakePath(num_edges, spatial_dim);
  Verify(graph, node_to_parent_in_spanning_tree_reference, spatial_dim);
}

#else
#if 0
//GTEST_TEST(GraphOfConvexSets, Cycle) {
//  int num_edges = 4;
//  int spatial_dim = 2;
//
//  std::vector<int> node_to_parent_in_spanning_tree_reference(num_edges + 1);
//  std::iota(
//      node_to_parent_in_spanning_tree_reference.begin(), 
//      node_to_parent_in_spanning_tree_reference.end(),
//      1);
//
//  GraphData graph = MakeCycle(num_edges, spatial_dim);
//  Verify(graph, node_to_parent_in_spanning_tree_reference, spatial_dim);
//}

GTEST_TEST(GraphOfConvexSets, TwoPaths) {

  int spatial_dim = 2;

  GraphData graph; 
  graph.nodes.resize(3);
  graph.edges.resize(4);

  for (auto& n : graph.nodes) {
    n.spatial_dimension = spatial_dim;
  }

 // 0 -
 // |   1
 // 2 -
  graph.edges.at(0).source = -1;
  graph.edges.at(0).sink = 0;
  graph.edges.at(1).source = 0;
  graph.edges.at(1).sink = 2;

  graph.edges.at(2).source = 0;
  graph.edges.at(2).sink = 1;
  graph.edges.at(3).source = 1;
  graph.edges.at(3).sink = 2;

  std::vector<int> node_to_parent_in_spanning_tree_reference{1, 2, -1};
  Verify(graph, node_to_parent_in_spanning_tree_reference, spatial_dim);
}
#else

GTEST_TEST(GraphOfConvexSets, TwoSegments) {
  int spatial_dim = 2;

  GraphData graph; 
  graph.nodes.resize(4);
  graph.edges.resize(6);

  for (auto& n : graph.nodes) {
    n.spatial_dimension = spatial_dim;
  }

  /* -  0 --
     |  |  1
     |  2--
     - -|
        3
  */


  graph.edges.at(0).source = -1;
  graph.edges.at(0).sink = 0;

  graph.edges.at(1).source = 0;
  graph.edges.at(1).sink = 1;
  graph.edges.at(2).source = 0;
  graph.edges.at(2).sink = 2;
  graph.edges.at(3).source = 0;
  graph.edges.at(3).sink = 3;

  graph.edges.at(4).source = 1;
  graph.edges.at(4).sink = 2;

  graph.edges.at(5).source = 2;
  graph.edges.at(5).sink = 3;

  std::vector<int> node_to_parent_in_spanning_tree_reference{1, 2, 3, -1};
  Verify(graph, node_to_parent_in_spanning_tree_reference, spatial_dim);
}
#endif

#endif

}  // namespace conex
