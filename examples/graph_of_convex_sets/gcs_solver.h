#pragma once

#include "conex/kkt_tree_solver.h" 
#include "kkt_subsystem.h"
#include "directed_graph.h"

#include "graph_data.h"

namespace conex {

/* Given a directed, acyclic graph (DAG) with vertex set V, edgeset E, source vertex s and sink vertex t, 
 * this class solves an optimization problems of the form
 *
 *    min.       \sum_{e \in E} f_e (w_e)
 *    subject to. flow constraints,
 *
 * where f_e is a quadratic polynomial in edge variable w_e. The flow constraints
 * are defined by partitioning each w_e as 
 *
 *     w_e = (y_e, z_e, phi_e) \in R^{d_u} \times R^{d_v} \times R, 
 *
 * where d_u and d_v denote the "spatial dimensions" of the vertices u \in V and v \in V connected 
 * by e \in E.  They take the form:
 *
 *    For all vertices v \in V not equal to the source or sink:
 *
 *         \sum{e \in Incoming Edges} z_e = \sum{e \in Outgoing Edges} y_e  
 *         \sum{e \in Incoming Edges} phi_e = \sum{e \in Outgoing Edges} \phi_e
 *
 *    For the source vertex s:
 *
 *         \sum{e \in Outgoing} y_e = source_spatial_value.
 *         \sum{e \in Outgoing} phi_e = 1.
 *
 *    For the sink vertex t:
 *
 *         \sum{e \in Incoming} z_e = sink_spatial_value.
 *         \sum{e \in Incoming} phi_e = 1.
 *
 */
class GraphSolver {
 public:

 /*  Throws if the graph contains cycles, has multiple sources or has multiple sinks*/
  GraphSolver(const GraphData& graph_data);

  // Provides direct access to the quadratic cost matrices for each block of f_e. 
  Eigen::Ref<Eigen::MatrixXd> quadratic_edge_cost_mutable(int edge_number, VariablePartition row_block, VariablePartition col_block);
  Eigen::Ref<Eigen::MatrixXd> linear_cost(int edge_number, VariablePartition row_block);

  struct FactorizationMode {
    bool left_looking = false;
    bool custom_block_inverse = false;
  };
  void SetFactorizationMode(const FactorizationMode&);
  SymmetricLinearSystemTreeSolver& tree_solver() { return tree_solver_; }

 private:
  std::vector<std::unique_ptr<ConvexSetNode>> nodes_;
  SymmetricLinearSystemTreeSolver tree_solver_;
  Graph graph_;
};

}
