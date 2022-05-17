#include "directed_graph.h"

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "gtest/gtest.h"
#include "gcs_solver.h"
#include <fstream>

namespace conex {

MatrixXd SparsityMask(const Eigen::MatrixXd& x) {
  MatrixXd y = x;
  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      if (y(i, j) != 0) {
        y(i, j) = 1;
      }
    }
  }
  return y;
}


void PrintCommaInitFormat(const Eigen::MatrixXd& matrix, 
                          const std::string& filename) {

  std::ofstream output;
  output.open(filename.c_str());
  if (output) {
    Eigen::IOFormat comma_init(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
    output << SparsityMask(matrix).format(comma_init);
    output.close();
  } else {
    throw std::runtime_error("Failed to open file.");

  }
}
struct Factorizations {
  Eigen::SparseMatrix<double> factor_amd;
  Eigen::SparseMatrix<double> factor_top;
  Eigen::SparseMatrix<double> kkt_matrix_primal_dual_order;
  Eigen::SparseMatrix<double> kkt_matrix_amd_order;
  Eigen::SparseMatrix<double> kkt_matrix_topological_order;
};

void PrintFactorizations(const Factorizations& f, const std::string& folder) {
  PrintCommaInitFormat(f.kkt_matrix_primal_dual_order, folder +"/kkt_matrix.txt");
  PrintCommaInitFormat(f.factor_top, folder+"/sparsity_cholesky_factor_topological.txt");
  PrintCommaInitFormat(f.factor_amd, folder+"/sparsity_cholesky_factor_amd.txt");
  PrintCommaInitFormat(f.kkt_matrix_amd_order, folder+"/kkt_matrix_amd_order.txt");
  PrintCommaInitFormat(f.kkt_matrix_topological_order, folder+"/kkt_matrix_topological_order.txt");
}

Factorizations GetFactorizations(const GraphData& graph) {
  Factorizations y;
#if 1
  int num_edges = graph.edges.size();
  #else
  int spatial_dim = 5;
  int num_edges = 12;
  GraphData graph = MakePath(num_edges, spatial_dim);
  #endif

  
  GraphSolver solver(graph);

  GraphSolver::FactorizationMode mode;
  mode.left_looking = false;
  mode.custom_block_inverse = false;
  solver.SetFactorizationMode(mode);
  solver.tree_solver().Assemble();

  const std::vector<VariablePartition> edge_vars {VariablePartition::incoming_spatial_variable, 
                                                  VariablePartition::outgoing_spatial_variable, 
                                                  VariablePartition::flow_variable}; 
  for (int i = 0; i < num_edges; i++) {
    for (const auto& var_i : edge_vars) {
      int k = 1;
      for (const auto& var_j : edge_vars) {
        solver.quadratic_edge_cost_mutable(i, var_i, var_j).setConstant(.001*(i + k));
        k++;
      }
      solver.quadratic_edge_cost_mutable(i, var_i, var_i).diagonal().setConstant(100 * (i+1));
    }
  }

  y.kkt_matrix_topological_order = solver.tree_solver().MakeSparseKKTMatrix(true).selfadjointView<Eigen::Lower>();
  Eigen::PermutationMatrix<-1> P = solver.variable_to_primal_dual_order_position().transpose();
  y.kkt_matrix_primal_dual_order = P*y.kkt_matrix_topological_order*P.transpose();


  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, 
                        Eigen::NaturalOrdering<int>> llt_top(y.kkt_matrix_topological_order);

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, 
  Eigen::Lower> llt_amd(y.kkt_matrix_primal_dual_order);

  Eigen::PermutationMatrix<-1> Pamd;
  Eigen::AMDOrdering<int>amd; amd(y.kkt_matrix_primal_dual_order, Pamd);
  y.kkt_matrix_amd_order = Pamd.transpose() * y.kkt_matrix_primal_dual_order *  Pamd;

  if (llt_top.info() != Eigen::Success) {
    throw std::runtime_error("Top. LDLT failed.");
  }
  if (llt_amd.info() != Eigen::Success) {
    throw std::runtime_error("Amd LDLT amd failed.");
  }

  y.factor_amd =  llt_amd.matrixL();
  y.factor_top =  llt_top.matrixL();
  return y;
}


GraphData MakeGraph(Eigen::MatrixXd& adj_matrix, const std::vector<int>& topological_order, 
                    int spatial_dim) {
  GraphData graph; 
  int num_nodes = adj_matrix.rows();
  graph.nodes.resize(num_nodes);

  for (int i = 0; i < num_nodes; i++) {
    for (int j  = i+1; j < num_nodes; j++) {
      if (adj_matrix(i, j) != 0) {
        graph.edges.push_back(Edge{});
        if (topological_order.at(i) > topological_order.at(j)) {
          graph.edges.back().source = i; 
          graph.edges.back().sink = j;  
        } else {
          graph.edges.back().source = j; 
          graph.edges.back().sink = i; 
        }
      }
    }
  }

  for (int i = 0; i < num_nodes; i++) {
    graph.nodes.at(i).spatial_dimension = spatial_dim;
  }
  return graph;
}

GraphData GenerateRandomDAG(int num_nodes, double edge_density, int spatial_dim) {
  CONEX_DEMAND(edge_density <= 1 && edge_density >= 0, "invalid edge density");
  GraphData graph; 
  graph.nodes.resize(num_nodes);
  std::vector<int> topological_order(num_nodes);
  for (int i = 0; i < num_nodes; i++) {
    topological_order.at(i) = num_nodes - 1 - i;
  }

  MatrixXd M(num_nodes, num_nodes);
  M.setZero();

  // Add path
  #if 1
  for (int i = 0; i < num_nodes; i++) {
    if (i < num_nodes -1 ) {
    M(i, i + 1) = 1;
    M(i + 1, i) = 1;
    }
  }
  int edge_count = num_nodes - 1;
  #else 
  int edge_count = 0;
  #endif

  // Add random edges
  int target = edge_density * .5 * (num_nodes *  num_nodes - num_nodes);
  while (edge_count < target) {
    int node_1 = rand() % num_nodes;
    int node_2 = rand() % num_nodes;
    if (node_1 != node_2) {
      if (M(node_1, node_2) == 0) {
        edge_count++;
      }
      M(node_1, node_2) = 1;
      M(node_2, node_1) = 1;
    }
  }
  DUMP(M);
  return MakeGraph(M, topological_order, spatial_dim);
}

 //     a    c
//   0 -> 1 -> 2
// b |         | d
//   3   ->    4
//        e
//   
GTEST_TEST(EdgeOrdering, TestTopologicalSort) {
  Eigen::MatrixXd M(5, 5); M.setZero();
  M << 0, 1, 0, 1, 0,
       0, 0, 1, 1, 1,
       0, 0, 0, 0, 1,
       0, 0, 0, 0, 1,
       0, 0, 0, 0, 0;
  //M << 0, 1, 0, 0, 0,
  //     0, 0, 1, 0, 0,
  //     0, 0, 0, 1, 0,
  //     0, 0, 0, 0, 1,
  //     0, 0, 0, 0, 0;
  int spatial_dim = 2;
  //GraphData graph_data = MakeGraph(M, {4, 3, 2, 1, 0}, spatial_dim);
  GraphData graph_data = GenerateRandomDAG(15, .3, spatial_dim); 
  Graph graph(graph_data.nodes, graph_data.edges);
  graph.BuildSpanningTree();
  graph.SortEdgeListInReverseTopologicalOrder();

  std::vector<int> edge_to_order_position = graph.edge_to_topological_order_position();

  int i = 0;
  int j = 0;
  std::vector<int> node_to_topological_order_position = graph.node_to_topological_order_position();
  for (auto& ei : graph_data.edges) {
    int j = 0;
    for (auto& ej : graph_data.edges) {
      int first_edge = i;
      int second_edge = j;
      if (edge_to_order_position.at(first_edge) > edge_to_order_position.at(second_edge)) {
        std::swap(second_edge, first_edge);
      }
      // If edge comes first, then it's sink cannot be smaller in topological ordering.
      EXPECT_TRUE(node_to_topological_order_position.at(graph_data.edges.at(first_edge).sink) >=
                  node_to_topological_order_position.at(graph_data.edges.at(second_edge).sink));
      j++;
    }
    i++;
  }

  //std::vector<int> primal_dual_order_to_edge_topological = graph.primal_dual_to_node_edge_order();
  std::vector<int> primal_dual_order_to_edge_topological = graph.primal_dual_to_interleaved_topological_order();

  auto f = GetFactorizations(graph_data);
  Eigen::PermutationMatrix<-1> P;
  P.indices() = Eigen::Map<Eigen::VectorXi>(primal_dual_order_to_edge_topological.data(), primal_dual_order_to_edge_topological.size());

  Eigen::SparseMatrix<double> kkt_matrix_edge_top_order = P.transpose()*f.kkt_matrix_primal_dual_order*P;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, 
                        Eigen::NaturalOrdering<int>> llt_edge_top(kkt_matrix_edge_top_order);

 Eigen::SparseMatrix<double> l_top; 
 if (llt_edge_top.info() == Eigen::Success) {
    l_top = llt_edge_top.matrixL();
 }
 f.kkt_matrix_topological_order = kkt_matrix_edge_top_order;
 f.factor_top = l_top;
 PrintFactorizations(f, "nonunique");
}

} // namespace conex
