#define CONEX_ENABLE_TIMER 1
#include <map>
#include <memory>
#include <numeric>
#include <stack>

#include "conex/error_checking_macros.h"
#include "conex/RLDLT.h"
#include "directed_graph.h"
#include "kkt_subsystem.h"
#include "conex/kkt_tree_solver.h"
#include "convex_set_node_factory.h"
#include "gtest/gtest.h"
#include "profile.h"
#include <Eigen/Dense>

#include <stdlib.h>

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
#endif
#if 0
GraphData MakeTwoSegments() {
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

  //std::vector<int> v{0, 1, 2, 3};
  //std::vector<int> e{0, 1, 2, 3, 4, 5};

  std::vector<int> v{1, 3, 0, 2};
  std::vector<int> e{4, 2, 1, 3, 0, 5};
  graph.edges.at(e[0]).source = -1;
  graph.edges.at(e[0]).sink = v[0];

  graph.edges.at(e[1]).source = v[0];
  graph.edges.at(e[1]).sink = v[1];
  graph.edges.at(e[2]).source = v[0];
  graph.edges.at(e[2]).sink = v[2];
  graph.edges.at(e[3]).source = v[0];
  graph.edges.at(e[3]).sink = v[3];

  graph.edges.at(e[4]).source = v[1];
  graph.edges.at(e[4]).sink = v[2];

  graph.edges.at(e[5]).source = v[2];
  graph.edges.at(e[5]).sink = v[3];
  return graph;
}


GTEST_TEST(GraphOfConvexSets, TwoSegments) {
  GraphData graph = MakeTwoSegments();
  //Verify(graph, node_to_parent_in_spanning_tree_reference, spatial_dim);
  Profile(graph, {});
  auto stats = Profile(graph, {});

  std::cout 
            << ", AMD fill-in: " <<  (double) stats.non_zeros_amd/stats.non_zeros_lower_tri << ", " 
            << ", AMD solve: " <<  stats.factor_time_amd << ", " 

            << ", Nat fill-in: " <<  (double) stats.non_zeros_natural/stats.non_zeros_lower_tri << ", "
            << ", Nat solve: " <<  stats.factor_time_natural 
            //<< ", Custom Solve: " <<  (double) stats.factor_time << ", " 
            << ", Custom Solve Left: " <<  (double) stats.factor_time_left_looking << ", "
            << ", Custom Solve CustomInv: " <<  (double) stats.factor_time_custom_inverse << ", ";
}
#endif
#if 1

GraphData MakeGraph(Eigen::MatrixXd& adj_matrix, const std::vector<int>& topological_order) {
  GraphData graph; 
  int num_nodes = adj_matrix.rows();
  graph.nodes.resize(num_nodes);

  graph.edges.push_back(Edge{});
  graph.edges.back().source = -1;
  graph.edges.back().sink = 0;
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
  return graph;
}

GraphData GenerateRandomDAG(int num_nodes, double edge_density) {
  GraphData graph; 
  graph.nodes.resize(num_nodes);
  std::vector<int> topological_order(num_nodes);
  for (int i = 0; i < num_nodes; i++) {
    topological_order.at(i) = num_nodes - 1 - i;
  }

  MatrixXd M(num_nodes, num_nodes);
  M.setZero();

  // Add path
  #if 0
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
  return MakeGraph(M, topological_order);
}


void DoTest(int num_nodes, int spatial_dim, double edge_density, Time* stats_ptr) {
  auto& stats = *stats_ptr;

  GraphData graph = GenerateRandomDAG(num_nodes, edge_density);
  for (int i = 0; i < num_nodes; i++) {
    graph.nodes.at(i).spatial_dimension = spatial_dim;
  }
  stats = Profile(graph, {});

  std::cout << "\n nodes: " << num_nodes << ", Spatial dim: " << spatial_dim
            << ", AMD fill-in: " <<  (double) stats.non_zeros_amd/stats.non_zeros_lower_tri << ", " 
            << ", AMD solve: " <<  stats.factor_time_amd << ", " 

            << ", Nat fill-in: " <<  (double) stats.non_zeros_natural/stats.non_zeros_lower_tri << ", "
            << ", Nat solve: " <<  stats.factor_time_natural 
            << ", Solve: " <<  (double) stats.factor_time << ", " 
            << ", Solve Left: " <<   stats.factor_time_left_looking << ", "
            << ", Solve CustomInv: " <<   stats.factor_time_custom_inverse << ", ";
}

GTEST_TEST(GraphOfConvexSets, RandomDAG) {
  srand(4);
  //DoTest(10, 10);
  //DoTest(20, 10);
  //DoTest(30, 10);
  //DoTest(10, 20);
  //DoTest(20, 20);
  //DoTest(30, 20);
  //DoTest(10, 30);
  //DoTest(20, 30);
  Time stats;
  stats.factor_time_amd = 0;
  stats.factor_time_natural = 0;
  stats.factor_time = 0;
  for (int i = 0; i < 10; i++) {
    Time stats_i;
    //DoTest(4 /*nodes*/, 1 /*dim*/, .7 /*edge*/ , &stats_i);
    try {
    DoTest(20 /*nodes*/, 5 /*dim*/, .5 /*edge*/ , &stats_i);
    } catch(const std::exception& e) {

    }
    stats.factor_time_natural += stats_i.factor_time_natural;
    stats.factor_time += stats_i.factor_time;
  }
}
#endif

}  // namespace conex
