#include "gcs_solver.h"
#define CONEX_ENABLE_TIMER 1
#include <stdlib.h>

#include <map>
#include <memory>
#include <numeric>
#include <stack>

#include "gtest/gtest.h"
#include "profile.h"
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {
#if 0
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

GTEST_TEST(GraphOfConvexSets, Path) {
  int num_edges = 3;
  int spatial_dim = 2;

  std::vector<int> node_to_parent_in_spanning_tree_reference(num_edges + 1);
  std::iota(
      node_to_parent_in_spanning_tree_reference.begin(), 
      node_to_parent_in_spanning_tree_reference.end(),
      1);

  GraphData graph = MakePath(num_edges, spatial_dim);
  GraphSolver solver(graph);
}

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
#endif
GraphData MakeGraph(Eigen::MatrixXd& adj_matrix,
                    const std::vector<int>& topological_order,
                    int spatial_dim) {
  GraphData graph;
  int num_nodes = adj_matrix.rows();
  graph.nodes.resize(num_nodes);

  graph.edges.push_back(Edge{});
  graph.edges.back().source = -1;
  graph.edges.back().sink = 0;
  for (int i = 0; i < num_nodes; i++) {
    for (int j = i + 1; j < num_nodes; j++) {
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

GTEST_TEST(FillIn, NonUnique) {
  Eigen::MatrixXd M(5, 5);
  M.setZero();
  M << 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0;
  int spatial_dim = 5;
  auto stats = Profile(MakeGraph(M, {4, 3, 2, 1, 0}, spatial_dim), {});

  std::cout << ", AMD fill-in: "
            << (double)stats.non_zeros_amd / stats.non_zeros_lower_tri << ", "
            << ", AMD solve: " << stats.factor_time_amd << ", "

            << ", Nat fill-in: "
            << (double)stats.non_zeros_natural / stats.non_zeros_lower_tri
            << ", "
            << ", Nat solve: " << stats.factor_time_natural
            << ", Solve: " << (double)stats.factor_time << ", "
            << ", Solve Left: " << stats.factor_time_left_looking << ", "
            << ", Solve CustomInv: " << stats.factor_time_custom_inverse
            << ", ";
}

GTEST_TEST(FillIn, AMDFailure) {
  Eigen::MatrixXd M(5, 5);
  M.setZero();
  M(5, 5);
  M << 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1,
      0;

  int spatial_dim = 5;
  auto stats = Profile(MakeGraph(M, {4, 3, 2, 1, 0}, spatial_dim), {});

  std::cout << ", AMD fill-in: "
            << (double)stats.non_zeros_amd / stats.non_zeros_lower_tri << ", "
            << ", AMD solve: " << stats.factor_time_amd << ", "

            << ", Nat fill-in: "
            << (double)stats.non_zeros_natural / stats.non_zeros_lower_tri
            << ", "
            << ", Nat solve: " << stats.factor_time_natural
            << ", Solve: " << (double)stats.factor_time << ", "
            << ", Solve Left: " << stats.factor_time_left_looking << ", "
            << ", Solve CustomInv: " << stats.factor_time_custom_inverse
            << ", ";
}

}  // namespace conex
