#include "profile.h"

#include "conex/kkt_tree_solver.h"
#include "convex_set_node_factory.h"
#include "gcs_solver.h"
#include "gtest/gtest.h"
#include "kkt_subsystem.h"
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

using Eigen::MatrixXd;
using Eigen::VectorXd;
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

using StaticAssembler = StaticSubsystem<false>;
Time Verify(const GraphData& data,
            std::vector<int> node_to_parent_in_spanning_tree_reference,
            int spatial_dim) {
  Time stats;
  Graph graph(data.nodes, data.edges);

  graph.BuildSpanningTree();
  graph.SortEdgeListInReverseTopologicalOrder();
  graph.AssignEliminationOrder();
  graph.IdentifyFillInEdges();

  int num_nodes = graph.nodes_.size();
  std::vector<std::unique_ptr<ConvexSetNode>> nodes(num_nodes);
  std::vector<std::unique_ptr<StaticAssembler>> static_subsystems(num_nodes);

  for (int i = 0; i < num_nodes; i++) {
    nodes.at(i) = MakeConvexSetNode(graph, i);
  }

  for (int i = 0; i < num_nodes; i++) {
    static_subsystems.at(i) = std::make_unique<StaticAssembler>(
        nodes.at(i)->Submatrix(), nodes.at(i)->shared_variables());
  }

  SymmetricLinearSystemTreeSolver system_using_custom_assemblers;
  SymmetricLinearSystemTreeSolver system_using_static_assemblers;
  for (auto& n : nodes) {
    system_using_custom_assemblers.AddSubsystem(n.get());
  }

  for (auto& n : static_subsystems) {
    system_using_static_assemblers.AddSubsystem(n.get());
  }

  EXPECT_EQ(graph.node_to_parent_in_spanning_tree(),
            node_to_parent_in_spanning_tree_reference);

  system_using_static_assemblers.Finalize(
      graph.node_to_parent_in_spanning_tree(), true);
  system_using_static_assemblers.Assemble();

  for (int i = 0; i < num_nodes; i++) {
    EXPECT_EQ(static_subsystems.at(i)->supernodes(), nodes.at(i)->supernodes());
    EXPECT_EQ(static_subsystems.at(i)->separators(), nodes.at(i)->separators());
  }

  system_using_custom_assemblers.SetEliminationTree(
      graph.node_to_parent_in_spanning_tree());
  system_using_custom_assemblers.Assemble();
  Eigen::MatrixXd M = system_using_custom_assemblers.KKTMatrix(true);

  START_LOG_TIMER
  system_using_custom_assemblers.Factor();
  END_LOG_TIMER(stats.factor_time)
  VectorXd x;
  x.setLinSpaced(M.cols(), -1, 1);
  VectorXd y = M * x;

  START_LOG_TIMER
  system_using_custom_assemblers.SolveInPlace(y, false);
  END_LOG_TIMER(stats.solve_time)

  EXPECT_NEAR((y - x).norm(), 0, 1e-12);

  Eigen::MatrixXd M2 = system_using_static_assemblers.KKTMatrix(true);
  EXPECT_NEAR((M2 - M).norm(), 0, 1e-12);
  y = M2 * x;
  system_using_static_assemblers.Factor();
  system_using_static_assemblers.SolveInPlace(y, false);
  EXPECT_NEAR((y - x).norm(), 0, 1e-12);
  return stats;
}

Time Profile(GraphSolver& graph_solver) {
  Time stats;
  auto system_using_custom_assemblers = graph_solver.tree_solver();

  GraphSolver::FactorizationMode mode;
  mode.left_looking = false;
  mode.custom_block_inverse = false;
  graph_solver.SetFactorizationMode(mode);

  system_using_custom_assemblers.Assemble();
  START_LOG_TIMER
  system_using_custom_assemblers.Factor();
  END_LOG_TIMER(stats.factor_time)

  mode.custom_block_inverse = true;
  graph_solver.SetFactorizationMode(mode);

  system_using_custom_assemblers.Assemble();
  START_LOG_TIMER
  system_using_custom_assemblers.Factor();
  END_LOG_TIMER(stats.factor_time_custom_inverse)

  mode.left_looking = true;
  mode.custom_block_inverse = false;
  graph_solver.SetFactorizationMode(mode);
  system_using_custom_assemblers.Assemble();
  START_LOG_TIMER
  system_using_custom_assemblers.Factor();
  END_LOG_TIMER(stats.factor_time_left_looking)

  system_using_custom_assemblers.Assemble();
  Eigen::SparseMatrix<double> M =
      system_using_custom_assemblers.MakeSparseKKTMatrix()
          .triangularView<Eigen::Lower>();
  VectorXd x;
  x.setLinSpaced(M.cols(), -1, 1);
  VectorXd y = M.selfadjointView<Eigen::Lower>() * x;
  VectorXd b = y;

  START_LOG_TIMER
  system_using_custom_assemblers.Factor();
  system_using_custom_assemblers.SolveInPlace(y, false);
  END_LOG_TIMER(stats.solve_time)
  EXPECT_NEAR((y - x).norm(), 0, 1e-10);

  Eigen::RLDLT<Eigen::MatrixXd> llt;
  y = M * x;
  START_LOG_TIMER
  llt.compute(M);
  END_LOG_TIMER(stats.factor_time_dense);

  stats.non_zeros_lower_tri = M.nonZeros();
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower,
                        Eigen::NaturalOrdering<int>>
      llt_sparse;

  START_LOG_TIMER
  llt_sparse.compute(M);
  END_LOG_TIMER(stats.factor_time_natural);

  if (llt_sparse.info() != Eigen::Success) {
    stats.factor_time_natural = -1;
    stats.non_zeros_natural = -1;
  } else {
    Eigen::SparseMatrix<double> factor = llt_sparse.matrixL();
    stats.non_zeros_natural = factor.nonZeros();
  }

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower> llt_amd;
  START_LOG_TIMER
  llt_amd.compute(M);
  END_LOG_TIMER(stats.factor_time_amd);
  if (llt_amd.info() != Eigen::Success) {
    stats.non_zeros_amd = -1;
    stats.factor_time_amd = -1;
  } else {
    Eigen::SparseMatrix<double> factor_amd = llt_amd.matrixL();
    stats.non_zeros_amd = factor_amd.nonZeros();
  }
  return stats;
}

Time Profile(const GraphData& data,
             std::vector<int> node_to_parent_in_spanning_tree_reference) {
  GraphSolver graph_solver(data);
  return Profile(graph_solver);
}

}  // namespace conex
