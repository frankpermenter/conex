#include "supernode_inverse.h"

#include <Eigen/Dense>
#include "conex/kkt_tree_solver.h"
#include "conex/cholesky_solvers.h"
#include "conex/RLDLT.h"
#include <numeric>

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace {
using IncomingSpatialVariableBlockBase = 
KKTCholeskySystem<CholeskySolver<Eigen::LLT<MatrixXd>, true>>;

using DenseBlockBase = 
KKTCholeskySystem<CholeskySolver<Eigen::RLDLT<MatrixXd>, true>>;

using Parameters = SupernodeSubmatrix::Parameters;


int NumberOfSeparators(const Parameters& p) {
  return p.spatial_dimension * 2 + 1;
}

class IncomingSpatialVariableBlock : public IncomingSpatialVariableBlockBase  {
 public:

  static int SupernodeGlobalOffset(const Parameters& p, int edge_number) {
    return edge_number *  p.spatial_dimension;
  }

  static int EdgeSeparatorGlobalOffset(const Parameters& p, int edge_number) {
    return p.spatial_dimension * p.num_edges +  edge_number * (p.spatial_dimension + 1);
  }

  static int FlowMultiplierGlobalOffset(const Parameters& p) {
    return (2 * p.spatial_dimension + 1) * p.num_edges;
  }

  IncomingSpatialVariableBlock(const Parameters& p, int edge_number) {
    std::vector<int> supernodes(p.spatial_dimension);
    std::iota(supernodes.begin(), supernodes.end(), SupernodeGlobalOffset(p, edge_number));
    SetSupernodes(supernodes);

    std::vector<int> separators(NumberOfSeparators(p));
    std::iota(separators.begin(), separators.begin() + p.spatial_dimension + 1, 
    EdgeSeparatorGlobalOffset(p, edge_number));

    SetSupernodes(supernodes);
    SetSeparators(separators);
    Initialize();
  }
};

class DenseBlock : public DenseBlockBase {
 public:
  using FactorizationMethod = CholeskySolver<Eigen::RLDLT<MatrixXd>, true>;
  DenseBlock(int num_edges,  int spatial_dimension) {
    int offset = spatial_dimension * num_edges; 
    std::vector<int> supernodes((spatial_dimension + 1) * (num_edges + 1));
    std::iota(supernodes.begin(), supernodes.end(), offset);
    SetSupernodes(supernodes);
    Initialize();
  }
};
} // namespace

using T = SupernodeSubmatrix;
T::SupernodeSubmatrix(const Parameters& p) : incoming_blocks_(p.num_edges) {
  tree_solver_ = std::make_unique<SymmetricLinearSystemTreeSolver>();
  auto& tree_solver = *dynamic_cast<SymmetricLinearSystemTreeSolver*>(tree_solver_.get());
  for (int i = 0; i < p.num_edges; i++) {
    incoming_blocks_.at(i) = std::make_unique<IncomingSpatialVariableBlock>(p, i);
  }
  tree_solver.AddSubsystem(dense_block_.get());
  for (int i = 0; i < p.num_edges; i++) {
    tree_solver.AddSubsystem(incoming_blocks_.at(i).get());
  }
  std::vector<int> node_to_parent(incoming_blocks_.size() + 1, 0);
  node_to_parent.at(0) = -1;
  tree_solver.Finalize(node_to_parent);
}

void T::SetData(Eigen::Ref<Eigen::MatrixXd> full_matrix) {

}

} // namespace conex
