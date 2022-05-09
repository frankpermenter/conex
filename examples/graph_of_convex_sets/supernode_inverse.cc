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

using DenseBlockBase = CholeskySolver<Eigen::RLDLT<MatrixXd>, true>;

using Parameters = SupernodeSubmatrix::Parameters;


int NumberOfSeparators(const Parameters& p) {
  return p.spatial_dimension * 2 + 1;
}

#define CONEX_NO_COPY_NO_MOVE(T)\
T(const T&) = delete;\
T(T&&) = delete;\
T& operator=(const T&) = delete;\
T& operator=(T&&) = delete;\

} // namespace

Eigen::MatrixXd dummy;
class DenseBlock : public DenseBlockBase {
  CONEX_NO_COPY_NO_MOVE(DenseBlock)
 public:
  static int SupernodeSize(const Parameters& p) { return (p.spatial_dimension + 1) * (p.num_edges + 1); }
  using FactorizationMethod = CholeskySolver<Eigen::RLDLT<MatrixXd>, true>;
  DenseBlock(const Parameters& p, 
  Eigen::Ref<MatrixXd> full_matrix) :
    DenseBlockBase(full_matrix.bottomRightCorner(SupernodeSize(p), SupernodeSize(p)),
    dummy, 
    dummy)  {
    int offset = p.spatial_dimension * p.num_edges; 
    std::vector<int> supernodes((p.spatial_dimension + 1) * (p.num_edges + 1));
    std::iota(supernodes.begin(), supernodes.end(), offset);
    SetSupernodes(supernodes);
    SetSeparators({});
  }
  void SetData(Eigen::Ref<MatrixXd> full_matrix) {
    supernode_submatrix() = full_matrix.bottomRightCorner(supernodes().size(), supernodes().size());
  }
};

class IncomingSpatialVariableBlock : public IncomingSpatialVariableBlockBase  {
  CONEX_NO_COPY_NO_MOVE(IncomingSpatialVariableBlock)
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

    seperator_global_offset_1_ = EdgeSeparatorGlobalOffset(p, edge_number);
    seperator_global_size_1_ = p.spatial_dimension + 1;
    std::vector<int> separators(NumberOfSeparators(p));
    std::iota(separators.begin(), separators.begin() + seperator_global_size_1_, seperator_global_offset_1_);

    seperator_global_offset_2_ = p.num_edges * (2 * p.spatial_dimension + 1);
    seperator_global_size_2_ = p.spatial_dimension;
    std::iota(separators.begin() + seperator_global_size_1_, separators.end(), seperator_global_offset_2_);

    SetSeparators(separators);
    DoInitialize();
  }

   void SetData(Eigen::Ref<MatrixXd> full_matrix) {
     int offset = supernodes().at(0);
     int size_super = supernodes().size();
     supernode_submatrix() = full_matrix.block(offset, offset, size_super, size_super);
     separator_rows().topRows(seperator_global_size_1_) = full_matrix.block(seperator_global_offset_1_ , offset, seperator_global_size_1_, size_super);
     separator_rows().bottomRows(seperator_global_size_2_) = full_matrix.block(seperator_global_offset_2_ , offset, seperator_global_size_2_, size_super);
     separator_schur_complement().setZero();
   }
   int seperator_global_offset_1_ = 0;
   int seperator_global_size_1_ = 0;
   int seperator_global_offset_2_ = 0;
   int seperator_global_size_2_ = 0;
};

using T = SupernodeSubmatrix;
T::~SupernodeSubmatrix() {}

T::SupernodeSubmatrix(const Parameters& p, Eigen::Ref<Eigen::MatrixXd> full_matrix) : 
incoming_blocks_(p.num_edges) {
  CONEX_CHECK(p.num_edges > 0 && p.spatial_dimension > 0);
  tree_solver_ = std::make_unique<SymmetricLinearSystemTreeSolver>();
  for (int i = 0; i < p.num_edges; i++) {
    incoming_blocks_.at(i) = std::make_unique<IncomingSpatialVariableBlock>(p, i);
  }
  dense_block_ = std::make_unique<DenseBlock>(p,  full_matrix);
  tree_solver_->AddSubsystem(dense_block_.get());
  for (int i = 0; i < p.num_edges; i++) {
    tree_solver_->AddSubsystem(incoming_blocks_.at(i).get());
  }
  std::vector<int> node_to_parent(incoming_blocks_.size() + 1, 0);
  node_to_parent.at(0) = -1;
  tree_solver_->SetEliminationTree(node_to_parent);
}

void T::SetData(Eigen::Ref<Eigen::MatrixXd> full_matrix) {
  for (auto& i : incoming_blocks_) {
    i->SetData(full_matrix);
  }
  //dense_block_->SetData(full_matrix);
//  tree_solver_->Assemble();
}

} // namespace conex
