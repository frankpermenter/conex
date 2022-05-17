#include "conex/static_subsystem.h"
#include "conex/cholesky_solvers.h"

namespace conex {
using T = KKTAssemblerToSubsystemAdapter;

namespace {

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


} // namespace 
T::KKTAssemblerToSubsystemAdapter(SupernodalAssemblerBase* base) : assembler_(base) { 
  //using SystemType = KKTCholeskySystem<CholeskySolver<Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>>, false>>;
  using SystemType = KKTCholeskySystem<CholeskySolver<Eigen::LLT<Eigen::MatrixXd>, false>>;
  kkt_subsystem_ = std::make_unique<SystemType>(assembler_->variables());
}

void T::UpdateData() {
  int n1 = kkt_subsystem_->supernodes().size();
  int n2 = kkt_subsystem_->separators().size();
  assembler_->SetDenseData();
  MatrixXd Q_in_elimination_order_(n1 + n2, n1 + n2);
  MatrixXd Q_ = assembler_->submatrix_data()->G;
  AssignSubmatrix(Q_, Q_in_elimination_order_, kkt_subsystem_->variable_to_local_elimination_rank());
  kkt_subsystem_->supernode_submatrix() = Q_in_elimination_order_.topLeftCorner(n1, n1);
  kkt_subsystem_->separator_rows() = Q_in_elimination_order_.bottomLeftCorner(n2, n1);
  kkt_subsystem_->separator_schur_complement() = Q_in_elimination_order_.bottomRightCorner(n2, n2);
}

} // namespace conex
