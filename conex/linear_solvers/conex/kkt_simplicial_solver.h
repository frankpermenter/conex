#pragma once
#include <memory>

#include "conex/kkt_solver_interface.h"
#include "conex/kkt_tree_solver.h"
#include <Eigen/Dense>

namespace conex {

class SparseFactorization;
class EigenSparseCholesky : public KKTSolverBase {
 public:
  EigenSparseCholesky(
      std::unique_ptr<SymmetricLinearSystemTreeSolver>&& solver);
  ~EigenSparseCholesky();

 private:
  Eigen::MatrixXd DoKKTMatrix(
      bool permute_to_elimination_order = true) const override;
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool in_original_order) const;

  void DoAssemble() override;
  bool DoAssembleAndFactor() override;
  bool DoFactor() override;
  std::unique_ptr<SymmetricLinearSystemTreeSolver> solver_;
  std::unique_ptr<SparseFactorization> factorization_;
};

}  // namespace conex
