#pragma once

#include "conex/RLDLT.h"
#include "conex/kkt_solver_interface.h"
#include "conex/supernodal_assembler.h"
#include "conex/supernodal_solver.h"

namespace conex {

enum : int {
  CONEX_LLT_FACTORIZATION = 0,
  CONEX_LDLT_FACTORIZATION = 1,
  CONEX_QR_FACTORIZATION = 2,
};

class SupernodalKKTSolver : public KKTSolverBase {
 public:
  SupernodalKKTSolver(const std::vector<std::vector<int>>& cliques,
                      const std::vector<std::vector<int>>& dual_vars);

  SupernodalKKTSolver(const std::vector<std::vector<int>>& cliques);

  SupernodalKKTSolver(const std::vector<std::vector<int>>& cliques,
                      int num_vars, const std::vector<int>& order,
                      const std::vector<std::vector<int>>& supernodes,
                      const std::vector<std::vector<int>>& separators);

  template <typename SupernodalAssemblerDerivedClass>
  void Bind(const std::vector<SupernodalAssemblerDerivedClass*>&
                supernodal_assembler) {
    DoBind(data, mat.workspace_, supernodal_assembler);
    for (auto v : supernodal_assembler) {
      assembler.push_back(v);
    }
  }

  void SetIterativeRefinementIterations(int x) {
    iterative_refinement_iterations_ = x;
  }
  void SetSolverMode(int mode);

  Eigen::VectorXd Solve(const Eigen::VectorXd& b,
                        bool permute_to_elimination_order = true) const;
  int SizeOfSystem() const { return permutation_to_elimination_order_.rows(); }
  const Eigen::PermutationMatrix<-1>& permutation_to_elimination_order() const {
    return permutation_to_elimination_order_;
  }
  const Eigen::PermutationMatrix<-1>& permutation_from_elimination_order()
      const {
    return permutation_from_elimination_order_;
  }

 private:
  void DoAssemble() override;
  bool DoFactor() override;
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool permute_to_elimination_order) const override;
  Eigen::MatrixXd DoKKTMatrix(bool permute_to_elimination_order) const override;

  void RelabelCliques(MatrixData* data_ptr);
  // Copies of inputs.
  const std::vector<std::vector<int>> cliques_;
  const std::vector<std::vector<int>> dual_variables_;
  MatrixData data;
  SparseTriangularMatrix mat;
  std::vector<Eigen::RLDLT<Eigen::Ref<Eigen::MatrixXd>>> factorization;
  Eigen::PermutationMatrix<-1> permutation_from_elimination_order_;
  Eigen::PermutationMatrix<-1> permutation_to_elimination_order_;
  mutable Eigen::VectorXd b_permuted_;
  std::vector<SupernodalAssemblerBase*> assembler;
  bool factorization_regularized_ = false;
  int iterative_refinement_iterations_ = 0;
  mutable Eigen::MatrixXd kkt_matrix_;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_decomp_;
  int mode_ = CONEX_LLT_FACTORIZATION;
};

}  // namespace conex
