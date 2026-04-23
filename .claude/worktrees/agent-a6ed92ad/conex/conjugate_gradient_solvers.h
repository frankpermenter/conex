#pragma once
#include "conex/kkt_solver.h"
#include "conex/kkt_solver_interface.h"
#include "conex/supernodal_assembler.h"

namespace conex {
// Solves KKT system of form
//
//  G   B' [y] = f
//  B      [z]   g
//
// by applying CG gradient to the Schur complement system:
//
// B inv(G) B'z  = B inv(G) f - g.
//
//  y = Ginv(f - B' z)
//
// This solver is useful when evaluation of B, B' and inv(G) is
// inexpensive.

struct SparseEqualityConstraints {
  std::vector<std::vector<int>> columns;
  std::vector<std::vector<double>> matrix_entries;
  std::vector<double> affine_term;
};

struct ConstrainedLeastSquaresConjugateGradientSolverConfig {
  int iteration_limit = 10;
};

class ConstrainedLeastSquaresConjugateGradientSolver : public KKTSolverBase {
 public:
  ConstrainedLeastSquaresConjugateGradientSolver(
      std::unique_ptr<SupernodalKKTSolver>&& solver,
      const std::vector<std::vector<int>>& non_zero_columns_of_B,
      const std::vector<std::vector<double>>& entries_of_B);

  void Solve(const Eigen::VectorXd& f, const Eigen::VectorXd& g,
             Eigen::VectorXd* y, Eigen::VectorXd* z, bool use_llt) const;

  Eigen::VectorXd Solve(const Eigen::VectorXd& f) {
    using Eigen::VectorXd;
    VectorXd y = f;
    SolveInPlace(y);
    return y;
  }

  Eigen::VectorXd EvaluateEquationOperator(const Eigen::VectorXd& d) const;
  Eigen::VectorXd EvaluateEquationOperatorTranspose(
      const Eigen::VectorXd& d) const;
  std::unique_ptr<SupernodalKKTSolver> inverse_of_G_;
  int number_of_equations() const { return non_zero_columns_of_B_.size(); }

 private:
  void DoAssemble() override;
  bool DoFactor() override;
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool permute_to_elimination_order) const override;
  Eigen::MatrixXd DoKKTMatrix(bool permute_to_elimination_order) const override;

  std::vector<std::vector<int>> non_zero_columns_of_B_;
  std::vector<std::vector<double>> entries_of_B_;
  int number_of_variables() const { return inverse_of_G_->SizeOfSystem(); }

  Eigen::VectorXd SchurComplementConjugateGradientSolver(
      const Eigen::VectorXd& x) const;
  Eigen::VectorXd ApplyPreconditioner(const Eigen::VectorXd& x) const;
};

}  // namespace conex
