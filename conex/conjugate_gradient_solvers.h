#pragma once
#include "conex/kkt_solver.h"
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

struct ConstrainedLeastSquaresConjugateGradientSolverConfig {
  int iteration_limit = 10;
};

class ConstrainedLeastSquaresConjugateGradientSolver {
 public:
  ConstrainedLeastSquaresConjugateGradientSolver(
      const std::vector<std::vector<int>>& cliques_of_G,
      const std::vector<SupernodalAssemblerBase*>& clique_assemblers_of_G,
      const std::vector<std::vector<int>>& non_zero_columns_of_B,
      const std::vector<std::vector<double>>& entries_of_B);

  bool Factor();

  void Assemble();

  void Solve(const Eigen::VectorXd& f, const Eigen::VectorXd& g,
             Eigen::VectorXd* y, Eigen::VectorXd* z, bool use_llt);

  Eigen::MatrixXd KKTMatrix();

 private:
  SupernodalKKTSolver inverse_of_G_;
  bool factored_ = false;
  bool assembled_ = false;
  const std::vector<std::vector<int>> non_zero_columns_of_B_;
  const std::vector<std::vector<double>> entries_of_B_;

  Eigen::VectorXd SchurComplementConjugateGradientSolver(
      const Eigen::VectorXd& x);
  Eigen::VectorXd EvaluateEquationOperator(const Eigen::VectorXd& d);
  Eigen::VectorXd EvaluateEquationOperatorTranspose(const Eigen::VectorXd& d);
};

}  // namespace conex
