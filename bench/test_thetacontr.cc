// Minimal test: ThetaContR on the n=10 random LP from compare_embedding.
#include <cstdio>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/conex.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"

using namespace conex;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
  const int n = 10;
  const int m = 5;
  srand(42);
  MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd x0 = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);
  VectorXd b = Ad * x0;
  VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      trips.emplace_back(i, j, Ad(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  Eigen::SparseMatrix<double> I(n, n);
  I.setIdentity();
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model model;
  model.AddLinearConstraint(I, VectorXd::Zero(n), vars);
  model.AddEqualityConstraint(A, b, vars);
  model.SetLinearCost(c);

  SolverConfiguration cfg;
  auto solver = Solver::Build(model, cfg);

  // Run ThetaContR with verbose, gap switching policy.
  ThetaContinuationR algo;
  algo.tolerance = 1e-10;
  algo.verbose = true;
  algo.policy = [](double gap, double, int) { return gap < 0; };
  auto result = solver.Solve(algo);

  printf("\nResult: fac=%d obj=%.6f stat=%.2e compl=%.2e eq=%.2e tau=%.2e kappa=%.2e\n",
         result.factorizations, result.objective,
         result.duals.stationarity_gradient.norm(),
         result.optimality.complementarity,
         result.duals.eq_residual.empty() ? 0.0 : result.duals.eq_residual[0].norm(),
         result.tau, result.kappa);

  return 0;
}
