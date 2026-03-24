#include "conex/sparse_linear_constraint.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <numeric>
#include <set>
#include <vector>

#include "conex/constraint_manager.h"
#include "conex/kkt_solver_factory.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Chain of k cliques, supernode size s, overlap (separator) sep.
Eigen::SparseMatrix<double> MakeChain(int s, int sep, int k, int rpc) {
  int nv = k * s + sep;
  int cs = s + sep;
  int nr = k * rpc;
  std::vector<Eigen::Triplet<double>> t;
  for (int ci = 0; ci < k; ci++)
    for (int r = 0; r < rpc; r++)
      for (int j = 0; j < cs; j++)
        t.emplace_back(ci * rpc + r, ci * s + j,
                       0.5 + static_cast<double>(rand()) / RAND_MAX);
  Eigen::SparseMatrix<double> A_unperm(nr, nv);
  A_unperm.setFromTriplets(t.begin(), t.end());

  std::vector<int> perm(nv);
  std::iota(perm.begin(), perm.end(), 0);
  std::random_shuffle(perm.begin(), perm.end());
  Eigen::PermutationMatrix<Eigen::Dynamic> P(nv);
  for (int i = 0; i < nv; i++) P.indices()(i) = perm[i];
  return A_unperm * P;
}

}  // namespace conex

int main() {
  using namespace conex;
  srand(42);

  const int s = 10, sep = 10, k = 30;
  const int rpc = s + sep + s;
  const int num_iters = 5000;

  auto A = MakeChain(s, sep, k, rpc);
  int nv = A.cols();

  VectorXd x_true = VectorXd::Random(nv);
  Eigen::MatrixXd Ad(A);
  VectorXd rhs = Ad.transpose() * (Ad * x_true);
  VectorXd b0 = VectorXd::Zero(A.rows());

  auto slc = std::make_unique<SparseLinearConstraint>(A, b0);
  std::set<int> vs;
  for (const auto& sup : slc->row_supports()) vs.insert(sup.begin(), sup.end());
  std::vector<int> av(vs.begin(), vs.end());
  ConstraintManager cm(nv);
  auto asm_ = std::make_unique<SparseLinearConstraintAssembler>(std::move(slc), av);
  cm.AddCustomAssembler(asm_.get());
  SolverConfiguration cfg;
  cfg.num_threads = 1;
  auto solver = MakeTreeSolver(&cm, cfg);

  // Warm up.
  solver->AssembleAndFactor();
  solver->Solve(rhs);

  // Hot loop.
  for (int i = 0; i < num_iters; i++) {
    solver->AssembleAndFactor();
    VectorXd sol = solver->Solve(rhs);
  }

  solver->AssembleAndFactor();
  VectorXd sol = solver->Solve(rhs);
  printf("chain s=%d sep=%d k=%d  n=%d  iters=%d  err=%.2e\n",
         s, sep, k, nv, num_iters, (sol - x_true).norm() / x_true.norm());
  return 0;
}
