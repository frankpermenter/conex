#include "conex/common/sparse_linear_constraint.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <numeric>
#include <set>
#include <vector>

#include "conex/common/constraint_manager.h"
#include "conex/tree_solver/kkt_solver_factory.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Clock = std::chrono::high_resolution_clock;

double us(Clock::time_point t0, Clock::time_point t1) {
  return std::chrono::duration<double, std::micro>(t1 - t0).count();
}

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

double BenchAssembleAndFactor(const Eigen::SparseMatrix<double>& A,
                              bool scatter_to_parent, int iters) {
  int nv = A.cols();
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
  cfg.tree.max_merge_supernode_size = 0;
  auto solver = MakeTreeSolver(&cm, cfg);
  if (scatter_to_parent) {
    solver->SetScatterToParent(true);
  }

  // Warm up.
  solver->AssembleAndFactor();

  // Verify correctness.
  VectorXd x_true = VectorXd::Random(nv);
  MatrixXd Ad(A);
  VectorXd rhs = Ad.transpose() * (Ad * x_true);
  VectorXd sol = solver->Solve(rhs);
  double err = (sol - x_true).norm() / x_true.norm();
  if (err > 1e-8) {
    fprintf(stderr, "ERROR: residual %.2e (scatter_to_parent=%d)\n",
            err, scatter_to_parent);
  }

  // Benchmark.
  auto t0 = Clock::now();
  for (int i = 0; i < iters; i++) {
    solver->AssembleAndFactor();
  }
  auto t1 = Clock::now();
  return us(t0, t1) / iters;
}

}  // namespace conex

int main() {
  using namespace conex;

  printf("%-30s  %8s %8s %8s\n", "Graph", "Legacy", "ScatPar", "Speedup");
  printf("%s\n", std::string(65, '-').c_str());

  struct Case { int s; int sep; int k; };
  std::vector<Case> cases = {
      {1, 10, 30}, {1, 10, 60}, {1, 20, 30},
      {5, 10, 30}, {5, 10, 60},
      {10, 10, 30}, {10, 10, 60},
      {10, 20, 10}, {10, 20, 30},
  };

  for (const auto& c : cases) {
    srand(42);
    int rpc = c.s + c.sep + c.s;
    int nv = c.k * c.s + c.sep;
    auto A = MakeChain(c.s, c.sep, c.k, rpc);
    int iters = std::max(50, 5000 / nv);

    double legacy = BenchAssembleAndFactor(A, false, iters);
    double scatter = BenchAssembleAndFactor(A, true, iters);

    char label[64];
    snprintf(label, sizeof(label), "chain s=%d sep=%d k=%d n=%d",
             c.s, c.sep, c.k, nv);
    printf("%-30s  %7.0fus %7.0fus  %5.2fx\n",
           label, legacy, scatter, legacy / scatter);
  }

  return 0;
}
