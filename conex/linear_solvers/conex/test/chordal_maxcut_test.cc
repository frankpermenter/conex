// Test DecomposeChordalPSD on banded max-cut SDPs.
#include <cstdio>
#include <cmath>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"
#include "conex/common/chordal_decomp.h"
#include "conex/algorithms/geodesic_ipm.h"
using namespace conex;

double Solve(Problem& p) {
  auto cost = p.linear_cost();
  SolverConfiguration cfg;
  auto solver = Solver::Build(p, cfg);
  auto* kkt = solver.solver();
  int nv = kkt->number_of_variables();
  Eigen::VectorXd c_padded = Eigen::VectorXd::Zero(nv);
  c_padded.head(cost.size()) = cost;
  auto c = kkt->MakeSolverRHS();
  c = kkt->MakeBlockVariable(c_padded);
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  auto r = SolveGeodesicThetaContinuation(*kkt, c, W, 500, 1, 1e-8, false);
  return r.x.size() > 0 ? c_padded.dot(r.x) : 0;
}

Eigen::MatrixXd BandedLaplacian(int n, int bw) {
  std::srand(42);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = i+1; j < std::min(n, i+bw+1); ++j) {
      double w = 0.5 + (double)std::rand() / RAND_MAX;
      C(i,j) = -0.25*w; C(j,i) = -0.25*w;
      C(i,i) += 0.25*w; C(j,j) += 0.25*w;
    }
  return C;
}

Problem BuildSingle(int n, const Eigen::MatrixXd& C) {
  Problem p;
  std::vector<int> vars(n); std::iota(vars.begin(), vars.end(), 0);
  std::vector<Eigen::SparseMatrix<double>> A;
  for (int i = 0; i < n; ++i) {
    Eigen::SparseMatrix<double> Ei(n,n); Ei.insert(i,i)=1; Ei.makeCompressed();
    A.push_back(std::move(Ei));
  }
  p.AddPSDConstraint(A, (-C).sparseView(1e-15), vars, false);
  p.SetLinearCost(Eigen::VectorXd::Ones(n));
  return p;
}

int main() {
  printf("%-5s %2s | %12s | %12s | %10s\n",
         "n", "bw", "single", "split", "diff");
  printf("%s\n", std::string(55, '-').c_str());

  // Banded graphs.
  printf("\n--- Banded graphs ---\n");
  for (auto [n, bw] : std::vector<std::pair<int,int>>{
        {3,1},{5,1},{5,2},{10,1},{10,2},{20,3}}) {
    auto C = BandedLaplacian(n, bw);
    auto p1 = BuildSingle(n, C);
    auto p2 = DecomposeChordalPSD(p1);
    double v1 = Solve(p1);
    double v2 = Solve(p2);
    printf("band n=%2d bw=%d | %12.6f | %12.6f | %10.2e %s\n",
           n, bw, v1, v2, std::abs(v1-v2),
           std::abs(v1-v2) < 1e-3 ? "OK" : "MISMATCH");
    fflush(stdout);
  }

  // Star graphs: center node 0 connected to all others.
  // Cliques: {0,1}, {0,2}, ..., {0,n-1}. All share node 0.
  printf("\n--- Star graphs ---\n");
  for (int n : {4, 6, 10, 20}) {
    std::srand(42);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n, n);
    for (int j = 1; j < n; ++j) {
      double w = 0.5 + (double)std::rand() / RAND_MAX;
      C(0, j) = -0.25 * w; C(j, 0) = -0.25 * w;
      C(0, 0) += 0.25 * w; C(j, j) += 0.25 * w;
    }
    auto p1 = BuildSingle(n, C);
    auto p2 = DecomposeChordalPSD(p1);
    double v1 = Solve(p1);
    double v2 = Solve(p2);
    printf("star n=%2d      | %12.6f | %12.6f | %10.2e %s\n",
           n, v1, v2, std::abs(v1-v2),
           std::abs(v1-v2) < 1e-3 ? "OK" : "MISMATCH");
    fflush(stdout);
  }
}
