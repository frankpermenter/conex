// Compare three chordal decomposition approaches for banded max-cut SDP.
// (1) Single block (ground truth)
// (2) Splitting variables along clique tree
// (3) Per-clique Z_k variables + equality constraints (CVXOPT-style)
#include <cstdio>
#include <cmath>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"
#include "conex/algorithms/geodesic_ipm.h"
using namespace conex;
using SpMat = Eigen::SparseMatrix<double>;

SpMat SpSym(int n, int i, int j, double v) {
  SpMat M(n,n); M.insert(i,j)=v; if(i!=j) M.insert(j,i)=v;
  M.makeCompressed(); return M;
}

double Solve(Problem& p) {
  auto cost_orig = p.linear_cost();
  SolverConfiguration cfg;
  auto solver = Solver::Build(p, cfg);
  auto* kkt = solver.solver();
  // Pad cost to full KKT dimension (includes equality dual vars).
  int nv = kkt->number_of_variables();
  Eigen::VectorXd cost = Eigen::VectorXd::Zero(nv);
  cost.head(cost_orig.size()) = cost_orig;
  auto c = kkt->MakeSolverRHS();
  c = kkt->MakeBlockVariable(cost);
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  auto r = SolveGeodesicThetaContinuation(*kkt, c, W, 500, 1, 1e-8, false);
  return r.x.size() > 0 ? cost.dot(r.x) : 0;
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

// (1) Single block: diag(y) - C >= 0.
Problem BuildSingle(int n, const Eigen::MatrixXd& C) {
  Problem p;
  std::vector<int> vars(n); std::iota(vars.begin(), vars.end(), 0);
  std::vector<SpMat> A;
  for (int i = 0; i < n; ++i) A.push_back(SpSym(n, i, i, 1));
  p.AddPSDConstraint(A, (-C).sparseView(1e-15), vars, false);
  p.SetLinearCost(Eigen::VectorXd::Ones(n));
  return p;
}

// (3) Equality constraint approach (range-space decomposition).
// Variables: y_0..y_{n-1}, then Z_k entries for each clique.
// Z_k is |K|×|K| PSD. The entries of Z_k are separate variables.
// Equality: for each (i,j) in pattern, Σ_k Z_k(i_loc,j_loc) = M(i,j)
//   where M(i,j) = y_i*δ_{ij} - C(i,j).
Problem BuildEquality(int n, int bw, const Eigen::MatrixXd& C) {
  Problem p;
  int s = bw + 1;  // clique size
  int nc = n - bw; // number of cliques
  int zsize = s * (s + 1) / 2;  // entries per Z_k (upper triangle)

  // Variable layout: y_0..y_{n-1}, then Z_0 entries, Z_1 entries, ...
  // Z_k upper-triangle entry (a,b) with a<=b: index = n + k*zsize + tri(a,b).
  auto tri = [&](int a, int b) { return a*s - a*(a-1)/2 + (b-a); };
  auto zvar = [&](int k, int a, int b) {
    int aa = std::min(a,b), bb = std::max(a,b);
    return n + k * zsize + tri(aa, bb);
  };
  int total = n + nc * zsize;

  // PSD constraints: Z_k >= 0 for each clique k.
  for (int k = 0; k < nc; ++k) {
    std::vector<SpMat> A_list;
    std::vector<int> vars;
    for (int a = 0; a < s; ++a) {
      for (int b = a; b < s; ++b) {
        A_list.push_back(SpSym(s, a, b, 1));
        vars.push_back(zvar(k, a, b));
      }
    }
    SpMat B(s, s); // B = 0 (all structure from Z variables)
    p.AddPSDConstraint(A_list, B, vars, false);
  }

  // Equality constraints: for each (i,j) in pattern with i<=j:
  //   Σ_{k: i,j ∈ K_k} Z_k(i-k, j-k) = y_i*δ_{ij} - C(i,j).
  // Rearrange: Σ Z_k(...) - y_i*δ_{ij} = -C(i,j).
  // Conex form: Cx = d where x includes y's and Z entries.
  std::vector<Eigen::Triplet<double>> eq_trips;
  std::vector<double> eq_rhs;
  int eq_row = 0;
  for (int i = 0; i < n; ++i) {
    for (int j = i; j < std::min(n, i + bw + 1); ++j) {
      // All cliques containing both i and j:
      // K_k contains i,j iff k <= i and k+bw >= j, i.e., k >= j-bw.
      // So k in [max(0,j-bw), min(i, nc-1)].
      for (int k = std::max(0, j-bw); k <= std::min(i, nc-1); ++k) {
        int a = i - k, b = j - k;
        eq_trips.emplace_back(eq_row, zvar(k, a, b), 1.0);
      }
      // -y_i if diagonal (i==j):
      if (i == j) {
        eq_trips.emplace_back(eq_row, i, -1.0);
      }
      eq_rhs.push_back(-C(i, j));
      eq_row++;
    }
  }
  int neq = eq_row;
  SpMat Ceq(neq, total);
  Ceq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  Eigen::VectorXd d(neq);
  for (int i = 0; i < neq; ++i) d(i) = eq_rhs[i];

  // Primal vars for equality: all variables.
  std::vector<int> all_vars(total);
  std::iota(all_vars.begin(), all_vars.end(), 0);
  p.AddEqualityConstraint(Ceq, d, all_vars);

  // Cost: min Σ y_i. Z entries have cost 0.
  Eigen::VectorXd cost = Eigen::VectorXd::Zero(total);
  cost.head(n).setOnes();
  p.SetLinearCost(cost);
  return p;
}

int main() {
  printf("%-5s %2s | %12s | %12s | %12s\n",
         "n", "bw", "single", "equality", "diff");
  printf("%s\n", std::string(55, '-').c_str());

  for (auto [n, bw] : std::vector<std::pair<int,int>>{
        {3,1},{5,1},{5,2},{10,1},{10,2}}) {
    auto C = BandedLaplacian(n, bw);
    auto p1 = BuildSingle(n, C);
    auto p3 = BuildEquality(n, bw, C);
    double v1 = Solve(p1);
    double v3 = Solve(p3);
    printf("n=%2d bw=%d | %12.6f | %12.6f | %.2e %s\n",
           n, bw, v1, v3, std::abs(v1-v3),
           std::abs(v1-v3) < 1e-3 ? "OK" : "MISMATCH");
    fflush(stdout);
  }
}
