// Benchmark: random conic program over a product of m exponential cones.
//
// Problem:
//   min  c'x
//   s.t. b_i - A_i x ∈ K_exp,  i = 1, ..., m
//
// where K_exp = cl{(x,y,z) : y*exp(x/y) ≤ z, y > 0} (3D exponential cone).
//
// Generation:
//   - A_i ∈ R^{3×p} random, b_i = (0, 1, e+1) (interior of K_exp).
//   - c = -Σ A_i^T ∇F(b_i) so x=0 is approximately optimal.
//
// Also writes the problem in Clarabel form to stdout (--dump) so that
// a Rust Clarabel binary can solve the same instance.
//
// Usage:
//   ./benchmark_expcone [m] [p] [seed] [--dump] [--verbose]

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/exp_cone_ops.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/algorithms/solve_strategies.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using conex::EuclideanJordanAlgebra::ExpConeOps;
using Clock = std::chrono::high_resolution_clock;

struct ExpConeProblem {
  int m;                          // number of exp cones
  int p;                          // number of decision variables
  std::vector<MatrixXd> A;       // A[i] is 3 x p
  std::vector<Vector3d> b;       // b[i] is 3 x 1
  VectorXd c;                    // p x 1
};

// Generate a random exp cone problem.
ExpConeProblem GenerateProblem(int m, int p, int seed) {
  srand(seed);
  ExpConeProblem prob;
  prob.m = m;
  prob.p = p;
  prob.A.resize(m);
  prob.b.resize(m);
  prob.c = VectorXd::Zero(p);

  ExpConeOps ops;
  for (int i = 0; i < m; ++i) {
    prob.A[i] = 0.3 * MatrixXd::Random(3, p);
    prob.b[i] = Vector3d(0, 1.0, std::exp(1.0) + 1.0);
    // Cost: c = -Σ A_i^T ∇F(b_i) so x=0 is near-optimal.
    double gi[3];
    ExpConeOps::BarrierGrad(prob.b[i](0), prob.b[i](1), prob.b[i](2), gi);
    prob.c -= prob.A[i].transpose() * Eigen::Map<Vector3d>(gi);
  }
  return prob;
}

// Dump problem in Clarabel-compatible format:
//   Line 1: m p
//   Line 2: c (p values)
//   For each cone i:
//     Line: A_i row-major (3*p values)
//     Line: b_i (3 values)
void DumpProblem(const ExpConeProblem& prob) {
  printf("%d %d\n", prob.m, prob.p);
  for (int j = 0; j < prob.p; ++j) printf("%.17e%c", prob.c(j), j+1<prob.p?' ':'\n');
  for (int i = 0; i < prob.m; ++i) {
    for (int r = 0; r < 3; ++r)
      for (int j = 0; j < prob.p; ++j)
        printf("%.17e%c", prob.A[i](r, j), (r*prob.p+j+1 < 3*prob.p) ? ' ':'\n');
    printf("%.17e %.17e %.17e\n", prob.b[i](0), prob.b[i](1), prob.b[i](2));
  }
}

int main(int argc, char** argv) {
  int m = 10, p = 6, seed = 42;
  bool dump = false, verbose = false;
  std::string algo_filter;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--dump") == 0) { dump = true; continue; }
    if (strcmp(argv[i], "--verbose") == 0 || strcmp(argv[i], "-v") == 0) { verbose = true; continue; }
    if (strcmp(argv[i], "--algo") == 0 && i + 1 < argc) { algo_filter = argv[++i]; continue; }
    if (i == 1) m = atoi(argv[i]);
    else if (i == 2) p = atoi(argv[i]);
    else if (i == 3) seed = atoi(argv[i]);
  }

  auto prob = GenerateProblem(m, p, seed);
  if (dump) { DumpProblem(prob); return 0; }

  // Build conex Model.
  ExpConeOps ops;
  conex::Model model;
  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  VectorXd z0(3 * m);
  for (int i = 0; i < m; ++i) {
    Eigen::SparseMatrix<double> As = prob.A[i].sparseView();
    model.AddBarrierConstraint(As, prob.b[i], vars, &ops);
    z0.segment(3 * i, 3) = prob.b[i];
  }
  model.SetLinearCost(prob.c);

  printf("=== Exp cone: m=%d cones, p=%d vars, seed=%d ===\n", m, p, seed);
  printf("  %-18s  %5s  %5s  %5s  %12s  %10s  %8s\n",
         "Algorithm", "fac", "sol", "iter", "objective", "gap", "ms");
  printf("  %s\n", std::string(78, '-').c_str());

  auto should_run = [&](const char* name) {
    return algo_filter.empty() ||
           std::string(name).find(algo_filter) != std::string::npos;
  };

  auto run = [&](const char* name, auto strategy) {
    if (!should_run(name)) return;
    auto solver = conex::Solver::Build(model);
    auto cm = solver.MakeCompiledModel();
    auto t0 = Clock::now();
    auto result = strategy.Run(cm);
    double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    // Compute objective c'x.
    double obj = 0;
    if (result.x.size() > 0) {
      for (int j = 0; j < p; ++j) obj += prob.c(j) * result.x[j];
    }
    printf("  %-18s  %5d  %5d  %5d  %12.6e  %10.2e  %8.1f\n",
           name, result.total_factorizations, result.total_solves,
           result.iterations, obj, result.complementarity, ms);
  };

  run("BarrierLP", conex::GeodesicBarrierLP{1e-8, 200, 0, verbose, z0});
  run("BarrierLP+frzJ", conex::GeodesicBarrierLP{1e-8, 200, 1, verbose, z0});
  run("BarrierTC", conex::GeodesicBarrierThetaContinuation{1e-8, 200, 0, verbose, z0});
  run("BarrierTC+frzJ", conex::GeodesicBarrierThetaContinuation{1e-8, 200, 1, verbose, z0});

  return 0;
}
