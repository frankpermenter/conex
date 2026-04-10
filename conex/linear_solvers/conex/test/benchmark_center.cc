// Benchmark centering: read a problem, compute minimum-norm mu,
// and run geodesic centering steps at that mu.
//
// Usage:
//   ./benchmark_center <file.dat-s|file.mps|file.cbf> [max_iters]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/cbf_reader.h"
#include "conex/common/eja_ops.h"
#include "conex/common/mps_reader.h"
#include "conex/common/problem.h"
#include "conex/common/sdpa_reader.h"
#include "conex/common/solver.h"

namespace conex {

using Eigen::VectorXd;

void RunCentering(Problem& problem, const std::string& name, int max_iters) {
  printf("=== %s ===\n", name.c_str());
  printf("  Variables: %d, Constraints: %d\n",
         problem.num_variables(), problem.num_constraints());

  auto t0 = std::chrono::high_resolution_clock::now();
  auto solver = Solver::Build(problem);
  auto t1 = std::chrono::high_resolution_clock::now();
  printf("  Build: %.0f ms\n",
         std::chrono::duration<double, std::milli>(t1 - t0).count());

  auto* kkt = solver.solver();
  auto cost_rhs = kkt->MakeSolverRHS();
  if (problem.has_linear_cost()) {
    cost_rhs = kkt->MakeBlockVariable(problem.linear_cost());
  } else {
    cost_rhs.SetZero();
  }

  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // Compute min-norm k from decomposition.
  kkt->SetScaling(W);
  kkt->AssembleAndFactor();

  RowSpace d0 = kkt->MakeRowSpace();
  RowSpace d1 = kkt->MakeRowSpace();
  {
    // Manually call ComputeDirectNewtonStep at k=0 and k=1 to get d0, d1.
    // d0 = d(k=0): direction with zero cost.
    // d1 = d(k=1) - d(k=0): cost component.
    // Actually, use the fact that d(k) = d0 + k*d1 where:
    //   d0 from RHS = A^T(2W)
    //   d1 from RHS = -(cost + A^T P(W)b)
    // These are what ComputeDecomposition gives, but it's static.
    // Instead, compute d at k=0 and k=1 directly.
    RowSpace d_at_0 = kkt->MakeRowSpace();
    RowSpace d_at_1 = kkt->MakeRowSpace();
    Eigen::VectorXd y0_dummy, y1_dummy;

    // d at k=0: solve with zero cost.
    {
      auto zero_cost = kkt->MakeSolverRHS();
      zero_cost.SetZero();
      RowSpace b = kkt->GetAffineTerm();
      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      auto y = kkt->MakeSolverRHS();
      y.SetZero();
      RowSpace v = W;
      v *= 2.0;
      kkt->AccumulateAtranspose(v, y);
      kkt->SolveSolverRHS(y);
      RowSpace row = kkt->MakeRowSpace();
      kkt->MultiplyA(y, row);
      d_at_0 = addScaled(b, row, 0, -1.0);  // -Ay
      d_at_0 = quadraticRepresentation(sqrtW, d_at_0);
      RowSpace ones = kkt->MakeRowSpace();
      setOnes(ones);
      d_at_0 += ones;
    }

    // d at k=1: solve with full cost.
    {
      RowSpace b = kkt->GetAffineTerm();
      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      auto y = kkt->MakeSolverRHS();
      y = cost_rhs;
      y *= -1;
      RowSpace v = addScaled(quadraticRepresentation(W, b), W, -1, 2.0);
      kkt->AccumulateAtranspose(v, y);
      kkt->SolveSolverRHS(y);
      RowSpace row = kkt->MakeRowSpace();
      kkt->MultiplyA(y, row);
      d_at_1 = addScaled(b, row, -1, -1.0);
      d_at_1 = quadraticRepresentation(sqrtW, d_at_1);
      RowSpace ones = kkt->MakeRowSpace();
      setOnes(ones);
      d_at_1 += ones;
    }

    // d0 = d_at_0, d1 = d_at_1 - d_at_0.
    d0 = d_at_0;
    d1 = d_at_1 - d_at_0;
  }

  double d0d1 = dot(d0, d1);
  double d1sq = squaredNorm(d1);
  double d0sq = squaredNorm(d0);
  double k_min = (d1sq > 1e-30) ? std::max(1e-6, -d0d1 / d1sq) : 1.0;
  double mu_min = 1.0 / (k_min * k_min);

  // d at k_min.
  RowSpace d_at_kmin = addScaled(d0, d1, 1.0, k_min);
  double d_inf_at_kmin = normInf(d_at_kmin);
  double d_sq_at_kmin = squaredNorm(d_at_kmin);

  printf("\n  Min-norm decomposition:\n");
  printf("    ||d0||_inf = %.4e, ||d0||^2 = %.4e\n",
         normInf(d0), d0sq);
  printf("    ||d1||_inf = %.4e, ||d1||^2 = %.4e\n",
         normInf(d1), d1sq);
  printf("    <d0,d1> = %.4e\n", d0d1);
  printf("    k_min = %.4e, mu = %.4e\n", k_min, mu_min);
  printf("    ||d(k_min)||_inf = %.4e, ||d(k_min)||^2 = %.4e\n\n",
         d_inf_at_kmin, d_sq_at_kmin);

  // Run centering at k_min.
  printf("  Centering at k=%.4e (mu=%.4e):\n", k_min, mu_min);
  printf("  %3s  %12s  %12s  %12s  %8s\n",
         "it", "d_inf", "d_sq", "s_dot_x", "alpha");
  printf("  %s\n", std::string(52, '-').c_str());

  setOnes(W);  // Reset W to identity.
  auto t2 = std::chrono::high_resolution_clock::now();
  auto result = GeodesicCenter(*kkt, cost_rhs, W, k_min,
                                max_iters, 1e-10, true);
  auto t3 = std::chrono::high_resolution_clock::now();

  printf("\n  Center: %d iters, d_inf=%.2e, %.0f ms\n",
         result.iterations, result.d_inf_norm,
         std::chrono::duration<double, std::milli>(t3 - t2).count());
  printf("  (%.0f ms/iter)\n",
         std::chrono::duration<double, std::milli>(t3 - t2).count() /
             std::max(result.iterations, 1));
}

}  // namespace conex

int main(int argc, char* argv[]) {
  if (argc < 2) {
    printf("Usage: %s <file.dat-s|file.mps|file.cbf> [max_iters]\n", argv[0]);
    return 1;
  }

  std::string filename = argv[1];
  int max_iters = argc > 2 ? std::atoi(argv[2]) : 30;
  std::string ext = filename.substr(filename.find_last_of('.') + 1);

  try {
    if (ext == "mps") {
      auto [problem, info] = conex::ReadMPS(filename);
      char name[256];
      snprintf(name, sizeof(name), "MPS: %s (%d vars)",
               info.name.c_str(), info.num_variables);
      conex::RunCentering(problem, name, max_iters);
    } else if (ext == "dat-s" || ext == "dat" ||
               filename.find(".dat-s") != std::string::npos) {
      auto [problem, info] = conex::ReadSDPA(filename);
      char name[256];
      snprintf(name, sizeof(name), "SDPA: %d vars, %d blocks, dim=%d",
               info.num_constraints, info.num_blocks, info.total_matrix_dim);
      conex::RunCentering(problem, name, max_iters);
    } else if (ext == "cbf") {
      auto [problem, info] = conex::ReadCBF(filename);
      char name[256];
      snprintf(name, sizeof(name), "CBF: %d vars, %d cons",
               info.num_variables, info.num_constraints);
      conex::RunCentering(problem, name, max_iters);
    } else {
      printf("Unknown extension: %s\n", ext.c_str());
      return 1;
    }
  } catch (const std::exception& e) {
    printf("Error: %s\n", e.what());
    return 1;
  }

  return 0;
}
