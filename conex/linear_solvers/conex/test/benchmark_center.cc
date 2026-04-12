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
#include "conex/common/rescale.h"
#include "conex/common/sdpa_reader.h"
#include "conex/common/solver.h"
#include "conex/tree_solver/kkt_tree_solver.h"

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

  // Rebuild problem with b = identity to place I on the central path.
  {
    Problem centered;
    for (int i = 0; i < problem.num_constraints(); ++i) {
      std::visit([&](const auto& data) {
        using T = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
          Eigen::VectorXd b_id = Eigen::VectorXd::Ones(data.A.rows());
          centered.AddLinearConstraint(data.A, b_id, data.vars);
        } else if constexpr (std::is_same_v<T, Problem::PSDConstraintData>) {
          int n = data.B.rows();
          Eigen::SparseMatrix<double> I_n =
              Eigen::MatrixXd::Identity(n, n).sparseView();
          // Disable chordal: identity B is dense and changes the
          // aggregate sparsity, which can break clique tree construction.
          centered.AddPSDConstraint(data.A_list, I_n, data.vars,
                                    /*use_chordal=*/false);
        } else if constexpr (std::is_same_v<T, Problem::SOCConstraintData>) {
          Eigen::VectorXd b_soc = Eigen::VectorXd::Zero(data.A.rows());
          b_soc(0) = 1.0;
          centered.AddSOCConstraint(data.A, b_soc, data.vars);
        }
      }, problem.constraint(i));
    }
    solver = Solver::Build(centered);
  }

  // Print KKT tree stats.
  {
    auto* ts = solver.tree_solver();
    int nc = ts->num_subsystems();
    int max_cs = 0;
    for (int k = 0; k < nc; ++k)
      max_cs = std::max(max_cs, ts->clique_size(k));
    printf("  KKT tree: %d cliques, max clique size %d\n", nc, max_cs);
  }

  auto* kkt = solver.solver();
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // Set cost = A^T I so that (W=I, k=1) → d=0.
  // At W=I, k=1, b=I: v = -P(I)I + 2I = -I + 2I = I.
  // RHS = -c + A^T I = 0 when c = A^T I. And d = I - (I + Ay) = -Ay = 0.
  kkt->SetScaling(W);
  kkt->AssembleAndFactor();
  RowSpace identity = kkt->MakeRowSpace();
  setOnes(identity);
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs.SetZero();
  kkt->AccumulateAtranspose(identity, cost_rhs);

  // Sanity: d at W=I, k=1 should be exactly 0.
  {
    RowSpace b = kkt->GetAffineTerm();
    auto y = kkt->MakeSolverRHS();
    y = cost_rhs;
    y *= -1;
    RowSpace v = addScaled(quadraticRepresentation(W, b), W, -1, 2.0);
    kkt->AccumulateAtranspose(v, y);
    kkt->SolveSolverRHS(y);
    RowSpace row = kkt->MakeRowSpace();
    kkt->MultiplyA(y, row);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace d_check = addScaled(b, row, -1, -1.0);
    d_check = quadraticRepresentation(sqrtW, d_check);
    RowSpace ones = kkt->MakeRowSpace();
    setOnes(ones);
    d_check += ones;
    printf("  Sanity: d_inf at W=I, k=1 = %.2e (should be ~0)\n",
           normInf(d_check));
  }

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

  // Center at mu=0.5 (k = sqrt(2)) starting from W=I (which is centered at k=1).
  double k_target = std::sqrt(2.0);  // mu = 1/k^2 = 0.5
  printf("  Centering at k=%.4f (mu=0.5) from W=I (centered at k=1):\n",
         k_target);
  printf("  %3s  %12s  %12s  %12s  %8s\n",
         "it", "d_inf", "d_sq", "s_dot_x", "alpha");
  printf("  %s\n", std::string(52, '-').c_str());

  setOnes(W);  // Reset W to identity (centered at k=1).
  auto t2 = std::chrono::high_resolution_clock::now();
  auto result = GeodesicCenter(*kkt, cost_rhs, W, k_target,
                                max_iters, 1e-10, true);
  auto t3 = std::chrono::high_resolution_clock::now();

  printf("\n  Center: %d iters, d_inf=%.2e, %.0f ms\n",
         result.iterations, result.d_inf_norm,
         std::chrono::duration<double, std::milli>(t3 - t2).count());
  printf("  (%.0f ms/iter)\n",
         std::chrono::duration<double, std::milli>(t3 - t2).count() /
             std::max(result.iterations, 1));
}

// Run centering on the original problem data (no b→I, no cost→A^T I).
// Starts from W=I and centers at k=1.
void RunCenteringRaw(Problem& problem, const std::string& name, int max_iters) {
  printf("=== %s [raw] ===\n", name.c_str());
  printf("  Variables: %d, Constraints: %d\n",
         problem.num_variables(), problem.num_constraints());

  // Regularize: add eps*I to each B and eps*trace(A_i) to cost.
  // eps = 10% of the nominal norm (per constraint).
  double eps = 0;
  {
    // First build unregularized to measure d0 norm.
    {
      auto solver0 = Solver::Build(problem);
      auto* kkt0 = solver0.solver();
      RowSpace W0 = kkt0->MakeRowSpace();
      setOnes(W0);
      kkt0->SetScaling(W0);
      kkt0->AssembleAndFactor();
      // d at k=0 (zero cost): measures how far W=I is from the analytic center.
      auto y0 = kkt0->MakeSolverRHS();
      y0.SetZero();
      RowSpace v0 = W0; v0 *= 2.0;
      kkt0->AccumulateAtranspose(v0, y0);
      kkt0->SolveSolverRHS(y0);
      RowSpace row0 = kkt0->MakeRowSpace();
      kkt0->MultiplyA(y0, row0);
      RowSpace b0 = kkt0->GetAffineTerm();
      RowSpace sqrtW0 = EuclideanJordanAlgebra::sqrt(W0);
      RowSpace d0 = quadraticRepresentation(sqrtW0, addScaled(b0, row0, 0, -1.0));
      RowSpace ones0 = kkt0->MakeRowSpace(); setOnes(ones0);
      d0 += ones0;
      double d0_inf = normInf(d0);
      // eps = 10% * d0_inf: shift B by enough to bring d_inf near 1.
      eps = 0.1 * std::max(d0_inf, 1.0);
      printf("  d0_inf=%.2e, eps=%.2e\n", d0_inf, eps);
    }

    Problem regularized;
    for (int i = 0; i < problem.num_constraints(); ++i) {
      std::visit([&](const auto& data) {
        using T = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T, Problem::PSDConstraintData>) {
          int n = data.B.rows();
          Eigen::SparseMatrix<double> B_reg = data.B +
              eps * Eigen::MatrixXd::Identity(n, n).sparseView();
          regularized.AddPSDConstraint(data.A_list, B_reg, data.vars,
                                       data.use_chordal);
        } else if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
          Eigen::VectorXd b_reg = data.b.array() + eps;
          regularized.AddLinearConstraint(data.A, b_reg, data.vars);
        } else if constexpr (std::is_same_v<T, Problem::SOCConstraintData>) {
          Eigen::VectorXd b_reg = data.b;
          b_reg(0) += eps;
          regularized.AddSOCConstraint(data.A, b_reg, data.vars);
        }
      }, problem.constraint(i));
    }
    // cost_reg = cost + eps * A^T(I).
    // A^T(I)_j = trace(A_j · I) = trace(A_j) for PSD, sum of A_j column for nonneg.
    int nv = problem.num_variables();
    VectorXd cost_reg = VectorXd::Zero(nv);
    if (problem.has_linear_cost())
      cost_reg = problem.linear_cost();
    for (int i = 0; i < problem.num_constraints(); ++i) {
      std::visit([&](const auto& data) {
        using T = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T, Problem::PSDConstraintData>) {
          for (int k = 0; k < static_cast<int>(data.A_list.size()); ++k) {
            // trace(A_k) = sum of diagonal
            double tr = 0;
            for (int outer = 0; outer < data.A_list[k].outerSize(); ++outer)
              for (Eigen::SparseMatrix<double>::InnerIterator it(
                       data.A_list[k], outer); it; ++it)
                if (it.row() == it.col()) tr += it.value();
            cost_reg(data.vars[k]) += eps * tr;
          }
        } else if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
          for (int k = 0; k < data.A.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
                 it; ++it)
              cost_reg(data.vars[it.col()]) += eps * it.value();
        }
      }, problem.constraint(i));
    }
    regularized.SetLinearCost(cost_reg);
    problem = std::move(regularized);
    printf("  Regularized with eps=%.2e\n", eps);
  }

  auto solver = Solver::Build(problem);

  {
    auto* ts = solver.tree_solver();
    int nc = ts->num_subsystems();
    int max_cs = 0;
    for (int k = 0; k < nc; ++k)
      max_cs = std::max(max_cs, ts->clique_size(k));
    printf("  KKT tree: %d cliques, max clique size %d\n", nc, max_cs);
  }

  auto* kkt = solver.solver();
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);

  // Use the problem's actual linear cost.
  auto cost_rhs = kkt->MakeSolverRHS();
  if (problem.has_linear_cost()) {
    cost_rhs = kkt->MakeBlockVariable(problem.linear_cost());
  } else {
    cost_rhs.SetZero();
  }

  // Compute d at W=I, k=1 to see how far we are from the central path.
  kkt->SetScaling(W);
  kkt->AssembleAndFactor();
  {
    RowSpace b = kkt->GetAffineTerm();
    auto y = kkt->MakeSolverRHS();
    y = cost_rhs;
    y *= -1;
    RowSpace v = addScaled(quadraticRepresentation(W, b), W, -1, 2.0);
    kkt->AccumulateAtranspose(v, y);
    kkt->SolveSolverRHS(y);
    RowSpace row = kkt->MakeRowSpace();
    kkt->MultiplyA(y, row);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace d_check = addScaled(b, row, -1, -1.0);
    d_check = quadraticRepresentation(sqrtW, d_check);
    RowSpace ones = kkt->MakeRowSpace();
    setOnes(ones);
    d_check += ones;
    double d_inf = normInf(d_check);
    double d_sq = squaredNorm(d_check);
    int m = W.total_rows();
    printf("  At W=I, k=1: d_inf=%.2e, d_sq=%.2e, s·x=%.2e\n",
           d_inf, d_sq, (m - d_sq));
  }

  // Decompose d(k) = d0 + k*d1 at W=I to find min-norm k.
  RowSpace d0 = kkt->MakeRowSpace();
  RowSpace d1 = kkt->MakeRowSpace();
  {
    RowSpace b = kkt->GetAffineTerm();
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

    // d at k=0: zero-cost direction.
    auto y0 = kkt->MakeSolverRHS();
    y0.SetZero();
    RowSpace v0 = W; v0 *= 2.0;
    kkt->AccumulateAtranspose(v0, y0);
    kkt->SolveSolverRHS(y0);
    RowSpace row0 = kkt->MakeRowSpace();
    kkt->MultiplyA(y0, row0);
    d0 = addScaled(b, row0, 0, -1.0);
    d0 = quadraticRepresentation(sqrtW, d0);
    RowSpace ones = kkt->MakeRowSpace(); setOnes(ones);
    d0 += ones;

    // d at k=1: full-cost direction.
    kkt->SetScaling(W); kkt->AssembleAndFactor();
    auto y1 = kkt->MakeSolverRHS();
    y1 = cost_rhs; y1 *= -1;
    RowSpace v1 = addScaled(quadraticRepresentation(W, b), W, -1, 2.0);
    kkt->AccumulateAtranspose(v1, y1);
    kkt->SolveSolverRHS(y1);
    RowSpace row1 = kkt->MakeRowSpace();
    kkt->MultiplyA(y1, row1);
    RowSpace d_at_1 = addScaled(b, row1, -1, -1.0);
    d_at_1 = quadraticRepresentation(sqrtW, d_at_1);
    d_at_1 += ones;

    d1 = d_at_1 - d0;
  }

  double d0d1 = dot(d0, d1);
  double d1sq = squaredNorm(d1);
  double d0sq = squaredNorm(d0);
  double k_min = (d1sq > 1e-30) ? std::max(1e-6, -d0d1 / d1sq) : 1.0;
  double mu_min = 1.0 / (k_min * k_min);

  RowSpace d_at_kmin = addScaled(d0, d1, 1.0, k_min);
  printf("\n  Min-norm decomposition:\n");
  printf("    ||d0||_inf = %.4e, ||d0||^2 = %.4e\n", normInf(d0), d0sq);
  printf("    ||d1||_inf = %.4e, ||d1||^2 = %.4e\n", normInf(d1), d1sq);
  printf("    <d0,d1> = %.4e\n", d0d1);
  printf("    k_min = %.4e, mu = %.4e\n", k_min, mu_min);
  printf("    ||d(k_min)||_inf = %.4e\n", normInf(d_at_kmin));

  // Center at k_min.
  printf("\n  Centering at k=%.4e (mu=%.4e) from W=I:\n", k_min, mu_min);
  printf("  %3s  %12s  %12s  %12s  %8s\n",
         "it", "d_inf", "d_sq", "s_dot_x", "alpha");
  printf("  %s\n", std::string(52, '-').c_str());

  setOnes(W);
  auto t0 = std::chrono::high_resolution_clock::now();
  auto result = GeodesicCenter(*kkt, cost_rhs, W, k_min,
                                max_iters, 1e-10, true);
  auto t1 = std::chrono::high_resolution_clock::now();

  printf("\n  Center: %d iters, d_inf=%.2e, %.0f ms\n",
         result.iterations, result.d_inf_norm,
         std::chrono::duration<double, std::milli>(t1 - t0).count());
  printf("  (%.0f ms/iter)\n",
         std::chrono::duration<double, std::milli>(t1 - t0).count() /
             std::max(1, result.iterations));
}

}  // namespace conex

int main(int argc, char* argv[]) {
  if (argc < 2) {
    printf("Usage: %s <file> [max_iters] [--raw] [--ruiz|--l2|--maxabs]\n",
           argv[0]);
    return 1;
  }

  std::string filename = argv[1];
  int max_iters = argc > 2 ? std::atoi(argv[2]) : 30;

  // Parse flags from remaining args.
  conex::ColumnScaling strategy = conex::ColumnScaling::Ruiz;
  bool do_rescale = false;
  bool raw_mode = false;
  for (int a = 3; a < argc; ++a) {
    std::string arg = argv[a];
    if (arg == "--rescale" || arg == "--ruiz") {
      do_rescale = true;
      strategy = conex::ColumnScaling::Ruiz;
    } else if (arg == "--l2") {
      do_rescale = true;
      strategy = conex::ColumnScaling::L2Norm;
    } else if (arg == "--maxabs") {
      do_rescale = true;
      strategy = conex::ColumnScaling::MaxAbsValue;
    } else if (arg == "--raw") {
      raw_mode = true;
    }
  }

  std::string ext = filename.substr(filename.find_last_of('.') + 1);

  try {
    conex::Problem problem;
    std::string name;
    if (ext == "mps") {
      auto [p, info] = conex::ReadMPS(filename);
      problem = std::move(p);
      char buf[256];
      snprintf(buf, sizeof(buf), "MPS: %s (%d vars)",
               info.name.c_str(), info.num_variables);
      name = buf;
    } else if (ext == "dat-s" || ext == "dat" ||
               filename.find(".dat-s") != std::string::npos) {
      auto [p, info] = conex::ReadSDPA(filename);
      problem = std::move(p);
      char buf[256];
      snprintf(buf, sizeof(buf), "SDPA: %d vars, %d blocks, dim=%d",
               info.num_constraints, info.num_blocks, info.total_matrix_dim);
      name = buf;
    } else if (ext == "cbf") {
      auto [p, info] = conex::ReadCBF(filename);
      problem = std::move(p);
      char buf[256];
      snprintf(buf, sizeof(buf), "CBF: %d vars, %d cons",
               info.num_variables, info.num_constraints);
      name = buf;
    } else {
      printf("Unknown extension: %s\n", ext.c_str());
      return 1;
    }

    if (do_rescale) {
      const char* sname[] = {"MaxAbsValue", "L2Norm", "Ruiz"};
      printf("Column scaling: %s\n", sname[static_cast<int>(strategy)]);
      auto [rescaled, rinfo] = conex::RescaleProblem(problem, strategy);
      if (rinfo.was_rescaled) {
        printf("Rescaled problem (col_scale range: [%.2e, %.2e])\n",
               rinfo.col_scale.minCoeff(), rinfo.col_scale.maxCoeff());
        problem = std::move(rescaled);
        name += " [rescaled]";
      } else {
        printf("Rescaling had no effect.\n");
      }
    }

    if (raw_mode) {
      conex::RunCenteringRaw(problem, name, max_iters);
    } else {
      conex::RunCentering(problem, name, max_iters);
    }
  } catch (const std::exception& e) {
    printf("Error: %s\n", e.what());
    return 1;
  }

  return 0;
}
