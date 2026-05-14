#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/geodesic_ipm_helpers.h"
#include "conex/common/eja_ops.h"
#include "conex/common/solve_stats.h"
#include "conex/linear_solvers/kkt_tree_solver.h"
#include <cmath>
#include <cstdio>

namespace conex {

using EuclideanJordanAlgebra::addScaled;
using EuclideanJordanAlgebra::dot;
using EuclideanJordanAlgebra::gap;
using EuclideanJordanAlgebra::minEigenvalue;
using EuclideanJordanAlgebra::minSlack;
using EuclideanJordanAlgebra::normInf;
using EuclideanJordanAlgebra::quadraticRepresentation;
using EuclideanJordanAlgebra::setOnes;
using EuclideanJordanAlgebra::shrinkR;
using EuclideanJordanAlgebra::solveLyapunovForD;
using EuclideanJordanAlgebra::squaredNorm;
using EuclideanJordanAlgebra::square;
using EuclideanJordanAlgebra::squareM;
using EuclideanJordanAlgebra::applyM;
using EuclideanJordanAlgebra::applyMt;
using EuclideanJordanAlgebra::updateAutomorphism;
using EuclideanJordanAlgebra::updateAutomorphismP;
using EuclideanJordanAlgebra::updateM;
using EuclideanJordanAlgebra::sqrt;

GeodesicResult SolveGeodesicBarrierLP(
    CompiledModel& model,
    RowSpace& z,
    int max_outer_iterations,
    int max_frozen_steps,
    double tolerance,
    bool verbose,
    SolveStats* stats) {
  const auto& cost_rhs = model.cost_rhs();
  const RowSpace b = model.GetAffineTerm();
  const int m = z.total_rows();
  const double nu = barrierParameter(z);
  double k = 0.0;

  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;

  // Arena for temporary RowSpaces (zero heap allocation in the loop).
  Arena& arena = model.arena();

  if (verbose) {
    printf("  %3s  %12s  %12s  %12s  %12s  %12s\n",
           "fac", "k", "k_new", "d_inf", "d_sqr", "gap");
    printf("  %s\n", std::string(72, '-').c_str());
  }

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    char* iter_mark = arena.SaveCursor();

    // 0. Sync z to constraint workspaces and factor Gram = A^T H(z) A.
    bool factor_ok;
    { CONEX_TIMER(stats, factor_us);
      model.SetScaling(z);
      factor_ok = model.AssembleAndFactor();
    }
    if (!factor_ok) {
      if (verbose) printf("  TERMINATED: factorization failed at iteration %d\n", outer);
      arena.RestoreCursor(iter_mark); break;
    }
    if (stats) stats->factor_count++;

    // 1. Centering RHS: A^T(-2∇F(z)).
    RowSpace grad = model.AllocRowSpace(arena);
    computeGradient(z, grad);
    grad *= -2.0;
    auto rhs0 = model.AllocSolverRHS();
    rhs0.SetZero();
    model.AccumulateAtranspose(grad, rhs0);

    // 2. Optimality RHS: -(c + A^T H(z) b).
    RowSpace hb = model.AllocRowSpace(arena);
    hessianProduct(z, b, hb);
    auto rhs1 = model.AllocSolverRHS();
    rhs1 = cost_rhs;
    model.AccumulateAtranspose(hb, rhs1);
    rhs1 *= -1;
    // Inject equality RHS if present.
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
    if (ts && !ts->equality_sub_assemblers().empty()) {
      auto d_rhs = ts->EqualityAffineTermRHS();
      rhs1 += d_rhs;
    }

    // 3. Solve both RHS.
    auto y = model.AllocSolverRHS(2);
    y.SetColumn(0, rhs0);
    y.SetColumn(1, rhs1);
    { CONEX_TIMER(stats, solve_us);
      model.SolveSolverRHS(y);
    }
    if (stats) stats->solve_count += 2;

    total_fac += 1;
    total_sol += 2;

    // 4. Recover y0, y1.
    SolverRHS y0 = model.MakeSolverRHS();
    SolverRHS y1 = model.MakeSolverRHS();
    {
      int nb = y.num_blocks();
      for (int bk = 0; bk < nb; ++bk) {
        y0.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(0);
        y1.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(1);
      }
    }

    // 5. Compute targets: target0 = Ay0, target1 = Ay1 + b.
    auto row = model.AllocRowSpace(arena, 2);
    model.MultiplyA(y, row);
    RowSpace target0 = model.AllocRowSpace(arena);
    RowSpace target1 = model.AllocRowSpace(arena);
    target0.col() = row.col(0);
    target1.col() = row.col(1);
    target1 += b;

    // 6. Line search for k.
    double k_new;
    { CONEX_TIMER(stats, cone_us);
      k_new = lineSearchTarget(z, target0, target1);
    }
    double k_prev = k;

    // Minimum-norm fallback (same logic as SolveGeodesicLP).
    if (k_new > k) {
      k = k_new;
    } else if (outer == 0 || k_new == 0) {
      // ||target0 + k*target1 - z_primal||²_H = a + 2fk + pk²
      // Minimizer: k* = -f/p.  Compute via polarization.
      double a_coeff = hessianNormSquared(z, target0);
      RowSpace t01 = model.AllocRowSpace(arena);
      addScaled(t01, target0, target1, 1.0, 1.0);
      double ap = hessianNormSquared(z, t01);
      double p_coeff = hessianNormSquared(z, target1);
      double f_coeff = 0.5 * (ap - a_coeff - p_coeff);
      if (p_coeff > 1e-30) {
        double k_min_norm = std::max(0.0, -f_coeff / p_coeff);
        if (k_min_norm > k) k = k_min_norm;
      }
    }

    // 7. Assemble target_k, compute step size and norms.
    RowSpace target_k = model.AllocRowSpace(arena);
    addScaled(target_k, target0, target1, 1.0, k);
    double d_sq = hessianNormSquared(z, target_k);
    double alpha = stepSize(z, target_k);

    // For reporting: d_inf from alpha.  alpha = min(1, 2/d_inf²)
    // => d_inf = sqrt(2/alpha) if alpha < 1, else d_inf ≈ 0.
    double d_inf = (alpha < 1.0) ? std::sqrt(2.0 / alpha) : 0.0;

    double mu = 1.0 / (k * k);
    double gap = mu * (nu - d_sq);

    if (verbose) {
      printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e\n",
             outer, k_prev, k, d_inf, d_sq, gap);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = outer + 1;
    result.mu = mu;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = gap;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;

    // Recover x = y0/k + y1.
    {
      auto x_rhs_conv = model.AllocSolverRHS();
      x_rhs_conv.SetZero();
      x_rhs_conv.AddScaled(1.0 / k, y0);
      x_rhs_conv += y1;
      int nr = model.number_of_variables();
      result.x.resize(nr);
      { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), nr); x_rhs_conv.supernodes->GatherInto(xm); }
    }

    bool converged = (gap < tolerance && d_inf < 1.01);
    bool last_iter = (outer + 1 == max_outer_iterations);

    if (converged || last_iter) {

      // Lambda recovery: λ = (1/k)(-2∇F(z) - H(z)·target_k).
      // Use heap for lambda since it outlives the arena.
      RowSpace lambda = model.MakeRowSpace();
      computeGradient(z, lambda);
      lambda *= -2.0;
      RowSpace h_target = model.AllocRowSpace(arena);
      hessianProduct(z, target_k, h_target);
      lambda -= h_target;  // safe: per-segment ops handle mixed layouts
      lambda *= (1.0 / k);
      result.lambda = std::move(lambda);

      result.optimality.mu = result.mu;
      if (verbose) {
        printf("  Optimality: mu=%.2e\n", result.mu);
      }
      arena.RestoreCursor(iter_mark);
      break;
    } else {
      // Save z₀ for frozen-J, then step.
      RowSpace z0 = z;
      { CONEX_TIMER(stats, cone_us);
        geodesicStepTarget(z, alpha, target_k);
      }

      // Frozen-Jacobian inner steps: use stale Gram (H(z₀)) with
      // frozen centering = -∇F(z_i) + H(z₀)·z_i.
      // Cost RHS (rhs1) is frozen (depends only on z₀ through H(z₀)·b).
      // d1 from the standard decomposition is reused.
      //
      // Debug: set refactor_inner=true to refactor at z_i and use the
      // standard (non-frozen) path.  This should produce the same
      // trajectory as baseline (max_frozen_steps=0).  Any difference
      // is a bug in the frozen-J logic.
      constexpr bool refactor_inner = false;

      for (int inner = 0; inner < max_frozen_steps; ++inner) {
        char* inner_mark = arena.SaveCursor();

        RowSpace target0_f = model.AllocRowSpace(arena);
        RowSpace target1_f = target1;  // default: frozen

        if (refactor_inner) {
          // Full refactor: makes inner iteration identical to outer.
          model.SetScaling(z);
          model.AssembleAndFactor();
          total_fac++;

          // Recompute centering RHS: A^T(-2∇F(z_i)).
          RowSpace grad_r = model.AllocRowSpace(arena);
          computeGradient(z, grad_r);
          grad_r *= -2.0;
          auto rhs0_r = model.AllocSolverRHS();
          rhs0_r.SetZero();
          model.AccumulateAtranspose(grad_r, rhs0_r);

          // Recompute optimality RHS: -(c + A^T H(z_i) b).
          RowSpace hb_r = model.AllocRowSpace(arena);
          hessianProduct(z, b, hb_r);
          auto rhs1_r = model.AllocSolverRHS();
          rhs1_r = cost_rhs;
          model.AccumulateAtranspose(hb_r, rhs1_r);
          rhs1_r *= -1;
          if (ts && !ts->equality_sub_assemblers().empty()) {
            rhs1_r += ts->EqualityAffineTermRHS();
          }

          // Solve both.
          auto y_r = model.AllocSolverRHS(2);
          y_r.SetColumn(0, rhs0_r);
          y_r.SetColumn(1, rhs1_r);
          model.SolveSolverRHS(y_r);
          total_sol += 2;

          // Recover targets.
          auto row_r = model.AllocRowSpace(arena, 2);
          model.MultiplyA(y_r, row_r);
          target0_f.col() = row_r.col(0);
          target1_f.col() = row_r.col(1);
          target1_f += b;
        } else {
          // Frozen centering: -∇F(z_i) + H(z₀)·z_i_raw
          // getConePoint recovers the raw cone point from stored representation
          // (identity for exp cone, inverse for symmetric cones).
          RowSpace grad_i = model.AllocRowSpace(arena);
          computeGradient(z, grad_i);  // ∇F(z_i)
          RowSpace z_raw_i = model.AllocRowSpace(arena);
          getConePoint(z_raw_i, z);
          RowSpace hz0_zi = model.AllocRowSpace(arena);
          hessianProduct(z0, z_raw_i, hz0_zi);  // H(z₀)·z_i_raw
          RowSpace centering = model.AllocRowSpace(arena);
          addScaled(centering, grad_i, hz0_zi, -1.0, 1.0);

          // Centering-only solve with stale Gram (1 back-solve).
          auto rhs0_f = model.AllocSolverRHS();
          rhs0_f.SetZero();
          model.AccumulateAtranspose(centering, rhs0_f);
          model.SolveSolverRHS(rhs0_f);
          total_sol += 1;

          // Recover target0 from solve.
          model.MultiplyA(rhs0_f, target0_f);
        }

        // Line search for k.
        double k_new_f = lineSearchTarget(z, target0_f, target1_f);
        if (k_new_f > k) k = k_new_f;

        // Assemble target_k, step.
        RowSpace target_k_f = model.AllocRowSpace(arena);
        addScaled(target_k_f, target0_f, target1_f, 1.0, k);
        double alpha_f = stepSize(z, target_k_f);

        if (verbose) {
          double d_sq_f = hessianNormSquared(z, target_k_f);
          double d_inf_f = (alpha_f < 1.0) ? std::sqrt(2.0 / alpha_f) : 0.0;
          double mu_f = 1.0 / (k * k);
          double gap_f = mu_f * (nu - d_sq_f);
          printf("  %3d.%d  %12s  %12.4e  %12.4e  %12.4e  %12.4e  (frozen-J)\n",
                 outer, inner + 1, "", k, d_inf_f, d_sq_f, gap_f);
        }

        geodesicStepTarget(z, alpha_f, target_k_f);
        arena.RestoreCursor(inner_mark);
      }
    }
    arena.RestoreCursor(iter_mark);
  }

  return result;
}

GeodesicResult SolveGeodesicBarrierThetaContinuation(
    CompiledModel& model,
    RowSpace& z,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose,
    SolveStats* stats) {
  const auto& cost_rhs = model.cost_rhs();
  const RowSpace b = model.GetAffineTerm();
  const int m = z.total_rows();
  const double nu = barrierParameter(z);

  // Arena for temporary RowSpaces (zero heap allocation in the loop).
  Arena& arena = model.arena();

  // Starting point z_0 = z (the initial interior point).
  RowSpace z0 = model.AllocRowSpace(arena);
  z0 += z;  // per-segment copy from heap z

  // Precompute ∇F(z_0) (constant throughout).
  RowSpace grad_z0 = model.AllocRowSpace(arena);
  computeGradient(z0, grad_z0);
  const double R_theta1 = -dot(b, grad_z0) + 1.0;

  // Duality cost (with +d at equality dual positions).
  auto duality_cost = MakeDualityCost(model);

  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;
  double k = 0, tau = 0, theta = 1.0;

  if (verbose) {
    printf("  %3s  %8s  %10s  %12s  %12s  %12s  %12s\n",
           "out", "theta", "tau", "k", "d_sq", "gap", "mu");
    printf("  %s\n", std::string(85, '-').c_str());
  }

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    char* iter_mark = arena.SaveCursor();

    // 0. Factor Gram = A^T H(z) A.
    bool factor_ok;
    { CONEX_TIMER(stats, factor_us);
      model.SetScaling(z);
      factor_ok = model.AssembleAndFactor();
    }
    if (!factor_ok) {
      if (verbose) printf("  TERMINATED: factorization failed at iteration %d\n", outer);
      arena.RestoreCursor(iter_mark); break;
    }
    if (stats) stats->factor_count++;

    // 1. Build 3 RHS vectors.
    RowSpace grad = model.AllocRowSpace(arena);
    computeGradient(z, grad);

    // rhs0: centering = -2 A^T ∇F(z).
    auto rhs0 = model.AllocSolverRHS();
    rhs0.SetZero();
    RowSpace neg2grad = model.AllocRowSpace(arena);
    neg2grad += grad;  // per-segment copy
    neg2grad *= -2.0;
    model.AccumulateAtranspose(neg2grad, rhs0);

    // rhs1: optimality (θ=0) = -(c + A^T H(z)b) + d_eq.
    RowSpace hb = model.AllocRowSpace(arena);
    hessianProduct(z, b, hb);
    auto rhs1 = model.AllocSolverRHS();
    rhs1 = cost_rhs;
    model.AccumulateAtranspose(hb, rhs1);
    rhs1 *= -1;
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
    if (ts && !ts->equality_sub_assemblers().empty()) {
      auto d_rhs = ts->EqualityAffineTermRHS();
      rhs1 += d_rhs;
    }

    // rhs2: θ=1 correction = A^T(∇F(z_0) - H(z)z_0) - d_eq.
    RowSpace hz0 = model.AllocRowSpace(arena);
    hessianProduct(z, z0, hz0);
    RowSpace theta1_vec = model.AllocRowSpace(arena);
    addScaled(theta1_vec, grad_z0, hz0, 1.0, -1.0);  // ∇F(z_0) - H(z)z_0
    auto rhs2 = model.AllocSolverRHS();
    rhs2 = rhs1;
    rhs2 *= -1;
    model.AccumulateAtranspose(theta1_vec, rhs2);

    // 2. Solve all 3.
    auto y = model.AllocSolverRHS(3);
    y.SetColumn(0, rhs0);
    y.SetColumn(1, rhs1);
    y.SetColumn(2, rhs2);
    { CONEX_TIMER(stats, solve_us);
      model.SolveSolverRHS(y);
    }
    if (stats) stats->solve_count += 3;
    total_fac++;
    total_sol += 3;

    SolverRHS y0_vec = model.MakeSolverRHS();
    SolverRHS y1_0_vec = model.MakeSolverRHS();
    SolverRHS y1_theta_vec = model.MakeSolverRHS();
    {
      int nb = y.num_blocks();
      for (int bk = 0; bk < nb; ++bk) {
        y0_vec.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(0);
        y1_0_vec.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(1);
        y1_theta_vec.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(2);
      }
    }

    // 3. Compute A*y for all 3.
    auto row = model.AllocRowSpace(arena, 3);
    model.MultiplyA(y, row);
    RowSpace ay0 = model.AllocRowSpace(arena);
    RowSpace ay1_0 = model.AllocRowSpace(arena);
    RowSpace ay1_theta = model.AllocRowSpace(arena);
    ay0.col() = row.col(0);
    ay1_0.col() = row.col(1);
    ay1_theta.col() = row.col(2);

    // 4. Precompute target building blocks.
    RowSpace t1_tau = model.AllocRowSpace(arena);
    addScaled(t1_tau, ay1_0, b, 1.0, 1.0);     // Ay1_0 + b
    RowSpace z0_minus_b = model.AllocRowSpace(arena);
    addScaled(z0_minus_b, z0, b, 1.0, -1.0);
    RowSpace t1_th = model.AllocRowSpace(arena);
    addScaled(t1_th, ay1_theta, z0_minus_b, 1.0, 1.0);  // Ay1_theta + z_0 - b

    // 5. Binary search for θ: find smallest θ with V(τ)=0 feasible.
    constexpr double beta_target = 1.0;
    double theta_prev = theta;
    {
      double theta_lo = 0.0;
      double theta_hi = theta;
      for (int bisect = 0; bisect < 30; ++bisect) {
        double theta_mid = 0.5 * (theta_lo + theta_hi);
        auto [tau_try, d_inf_try] = EvalBarrierThetaCandidate(
            model, arena, duality_cost, z, z, b, grad, ay0, t1_tau, t1_th,
            y0_vec, y1_0_vec, y1_theta_vec, nu, R_theta1, theta_mid);
        if (tau_try > 0 && d_inf_try <= beta_target) {
          theta_hi = theta_mid;
        } else {
          theta_lo = theta_mid;
        }
      }
      theta = theta_hi;
    }
    k = 1.0 / std::sqrt(theta);


    // Evaluate at chosen θ.
    auto [tau_sel, d_inf_sel] = EvalBarrierThetaCandidate(
        model, arena, duality_cost, z, z, b, grad, ay0, t1_tau, t1_th,
        y0_vec, y1_0_vec, y1_theta_vec, nu, R_theta1, theta);
    if (tau_sel <= 0) {
      if (verbose) printf("  TERMINATED: tau <= 0 at iteration %d (theta=%.2e)\n", outer, theta);
      result.iterations = outer + 1;
      arena.RestoreCursor(iter_mark);
      break;
    }
    tau = tau_sel;

    // Final target and metrics.
    RowSpace t1_combined = model.AllocRowSpace(arena);
    addScaled(t1_combined, t1_tau, t1_th, tau, theta);
    RowSpace target_k = model.AllocRowSpace(arena);
    addScaled(target_k, ay0, t1_combined, 1.0, k);
    double d_sq = hessianNormSquared(z, target_k);
    double alpha = stepSize(z, target_k);
    double d_inf = (alpha < 1.0) ? std::sqrt(2.0 / alpha) : 0.0;
    double mu = 1.0 / (k * k);
    double gap = mu * (nu - d_sq);

    if (verbose) {
      printf("  %3d  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e\n",
             outer, theta, tau, k, d_sq, gap, mu);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = outer + 1;
    result.mu = mu;
    result.tau = tau;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = gap;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;

    // Recover x = (y0/k + tau*y1_0 + theta*y1_theta) / tau.
    {
      auto x_rhs = model.AllocSolverRHS();
      x_rhs.SetZero();
      x_rhs.AddScaled(1.0 / k, y0_vec);
      x_rhs.AddScaled(tau, y1_0_vec);
      x_rhs.AddScaled(theta, y1_theta_vec);
      x_rhs *= (1.0 / tau);
      int nr = model.number_of_variables();
      result.x.resize(nr);
      { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), nr); x_rhs.supernodes->GatherInto(xm); }
    }

    bool converged = (mu < tolerance);
    bool last_iter = (outer + 1 == max_outer_iterations);

    if (converged || last_iter) {
      // Lambda recovery (heap — outlives arena).
      RowSpace lambda = model.MakeRowSpace();
      computeGradient(z, lambda);
      lambda *= -2.0;
      RowSpace h_target = model.AllocRowSpace(arena);
      hessianProduct(z, target_k, h_target);
      lambda -= h_target;
      lambda *= (1.0 / (k * tau));
      result.lambda = std::move(lambda);

      result.optimality.mu = result.mu;
      if (verbose) {
        printf("  Optimality: mu=%.2e\n", result.mu);
      }
      arena.RestoreCursor(iter_mark);
      if (converged) break;
    } else {
      // Geodesic step (always take — d_inf may be ≤ 1 by design in ThetaCont).
      RowSpace z_outer = z;  // heap deep copy (z is heap-backed)
      { CONEX_TIMER(stats, cone_us);
        geodesicStepTarget(z, alpha, target_k);
      }

      // Frozen-Jacobian inner steps: use stale Gram (H(z_outer)) with
      // frozen centering = -∇F(z_i) + H(z_outer)·z_i.
      // rhs1 (optimality) and rhs2 (theta correction) are frozen.
      // y1_0 and y1_theta are frozen. Only y0 (centering) is refreshed.
      //
      // Debug: set refactor_inner=true to refactor at z_i and use the
      // standard (non-frozen) path.  This should produce the same
      // trajectory as baseline (max_centering_steps=0).
      constexpr bool refactor_inner = false;

      for (int inner = 0; inner < max_centering_steps; ++inner) {
        char* inner_mark = arena.SaveCursor();

        RowSpace ay0_f = model.AllocRowSpace(arena);
        SolverRHS y0_f{}, y1_0_f = y1_0_vec, y1_theta_f = y1_theta_vec;
        RowSpace z_hess_f = z_outer;
        RowSpace grad_f = model.AllocRowSpace(arena);

        if (refactor_inner) {
          // Allocate fresh t1 buffers (refactor overwrites them).
          RowSpace t1_tau_f = model.AllocRowSpace(arena);
          RowSpace t1_th_f = model.AllocRowSpace(arena);
          // Full refactor: makes inner iteration identical to outer.
          z_outer = z;
          z_hess_f = z;
          model.SetScaling(z);
          model.AssembleAndFactor();
          total_fac++;

          // Recompute all 3 RHS.
          computeGradient(z, grad_f);

          auto rhs0_r = model.AllocSolverRHS();
          rhs0_r.SetZero();
          RowSpace neg2grad_r = model.AllocRowSpace(arena);
          neg2grad_r += grad_f;
          neg2grad_r *= -2.0;
          model.AccumulateAtranspose(neg2grad_r, rhs0_r);

          RowSpace hb_r = model.AllocRowSpace(arena);
          hessianProduct(z, b, hb_r);
          auto rhs1_r = model.AllocSolverRHS();
          rhs1_r = cost_rhs;
          model.AccumulateAtranspose(hb_r, rhs1_r);
          rhs1_r *= -1;
          if (ts && !ts->equality_sub_assemblers().empty()) {
            rhs1_r += ts->EqualityAffineTermRHS();
          }

          RowSpace hz0_r = model.AllocRowSpace(arena);
          hessianProduct(z, z0, hz0_r);
          RowSpace theta1_vec_r = model.AllocRowSpace(arena);
          addScaled(theta1_vec_r, grad_z0, hz0_r, 1.0, -1.0);
          auto rhs2_r = model.AllocSolverRHS();
          rhs2_r = rhs1_r;
          rhs2_r *= -1;
          model.AccumulateAtranspose(theta1_vec_r, rhs2_r);

          auto y_r = model.AllocSolverRHS(3);
          y_r.SetColumn(0, rhs0_r);
          y_r.SetColumn(1, rhs1_r);
          y_r.SetColumn(2, rhs2_r);
          model.SolveSolverRHS(y_r);
          total_sol += 3;

          y0_f = model.AllocSolverRHS();
          y1_0_f = model.AllocSolverRHS();
          y1_theta_f = model.AllocSolverRHS();
          {
            int nb_r = y_r.num_blocks();
            for (int bk = 0; bk < nb_r; ++bk) {
              y0_f.supernodes->block(bk).col(0) = y_r.supernodes->block(bk).col(0);
              y1_0_f.supernodes->block(bk).col(0) = y_r.supernodes->block(bk).col(1);
              y1_theta_f.supernodes->block(bk).col(0) = y_r.supernodes->block(bk).col(2);
            }
          }

          auto row_r = model.AllocRowSpace(arena, 3);
          model.MultiplyA(y_r, row_r);
          ay0_f.col() = row_r.col(0);
          RowSpace ay1_0_r = model.AllocRowSpace(arena);
          RowSpace ay1_theta_r = model.AllocRowSpace(arena);
          ay1_0_r.col() = row_r.col(1);
          ay1_theta_r.col() = row_r.col(2);
          addScaled(t1_tau_f, ay1_0_r, b, 1.0, 1.0);
          RowSpace z0_minus_b_r = model.AllocRowSpace(arena);
          addScaled(z0_minus_b_r, z0, b, 1.0, -1.0);
          addScaled(t1_th_f, ay1_theta_r, z0_minus_b_r, 1.0, 1.0);

          // Full theta binary search (same as outer).
          double theta_lo_r = 0.0;
          double theta_hi_r = theta;
          for (int bisect = 0; bisect < 30; ++bisect) {
            double theta_mid = 0.5 * (theta_lo_r + theta_hi_r);
            auto [tau_try, d_inf_try] = EvalBarrierThetaCandidate(
                model, arena, duality_cost, z, z, b, grad_f,
                ay0_f, t1_tau_f, t1_th_f,
                y0_f, y1_0_f, y1_theta_f,
                nu, R_theta1, theta_mid);
            if (tau_try > 0 && d_inf_try <= beta_target) {
              theta_hi_r = theta_mid;
            } else {
              theta_lo_r = theta_mid;
            }
          }
          theta = theta_hi_r;
          k = 1.0 / std::sqrt(theta);

          auto [tau_r, d_inf_r] = EvalBarrierThetaCandidate(
              model, arena, duality_cost, z, z, b, grad_f,
              ay0_f, t1_tau_f, t1_th_f,
              y0_f, y1_0_f, y1_theta_f,
              nu, R_theta1, theta);
          if (tau_r <= 0) { arena.RestoreCursor(inner_mark); break; }
          tau = tau_r;

          RowSpace t1_comb_r = model.AllocRowSpace(arena);
          addScaled(t1_comb_r, t1_tau_f, t1_th_f, tau, theta);
          RowSpace target_k_r = model.AllocRowSpace(arena);
          addScaled(target_k_r, ay0_f, t1_comb_r, 1.0, k);
          double alpha_r = stepSize(z, target_k_r);

          if (verbose) {
            double d_sq_r = hessianNormSquared(z, target_k_r);
            double mu_r = theta;
            double gap_r = mu_r * (nu - d_sq_r);
            printf("  %3d.%d  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  (frozen-J)\n",
                   outer, inner + 1, theta, tau, k, d_sq_r, gap_r, mu_r);
          }
          geodesicStepTarget(z, alpha_r, target_k_r);
        } else {
          // Frozen centering: -∇F(z_i) + H(z_outer)·z_i.
          // Keep theta fixed, refresh centering, line search for k.
          computeGradient(z, grad_f);
          RowSpace z_raw_i = model.AllocRowSpace(arena);
          getConePoint(z_raw_i, z);
          RowSpace hz0_zi = model.AllocRowSpace(arena);
          hessianProduct(z_outer, z_raw_i, hz0_zi);
          RowSpace centering_f = model.AllocRowSpace(arena);
          addScaled(centering_f, grad_f, hz0_zi, -1.0, 1.0);

          auto rhs0_f = model.AllocSolverRHS();
          rhs0_f.SetZero();
          model.AccumulateAtranspose(centering_f, rhs0_f);
          model.SolveSolverRHS(rhs0_f);
          total_sol += 1;

          // Recover target0.
          model.MultiplyA(rhs0_f, ay0_f);

          // Frozen inner step: treat tau·t1_tau + theta·t1_th as
          // a single frozen "d1" direction. Line search for k using
          // the refreshed centering (ay0_f) and frozen d1.
          RowSpace d1_frozen = model.AllocRowSpace(arena);
          addScaled(d1_frozen, t1_tau, t1_th, tau, theta);
          double k_new_f = lineSearchTarget(z, ay0_f, d1_frozen);
          if (k_new_f > k) {
            k = k_new_f;
            theta = 1.0 / (k * k);
          }
          RowSpace target_k_f = model.AllocRowSpace(arena);
          addScaled(target_k_f, ay0_f, d1_frozen, 1.0, k);

          double alpha_f = stepSize(z, target_k_f);

          if (verbose) {
            double d_sq_f = hessianNormSquared(z, target_k_f);
            double mu_f = theta;
            double gap_f = mu_f * (nu - d_sq_f);
            printf("  %3d.%d  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  (frozen-J)\n",
                   outer, inner + 1, theta, tau, k, d_sq_f, gap_f, mu_f);
          }

          geodesicStepTarget(z, alpha_f, target_k_f);
        }
        arena.RestoreCursor(inner_mark);
      }
    }
    arena.RestoreCursor(iter_mark);
  }

  if (result.x.size() == 0) {
    result.x.assign(model.number_of_variables(), 0.0);
    result.mu = (k > 0) ? 1.0 / (k * k) : 1.0;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;
  }

  return result;
}

}  // namespace conex
