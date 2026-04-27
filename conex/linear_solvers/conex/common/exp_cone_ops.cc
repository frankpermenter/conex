// Exponential cone operations.
// K = cl{(x, y, z) : y*exp(x/y) <= z, y > 0}
// Barrier: F = -log(z - y*exp(x/y)) - log(y)
//
// Key quantities:
//   e = exp(x/y) = exp(s), where s = x/y
//   u = z - y*e  (slack, > 0 in interior)

#include "conex/common/exp_cone_ops.h"

#include <cmath>
#include <algorithm>

namespace conex {
namespace EuclideanJordanAlgebra {

double ExpConeOps::Barrier(double x, double y, double z) {
  double e = std::exp(x / y);
  double u = z - y * e;
  return -std::log(u) - std::log(y);
}

void ExpConeOps::BarrierGrad(double x, double y, double z, double* g) {
  double s = x / y;
  double e = std::exp(s);
  double u = z - y * e;
  g[0] = e / (-u);            // dF/dx = e/(-u) = -e/u... wait
  // F = -log(u) - log(y), u = z - y*e
  // dF/dx = -(1/u)*du/dx = -(1/u)*(-y*e*(1/y)) = e/u
  g[0] = e / u;               // but u < 0 if we define u = z - ye...
  // Actually u = z - y*e > 0 in the interior (y*e < z).
  // dF/dx = -(1/u)*(du/dx) = -(1/u)*(-e) = e/u
  // Wait: du/dx = d/dx(z - y*exp(x/y)) = -y * exp(x/y) * (1/y) = -exp(x/y) = -e
  // So dF/dx = -(1/u)*(-e) = e/u
  g[0] = e / u;
  // dF/dy = -(1/u)*du/dy - 1/y
  // du/dy = -e + y*e*(x/y^2) = -e + e*s/1 ... wait
  // du/dy = d/dy(z - y*e^{x/y}) = -e^{x/y} - y*e^{x/y}*(-x/y^2) = -e + e*x/y = e*(s - 1)
  // So dF/dy = -(1/u)*e*(s-1) - 1/y = -e*(s-1)/u - 1/y
  g[1] = -e * (s - 1) / u - 1.0 / y;
  // dF/dz = -(1/u)*1 = -1/u
  g[2] = -1.0 / u;
}

void ExpConeOps::BarrierHessian(double x, double y, double z, double* H) {
  // H is stored as 9 doubles in row-major: H[3*i + j].
  double s = x / y;
  double e = std::exp(s);
  double u = z - y * e;
  double u2 = u * u;
  double y2 = y * y;
  double ey = e * y;

  // H[0,0] = e*(e*y + u)/(u^2*y)
  H[0] = e * (ey + u) / (u2 * y);
  // H[0,1] = -e*(e*y*(s-1) + s*u)/(u^2*y)
  H[1] = -e * (ey * (s - 1) + s * u) / (u2 * y);
  // H[0,2] = -e/u^2
  H[2] = -e / u2;
  // Symmetric
  H[3] = H[1];
  // H[1,1] = e^2*(s-1)^2/u^2 + e*s^2/(u*y) + 1/y^2
  double sm1 = s - 1;
  H[4] = e * e * sm1 * sm1 / u2 + e * s * s / (u * y) + 1.0 / y2;
  // H[1,2] = e*(s-1)/u^2
  H[5] = e * sm1 / u2;
  // Symmetric
  H[6] = H[2];
  H[7] = H[5];
  // H[2,2] = 1/u^2
  H[8] = 1.0 / u2;
}

void ExpConeOps::ThirdDerivContract(double x, double y, double z,
                                     const double* v, double* T) {
  // Compute T_l = sum_{i,j} F_{ijl} * v_i * v_j for l = 0, 1, 2.
  // This is v^T * (dH/dx_l) * v.
  double s = x / y;
  double e = std::exp(s);
  double u = z - y * e;
  double u2 = u * u, u3 = u2 * u;
  double y2 = y * y, y3 = y2 * y;
  double sm1 = s - 1;
  double ey = e * y;

  // Third derivatives F_{ijl} — 10 unique values (symmetric in all indices).
  // Using notation: Fijk where i<=j<=k.
  double F000 = 2*e*e*e/u3 + 3*e*e/(u2*y) + e/(u*y2);
  double F001 = -e*(2*e*e*y2*sm1 + e*u*y*(3*s-1) + u2*(s+1)) / (u3*y2);
  double F002 = -e*(2*ey + u) / (u3*y);
  double F011 = e*(2*e*s*u*y*sm1 + e*y*(2*ey*sm1*sm1 + s*s*u)
                   + s*u2*(s+2)) / (u3*y2);
  double F012 = e*(2*ey*sm1 + s*u) / (u3*y);
  double F022 = 2*e / u3;
  double F111 = (-2*e*e*e*y3*sm1*sm1*sm1 + 3*e*e*s*s*u*y2*(1-s)
                 - e*s*s*u2*y*(s+3) - 2*u3) / (u3*y3);
  double F112 = -e*(2*ey*sm1*sm1 + s*s*u) / (u3*y);
  double F122 = 2*e*(1-s) / u3;
  double F222 = -2.0 / u3;

  // T_l = sum_{ij} F_{ijl} v_i v_j.  Since F is symmetric, use:
  // T_l = F_{00l}*v0^2 + F_{11l}*v1^2 + F_{22l}*v2^2
  //      + 2*F_{01l}*v0*v1 + 2*F_{02l}*v0*v2 + 2*F_{12l}*v1*v2
  double v0 = v[0], v1 = v[1], v2 = v[2];
  double v00 = v0*v0, v11 = v1*v1, v22 = v2*v2;
  double v01 = v0*v1, v02 = v0*v2, v12 = v1*v2;

  T[0] = F000*v00 + F011*v11 + F022*v22
       + 2*(F001*v01 + F002*v02 + F012*v12);
  T[1] = F001*v00 + F111*v11 + F122*v22
       + 2*(F011*v01 + F012*v02 + F112*v12);
  T[2] = F002*v00 + F112*v11 + F222*v22
       + 2*(F012*v01 + F022*v02 + F122*v12);
}

// Forward declaration.
static void Solve3x3(const double* A, const double* b, double* dx);

// Geodesic acceleration: a = -(1/2) H⁻¹ T(v,v).
static void GeodesicAccel(const double* pos, const double* vel,
                           const double* H, double* a) {
  double T[3];
  ExpConeOps::ThirdDerivContract(pos[0], pos[1], pos[2], vel, T);
  double neg_half_T[3] = {-0.5*T[0], -0.5*T[1], -0.5*T[2]};
  Solve3x3(H, neg_half_T, a);
}

// H(s)·s (= λ by log-homogeneity).
static void Hess_times_s(const double* H, const double* s, double* Hs) {
  for (int i = 0; i < 3; ++i)
    Hs[i] = H[3*i]*s[0] + H[3*i+1]*s[1] + H[3*i+2]*s[2];
}

void ExpConeOps::geodesicStep(double* w, double alpha,
                               const double* d) const {
  // Primal-dual Verlet with geodesic mean reconciliation.
  //
  // Integrate BOTH the primal geodesic (s̈ = -½H⁻¹T) and the dual
  // geodesic (λ̈ = -½T) simultaneously. The dual uses the same T —
  // no extra derivative evaluations.
  //
  // After integration, reconcile via geodesic mean:
  //   λ_primal = H(s_new)·s_new    (from log-homogeneity identity)
  //   λ_dual   = independently integrated dual
  //   λ̄ = (λ_primal + λ_dual) / 2  (Euclidean approx to geodesic mean)
  //   s̄ = (-∇F)⁻¹(λ̄)              (recover primal from averaged dual)
  //
  // This generalizes the Padé approximant: the primal and dual errors
  // have opposite structure, so averaging cancels leading-order terms.

  double pos[3] = {w[0], w[1], w[2]};
  double vel[3] = {alpha*d[0], alpha*d[1], alpha*d[2]};

  // Initialize dual: λ₀ = H(s₀)·s₀, λ̇₀ = -H(s₀)·ṡ₀.
  double H[9];
  BarrierHessian(pos[0], pos[1], pos[2], H);
  double lam[3], lam_dot[3];
  Hess_times_s(H, pos, lam);
  for (int i = 0; i < 3; ++i)
    lam_dot[i] = -(H[3*i]*vel[0] + H[3*i+1]*vel[1] + H[3*i+2]*vel[2]);

  const int steps = 8;
  double dt = 1.0 / steps;

  for (int step = 0; step < steps; ++step) {
    // Primal Verlet: half-step vel, full-step pos, half-step vel.
    double a[3];
    BarrierHessian(pos[0], pos[1], pos[2], H);
    GeodesicAccel(pos, vel, H, a);
    // Primal half-step velocity.
    for (int k = 0; k < 3; ++k) vel[k] += 0.5 * dt * a[k];

    // Dual half-step: λ̈ = -(1/2)T = H·s̈ = H·a (reuse primal accel).
    // But λ̈ = -(1/2)T, and a = -(1/2)H⁻¹T, so H·a = -(1/2)T = λ̈.
    double lam_accel[3];
    for (int i = 0; i < 3; ++i)
      lam_accel[i] = H[3*i]*a[0] + H[3*i+1]*a[1] + H[3*i+2]*a[2];
    for (int k = 0; k < 3; ++k) lam_dot[k] += 0.5 * dt * lam_accel[k];

    // Full-step position (primal and dual).
    for (int k = 0; k < 3; ++k) pos[k] += dt * vel[k];
    for (int k = 0; k < 3; ++k) lam[k] += dt * lam_dot[k];

    // Second half-step.
    BarrierHessian(pos[0], pos[1], pos[2], H);
    GeodesicAccel(pos, vel, H, a);
    for (int k = 0; k < 3; ++k) vel[k] += 0.5 * dt * a[k];
    for (int i = 0; i < 3; ++i)
      lam_accel[i] = H[3*i]*a[0] + H[3*i+1]*a[1] + H[3*i+2]*a[2];
    for (int k = 0; k < 3; ++k) lam_dot[k] += 0.5 * dt * lam_accel[k];
  }

  // Reconcile: average primal-derived and dual-integrated λ.
  double lam_primal[3];
  Hess_times_s(H, pos, lam_primal);  // λ = H(s_new)·s_new

  double lam_avg[3];
  for (int k = 0; k < 3; ++k)
    lam_avg[k] = 0.5 * (lam[k] + lam_primal[k]);

  // Recover s̄ from λ̄: solve -∇F(s̄) = λ̄.
  double s_avg[3] = {pos[0], pos[1], pos[2]};  // start from primal estimate
  if (InvertGradient(lam_avg, s_avg)) {
    w[0] = s_avg[0]; w[1] = s_avg[1]; w[2] = s_avg[2];
  } else {
    // Fallback: use primal Verlet result.
    w[0] = pos[0]; w[1] = pos[1]; w[2] = pos[2];
  }
}

double ExpConeOps::geodesicStepWithErrorEstimate(
    double* w, double alpha, const double* d) const {
  // Same as geodesicStep but returns the consistency error
  // ||λ_integrated - H(s)·s|| as a free error estimate.
  double pos[3] = {w[0], w[1], w[2]};
  double vel[3] = {alpha*d[0], alpha*d[1], alpha*d[2]};

  const int steps = 8;
  double dt = 1.0 / steps;

  // λ_integrated = H(s₀)·s₀.
  double H[9];
  BarrierHessian(pos[0], pos[1], pos[2], H);
  double lam[3];
  Hess_times_s(H, pos, lam);

  double max_err = 0;

  for (int step = 0; step < steps; ++step) {
    double a[3];
    BarrierHessian(pos[0], pos[1], pos[2], H);
    GeodesicAccel(pos, vel, H, a);
    for (int k = 0; k < 3; ++k) vel[k] += 0.5 * dt * a[k];
    for (int k = 0; k < 3; ++k) pos[k] += dt * vel[k];
    BarrierHessian(pos[0], pos[1], pos[2], H);
    GeodesicAccel(pos, vel, H, a);
    for (int k = 0; k < 3; ++k) vel[k] += 0.5 * dt * a[k];

    // Accumulate λ: λ += dt·(-H·ṡ).
    for (int i = 0; i < 3; ++i)
      lam[i] -= dt * (H[3*i]*vel[0] + H[3*i+1]*vel[1] + H[3*i+2]*vel[2]);

    // Consistency: λ_exact = H(s)·s.
    double lam_exact[3];
    Hess_times_s(H, pos, lam_exact);
    double err = 0;
    for (int i = 0; i < 3; ++i)
      err += (lam[i]-lam_exact[i]) * (lam[i]-lam_exact[i]);
    max_err = std::max(max_err, std::sqrt(err));
  }

  w[0] = pos[0]; w[1] = pos[1]; w[2] = pos[2];
  return max_err;
}

void ExpConeOps::geodesicUpdate(double* out, const double* a, double alpha,
                                 const double* d, int /*size*/) const {
  out[0] = a[0]; out[1] = a[1]; out[2] = a[2];
  geodesicStep(out, alpha, d);
}

// Solve A*dx = b for 3x3 A.
static void Solve3x3(const double* A, const double* b, double* dx) {
  double det = A[0]*(A[4]*A[8]-A[5]*A[7])
             - A[1]*(A[3]*A[8]-A[5]*A[6])
             + A[2]*(A[3]*A[7]-A[4]*A[6]);
  double inv[9];
  inv[0] = (A[4]*A[8]-A[5]*A[7])/det;
  inv[1] = (A[2]*A[7]-A[1]*A[8])/det;
  inv[2] = (A[1]*A[5]-A[2]*A[4])/det;
  inv[3] = (A[5]*A[6]-A[3]*A[8])/det;
  inv[4] = (A[0]*A[8]-A[2]*A[6])/det;
  inv[5] = (A[2]*A[3]-A[0]*A[5])/det;
  inv[6] = (A[3]*A[7]-A[4]*A[6])/det;
  inv[7] = (A[1]*A[6]-A[0]*A[7])/det;
  inv[8] = (A[0]*A[4]-A[1]*A[3])/det;
  for (int i = 0; i < 3; ++i)
    dx[i] = inv[3*i]*b[0] + inv[3*i+1]*b[1] + inv[3*i+2]*b[2];
}

void ExpConeOps::leapfrogStep(double* w, double alpha,
                               const double* d) const {
  // (s, λ) leapfrog geodesic integrator.
  //
  // Log-homogeneity identities along geodesic γ(t):
  //   λ = H(s)s,  λ̇ = -H(s)ṡ,  λ̈ = H(s)s̈.
  //
  // State: (s, λ) where s ∈ int(K), λ = -∇F(s) ∈ int(K*).
  // Initial velocity: ṡ = d·α, λ̇ = -H(s)·ṡ.
  //
  // Leapfrog (Störmer-Verlet in (s, λ)):
  //   1. Half-step s:  s_{1/2} = s + (dt/2) ṡ
  //   2. Full-step λ:  λ_{n+1} = λ + dt λ̇  where λ̇ = -H(s_{1/2}) ṡ_{1/2}
  //      But ṡ_{1/2} is unknown. Use the midpoint Hessian:
  //      λ̇ at midpoint = -H(s_{1/2}) · ṡ_n  (first-order approx).
  //      Then ṡ_{n+1} = -H(s_{1/2})⁻¹ · λ̇_{n+1}...
  //
  // Cleaner formulation: track (s, ṡ) but use H to update ṡ without Christoffel:
  //   The geodesic equation ṡ'' = -(1/2)H⁻¹ T(ṡ,ṡ) can be rewritten using
  //   λ̇ = -Hṡ as: d/dt(Hṡ) = -Ḣṡ - Hs̈ = -Tṡ - Hs̈. And since λ̈ = Hs̈ on
  //   geodesic: d/dt(Hṡ) = -Tṡ - λ̈ = -Tṡ - Hs̈. This still needs T.
  //
  // The ACTUAL trick: use the (s, p) formulation where p = H(s)ṡ (momentum).
  // Then λ̇ = -p. The Hamiltonian is E = (1/2) p^T H⁻¹ p.
  // Hamilton's equations: ṡ = H⁻¹ p,  ṗ = -(1/2) ∂/∂s [p^T H⁻¹ p].
  // The ṗ equation needs ∂H⁻¹/∂s which involves third derivatives again!
  //
  // Instead: use the IMPLICIT midpoint rule on (s, λ = -∇F(s)).
  // Given s_n with λ_n = -∇F(s_n):
  //   s_{n+1} = s_n + dt · ṡ  where ṡ = -H(s_mid)⁻¹ · (λ_{n+1} - λ_n)/dt
  //   λ_{n+1} = -∇F(s_{n+1})
  //
  // This is equivalent to: find s_{n+1} such that
  //   -∇F(s_{n+1}) = -∇F(s_n) + dt · (-H(s_mid) · (s_{n+1} - s_n)/dt)
  //   = λ_n - H(s_mid)(s_{n+1} - s_n)
  //
  // Simplified leapfrog exploiting λ̇ = -Hṡ directly:
  // Track (s, v) where v = ṡ. Use the identity that H maps s̈ to λ̈:
  //   Step 1: s_{1/2} = s + (dt/2) v           (primal half-step)
  //   Step 2: Compute H at s_{1/2}
  //   Step 3: λ_mid = -∇F(s_{1/2})             (dual at midpoint)
  //   Step 4: λ_new = 2λ_mid - λ_old           (dual extrapolation = Bregman!)
  //   Step 5: s_new such that -∇F(s_new) = λ_new  (invert gradient)
  //   Step 6: v_new from s_new - s_{1/2}
  //
  // This IS the Bregman midpoint. The (s,λ) leapfrog and Bregman midpoint
  // are the same method viewed differently.
  //
  // The TRUE Christoffel-free leapfrog uses MULTIPLE substeps of the
  // Bregman midpoint (small dt per substep) to get higher accuracy.
  // With N substeps of dt = 1/N:

  double s[3] = {w[0], w[1], w[2]};
  double v[3] = {alpha * d[0], alpha * d[1], alpha * d[2]};

  // Compute λ = H(s)·s at start (from log-homogeneity: λ = -∇F(s) = H(s)s).
  double H[9];
  BarrierHessian(s[0], s[1], s[2], H);
  double lam[3];
  for (int i = 0; i < 3; ++i)
    lam[i] = H[3*i]*s[0] + H[3*i+1]*s[1] + H[3*i+2]*s[2];

  const int N = 8;
  double dt = 1.0 / N;

  for (int step = 0; step < N; ++step) {
    // Half-step s.
    double s_half[3] = {s[0]+0.5*dt*v[0], s[1]+0.5*dt*v[1], s[2]+0.5*dt*v[2]};

    // λ at midpoint (exact from identity).
    double H_half[9];
    BarrierHessian(s_half[0], s_half[1], s_half[2], H_half);
    double lam_half[3];
    for (int i = 0; i < 3; ++i)
      lam_half[i] = H_half[3*i]*s_half[0] + H_half[3*i+1]*s_half[1] + H_half[3*i+2]*s_half[2];

    // Midpoint velocity from λ change: ṡ = -H⁻¹ · λ̇ ≈ -H⁻¹ · (λ_half - λ)/(dt/2).
    double dlam[3] = {(lam_half[0]-lam[0])/(0.5*dt),
                      (lam_half[1]-lam[1])/(0.5*dt),
                      (lam_half[2]-lam[2])/(0.5*dt)};
    double neg_dlam[3] = {-dlam[0], -dlam[1], -dlam[2]};
    double v_mid[3];
    Solve3x3(H_half, neg_dlam, v_mid);

    // Full-step s using midpoint velocity.
    s[0] += dt * v_mid[0];
    s[1] += dt * v_mid[1];
    s[2] += dt * v_mid[2];

    // λ at new s (exact from identity).
    double H_new[9];
    BarrierHessian(s[0], s[1], s[2], H_new);
    double lam_new[3];
    for (int i = 0; i < 3; ++i)
      lam_new[i] = H_new[3*i]*s[0] + H_new[3*i+1]*s[1] + H_new[3*i+2]*s[2];

    // Update velocity from λ change over second half-step.
    double dlam2[3] = {(lam_new[0]-lam_half[0])/(0.5*dt),
                       (lam_new[1]-lam_half[1])/(0.5*dt),
                       (lam_new[2]-lam_half[2])/(0.5*dt)};
    double neg_dlam2[3] = {-dlam2[0], -dlam2[1], -dlam2[2]};
    Solve3x3(H_new, neg_dlam2, v);

    // Update λ for next step.
    lam[0] = lam_new[0]; lam[1] = lam_new[1]; lam[2] = lam_new[2];
  }

  w[0] = s[0]; w[1] = s[1]; w[2] = s[2];
}

bool ExpConeOps::InvertGradient(const double* lambda, double* x,
                                 int max_iter) {
  // Solve -∇F(x) = λ by Newton: residual r(x) = -∇F(x) - λ,
  // Jacobian J = -∇²F(x) = -H.  Newton step: H·δx = r.
  // Start from a feasible interior point.
  // Initial guess: x = (0, 1, e+1) (near analytic center).
  x[0] = 0; x[1] = 1.0; x[2] = std::exp(1.0) + 1.0;

  for (int iter = 0; iter < max_iter; ++iter) {
    double g[3], H[9];
    BarrierGrad(x[0], x[1], x[2], g);
    // Residual: -g - lambda.
    double res[3] = {-g[0] - lambda[0], -g[1] - lambda[1], -g[2] - lambda[2]};
    double rnorm = std::sqrt(res[0]*res[0] + res[1]*res[1] + res[2]*res[2]);
    if (rnorm < 1e-12) return true;

    // Newton step: H · dx = res (since J = -H, and we want J·dx = -res,
    // i.e. -H·dx = -res, i.e. H·dx = res).
    BarrierHessian(x[0], x[1], x[2], H);
    double dx[3];
    Solve3x3(H, res, dx);

    // Line search: ensure we stay interior.
    double step = 1.0;
    for (int ls = 0; ls < 20; ++ls) {
      double xn[3] = {x[0]+step*dx[0], x[1]+step*dx[1], x[2]+step*dx[2]};
      if (xn[1] > 0 && xn[2] > xn[1] * std::exp(xn[0]/xn[1])) {
        x[0] = xn[0]; x[1] = xn[1]; x[2] = xn[2];
        break;
      }
      step *= 0.5;
    }
  }
  return false;  // didn't converge
}

void ExpConeOps::bregmanMidpointStep(double* w, double alpha,
                                      const double* d) const {
  // Bregman midpoint: second-order approximation to Levi-Civita geodesic.
  //
  // 1. Compute dual point: λ₀ = -∇F(x₀)
  // 2. Compute dual velocity: δλ = -∇²F(x₀) · (α·d) = -H · v
  // 3. Primal half-step: x_{1/2} = x₀ + (α/2)·d
  // 4. Map to dual at midpoint: λ_{1/2} = -∇F(x_{1/2})
  // 5. Dual full-step using midpoint velocity:
  //    λ₁ = λ₀ + α · (-∇²F(x_{1/2}) · d)
  // 6. Invert: x₁ = (-∇F)⁻¹(λ₁)

  double x0[3] = {w[0], w[1], w[2]};
  double v[3] = {alpha*d[0], alpha*d[1], alpha*d[2]};

  // Step 1: dual point at x₀.
  double g0[3];
  BarrierGrad(x0[0], x0[1], x0[2], g0);
  double lam0[3] = {-g0[0], -g0[1], -g0[2]};

  // Step 2: primal half-step (Euler in primal-flat coords).
  double xhalf[3] = {x0[0] + 0.5*v[0], x0[1] + 0.5*v[1], x0[2] + 0.5*v[2]};

  // Check xhalf is interior; if not, shrink.
  if (xhalf[1] <= 0 || xhalf[2] <= xhalf[1]*std::exp(xhalf[0]/xhalf[1])) {
    for (int k = 0; k < 3; ++k) xhalf[k] = x0[k] + 0.25*v[k];
  }

  // Step 3: dual point at midpoint.
  double ghalf[3];
  BarrierGrad(xhalf[0], xhalf[1], xhalf[2], ghalf);
  double lamhalf[3] = {-ghalf[0], -ghalf[1], -ghalf[2]};

  // Step 4: extrapolate dual: λ₁ = 2·λ_{1/2} - λ₀
  // (midpoint rule: λ̇ ≈ (λ_{1/2} - λ₀)/(α/2), so λ₁ = λ₀ + α·λ̇).
  double lam1[3] = {2*lamhalf[0]-lam0[0], 2*lamhalf[1]-lam0[1], 2*lamhalf[2]-lam0[2]};

  // Step 6: invert gradient map.
  double x1[3];
  // Use x0 as initial guess (closer than default).
  x1[0] = x0[0]; x1[1] = x0[1]; x1[2] = x0[2];
  // Newton solve: -∇F(x1) = lam1.
  for (int iter = 0; iter < 15; ++iter) {
    double g[3], H[9];
    BarrierGrad(x1[0], x1[1], x1[2], g);
    double res[3] = {-g[0]-lam1[0], -g[1]-lam1[1], -g[2]-lam1[2]};
    double rnorm = res[0]*res[0] + res[1]*res[1] + res[2]*res[2];
    if (rnorm < 1e-24) break;
    BarrierHessian(x1[0], x1[1], x1[2], H);
    double dx[3];
    Solve3x3(H, res, dx);
    double step = 1.0;
    for (int ls = 0; ls < 20; ++ls) {
      double xn[3] = {x1[0]+step*dx[0], x1[1]+step*dx[1], x1[2]+step*dx[2]};
      if (xn[1] > 1e-15 && xn[2] > xn[1]*std::exp(xn[0]/xn[1]) + 1e-15) {
        x1[0] = xn[0]; x1[1] = xn[1]; x1[2] = xn[2];
        break;
      }
      step *= 0.5;
    }
  }

  w[0] = x1[0]; w[1] = x1[1]; w[2] = x1[2];
}

void ExpConeOps::setIdentity(double* out, int /*size*/) const {
  // Interior point: (0, 1, exp(0)) = (0, 1, 1).
  // Actually the "analytic center" of the barrier:
  // ∇F = 0 at x=0, y=1, z=2 (where u = z - y*e^0 = 1, and dF/dy = 0 gives y=1).
  // Check: dF/dx = e/u = 1/1 = 1 ≠ 0. Hmm.
  // The analytic center satisfies ∇F = -e (the "identity direction").
  // For the exponential cone, a natural interior point is (0, 1, e) ≈ (0, 1, 2.718).
  // But there's no canonical "identity" like symmetric cones have.
  // Use (0, 1, 1+1) = (0, 1, 2) as a well-centered interior point.
  out[0] = 0.0;
  out[1] = 1.0;
  out[2] = std::exp(1.0);  // z = y*e^{x/y} + 1 at center
}

double ExpConeOps::normInf(const double* a, int /*size*/) const {
  return std::max({std::abs(a[0]), std::abs(a[1]), std::abs(a[2])});
}

double ExpConeOps::squaredNorm(const double* a, int /*size*/) const {
  return a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
}

double ExpConeOps::dot(const double* a, const double* b, int /*size*/) const {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

double ExpConeOps::minEigenvalue(const double* a, int /*size*/) const {
  // Not meaningful for exp cone (not a symmetric cone).
  // Return min component as a proxy.
  return std::min({a[0], a[1], a[2]});
}

void ExpConeOps::project(double* out, const double* a, int /*size*/) const {
  // Projection onto the exponential cone.  Rough approximation:
  // if already feasible, keep; otherwise project onto boundary.
  // TODO: proper projection.
  out[0] = a[0]; out[1] = a[1]; out[2] = a[2];
  if (a[1] > 0 && a[2] >= a[1] * std::exp(a[0] / a[1])) return;
  // Fallback: move z to boundary.
  out[1] = std::max(a[1], 1e-10);
  out[2] = out[1] * std::exp(out[0] / out[1]) + 1e-10;
}

const ExpConeOps& expConeOps() {
  static const ExpConeOps instance;
  return instance;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
