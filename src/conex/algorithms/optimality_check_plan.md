# Optimality Check Plan for Geodesic IPM

## The primal-dual triple (x, s, λ)

At the last Newton iteration with barrier parameter mu = 1/k², the
geodesic IPM has:

### Primal variable x
Already computed: `x = y/k` (or `y0/k + y1` for the decomposition path).
Available in `GeodesicResult::x`.

### Dual variable λ (Lambda)
From the parameterization:
```
Lambda = sqrt(mu) * P(W^{1/2})(e + d)
       = (1/k) * P(W^{1/2})(e + d)
```
For the hybrid with r:
```
Lambda = P(W^{1/2})(r + Delta)
```
Computed from: `sqrtW = sqrt(W)`, then `quadraticRepresentation(sqrtW, r + delta)`.

This is a RowSpace — lives in cone space.

### Primal slack s
From primal feasibility:
```
s = Ax + b
```
Computed via: `kkt.MultiplyA(x_rhs, s); s += b`.

Or from the parameterization:
```
s = sqrt(mu) * P(W^{-1/2})(e - d)
```
For the hybrid: `s = P(W^{-1/2})(r - Delta)`.

## KKT optimality conditions

1. **Primal feasibility**: s = Ax + b, s in cone K.
   - Check: `||s - (Ax + b)|| < tol`.
   - Check: `minEigenvalue(s) > -tol` (s in cone).

2. **Dual feasibility**: A^T λ = Qx + c, λ in cone K*.
   - Check: `||A^T λ - Qx - c|| < tol`.
   - Check: `minEigenvalue(λ) > -tol` (λ in dual cone = same cone for SOC/PSD/nonneg).

3. **Complementarity**: <s, λ> = mu * rank.
   - Check: `|dot(s, λ) - mu * rank| < tol`.
   - Or for the hybrid: `gap(r, delta) ≈ 0`.

4. **Duality gap**: c^T x - b^T λ = mu * rank (for LP).
   - Check: `|c^T x + <b, λ> - mu * rank| < tol`.
   - This combines primal/dual objectives.

## Implementation plan

### New function: `CheckOptimality`
```cpp
struct OptimalityReport {
  double primal_residual;    // ||s - (Ax + b)||
  double dual_residual;      // ||A^T λ + Qx - c||
  double complementarity;    // <s, λ>
  double min_slack;           // min eigenvalue of s
  double min_dual;            // min eigenvalue of λ
  double mu;                  // barrier parameter
};

OptimalityReport CheckOptimality(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W,
    const RowSpace& r,
    const RowSpace& d,
    const RowSpace& delta,
    double mu);
```

### Steps inside CheckOptimality:
1. Recover x from the last Newton solve (already in GeodesicResult::x).
2. Compute s = Ax + b via `kkt.MultiplyA` + `kkt.GetAffineTerm`.
3. Compute λ = P(W^{1/2})(r + delta) via `quadraticRepresentation(sqrt(W), r + delta)`.
4. Compute A^T λ via `kkt.AccumulateAtranspose(lambda, rhs)`.
5. Compute Qx via `kkt.AccumulateQx(x_rhs, rhs)` (for QP).
6. Check residuals.

### Where to call it:
- At the end of `SolveGeodesicHybrid`, before returning.
- At the end of `SolveGeodesicLP`, before returning.
- Optionally in `GeodesicCenter` for debugging.

### Output:
- Add `OptimalityReport` to `GeodesicResult`.
- Print in verbose mode.
- Optionally warn if residuals exceed tolerance.

### For the non-hybrid path (GeodesicCenter, SolveGeodesicLP):
- r = sqrt(mu) * ones (not explicitly stored).
- delta = sqrt(mu) * d (from Lyapunov with r = scalar * I).
- Lambda = sqrt(mu) * P(W^{1/2})(e + d) = (1/k) * P(W^{1/2})(e + d).

### Cone membership check:
- Nonneg: all entries >= 0.
- PSD: min eigenvalue >= 0.
- SOC: t >= ||x||, i.e., minEigenvalue >= 0.
- All via `minEigenvalue(s)` and `minEigenvalue(lambda)`.

### Complementarity:
- `dot(s, lambda)` uses the trace inner product (dispatched through ConeOps).
- Should equal `mu * rank` where rank is the EJA rank:
  - Nonneg: rank = m (number of constraints).
  - PSD: rank = n (matrix dimension).
  - SOC: rank = 2.
- The `total_rows()` gives the DIMENSION not the rank. Need a `rank()`
  method on ConeOps, or compute as `dot(e, e)` where e is the identity.

### Rank computation:
- `dot(e, e)` where e = identity:
  - Nonneg: dot(ones, ones) = 2 * sum(1) = 2m. No — dot for nonneg is
    just sum(a_i * b_i), so dot(ones, ones) = m.
  - Wait, nonneg dot doesn't have the factor of 2. PSD dot = trace(A^T B)
    = sum entries. SOC dot = 2(t₁t₂ + x₁·x₂).
  - So dot(e, e):
    - Nonneg: m (identity = ones).
    - PSD: trace(I^T I) = trace(I) = n. But squaredNorm(I) = ||I||_F² = n.
      And dot(I, I) = sum(I .* I) = n. So rank = n.
    - SOC: dot((1,0,...,0), (1,0,...,0)) = 2*(1*1 + 0) = 2. So rank = 2.
  - This matches: `rank = dot(e, e) / 1` doesn't work for SOC since
    dot(e,e) = 2 but squaredNorm(e) = 2 as well.
  - Actually rank = number of eigenvalues:
    - Nonneg: m.
    - PSD: n (matrix dimension).
    - SOC: 2.
  - Simplest: add `rank()` to ConeOps. Or compute as squaredNorm(e).
    - Nonneg: squaredNorm(ones) = m. ✓
    - PSD: squaredNorm(I) = ||I||_F² = n. ✓
    - SOC: squaredNorm((1,0,...)) = 1² + 0² = 1... no.
      SOC squaredNorm uses eigenvalues: λ₁² + λ₂² = 1² + 1² = 2. ✓
  - So `rank = squaredNorm(identity)` works for all three cones!
