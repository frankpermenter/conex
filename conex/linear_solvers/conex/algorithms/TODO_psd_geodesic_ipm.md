# PSD Geodesic IPM: Hybrid Algorithm Update

## Defining equations

The primal-dual pair (slack, lambda) is parameterized by (W, r, Delta):

    Lambda = P(W^{1/2}) (R + Delta)
    Slack  = P(W^{-1/2}) (R - Delta)

where P(X)Y = XYX is the quadratic representation.

Complementarity: Lambda * Slack involves (R + Delta)(R - Delta) in the
W-scaled space, generalizing s*lambda = (r+delta)(r-delta) from nonneg.

## Delta from d

The geodesic direction D lives in the tangent space.  Delta is the
symmetric part of the RD product, defined by the Lyapunov equation:

    R D + D R = 2 Delta

For nonneg (diagonal): Delta_i = R_i D_i (trivial).

For PSD: solve the Lyapunov equation for Delta given (R, D).
Eigendecompose R = V diag(lambda) V^T, then in the eigenbasis:

    Delta_ij = (lambda_i + lambda_j) / 2 * D_ij

(Hadamard product with the matrix of harmonic-mean-like weights.)

## Geodesic update (the key change)

Current nonneg update: W_i *= exp(alpha * D_i), R unchanged.

PSD update:

1. Compute M = W^{1/2} expm(alpha * D / 2)
2. Polar decomposition: M = P T  (P psd, T orthogonal)
3. W_new = P^2
4. R_new = T^T R T

### Interpretation

M is a cone automorphism.  Polar decomposition separates it into:
- P: the "scaling" part that changes the metric (updates W)
- T: the "rotation" part that reorients the frame (updates R)

For nonneg, all entries are positive scalars, so T = I always.  The
rotational degree of freedom is unique to PSD (and other non-simple EJAs).

The automorphism M maps R to a dual-feasible point, and its adjoint
inverse maps R to a feasible slack.  The polar decomposition ensures
W stays PSD and R stays well-conditioned.

### For nonneg, this reduces to the current code

- M_i = sqrt(W_i) * exp(alpha*D_i/2) > 0
- Polar: P_i = M_i, T_i = 1
- W_new_i = M_i^2 = W_i * exp(alpha*D_i)  (current geodesicUpdate)
- R_new_i = R_i  (R unchanged)

## Changes needed

### ConeOps additions
- `polarDecomposition(P, T, M, size)`: M = P*T, P psd, T orthogonal
- `solveLyapunov(Delta, R, D, size)`: solve RD + DR = 2*Delta
  - Nonneg: Delta_i = R_i * D_i
  - PSD: eigendecompose R, Hadamard in eigenbasis

### geodesicUpdate signature change
Current: `geodesicUpdate(W, alpha, d)` modifies W in place.
New: needs to also update R.  Options:
  a. `geodesicUpdate(W, R, alpha, d)` modifies both W and R
  b. Return a struct {W_new, R_new}
  c. Separate into `computeM` + `polarUpdate(W, R, M)`

Option (a) is simplest.

### Hybrid algorithm changes
- Store R as a RowSpace alongside W
- d formula: solve Lyapunov for Delta, then d from Delta
- RHS formula: uses P(W^{1/2})(R + ...) structure
- Geodesic step: compute M, polar decompose, update W and R
- Shrink step: update R (current shrinkR already modifies r)
- Gap computation: uses R (already does)

### SetWeights — unchanged
The orthogonal rotation T is absorbed into R, so W remains the pure
scaling factor.  SetWeights(W^2) and the Gram A^T kron(W^2, W^2) A
are unaffected by the polar decomposition.  The rotation only enters
through R in the RHS (Delta computation and primal-dual residuals).
The factored system is the same; only the RHS vector changes.
