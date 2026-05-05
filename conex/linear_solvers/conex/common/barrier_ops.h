// Abstract interface for log-homogeneous self-concordant barrier operations.
//
// Any proper cone K with a computable LHSCB F can implement this interface,
// enabling the geodesic IPM to optimize over K without symmetric cone
// structure (no Jordan algebra, no automorphisms).
//
// The geodesic IPM uses these operations as follows:
//   - gradient:       centering RHS (A^T(-∇F(s))), lambda recovery (λ = -μ∇F(s))
//   - hessian:        Gram matrix assembly (A^T ∇²F(s) A)
//   - normH:          step size (α = 1/(1 + ||d||_H)), line search
//   - ExponentialMap: geodesic step on the Hessian manifold
//   - isInterior:     feasibility check
//   - barrierParameter: gap formula (⟨s,λ⟩ = νμ at centering)

#pragma once

namespace conex {

class BarrierOps {
 public:
  virtual ~BarrierOps() = default;

  // Barrier parameter ν (degree of the LHSCB).
  // For log-homogeneous F: F(ts) = F(s) - ν log(t).
  virtual double barrierParameter() const = 0;

  // Dimension of the ambient space (number of doubles per point).
  virtual int dim() const = 0;

  // Gradient: out = ∇F(s).  |out| = |s| = dim().
  virtual void gradient(double* out, const double* s) const = 0;

  // Full Hessian matrix: out = ∇²F(s), column-major, dim() × dim().
  // |out| = dim() * dim().
  virtual void hessian(double* out, const double* s) const = 0;

  // Hessian norm: ||v||_{H(s)} = sqrt(v^T ∇²F(s) v).
  virtual double normH(const double* s, const double* v) const = 0;

  // Riemannian exponential map on the Hessian manifold of F:
  //   s_out = Exp_s(alpha * d).
  // Implementations may use the Legendre midpoint method (second-order,
  // no third derivatives) or higher-order Yoshida compositions.
  virtual void ExponentialMap(double* s_out, const double* s,
                              double alpha, const double* d) const = 0;

  // Interior check: returns true if s ∈ int(K), i.e., F(s) is finite.
  virtual bool isInterior(const double* s) const = 0;
};

}  // namespace conex
