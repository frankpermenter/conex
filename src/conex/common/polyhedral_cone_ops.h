// Barrier operations for the polyhedral cone K = {x : Cx >= 0}.
//
// Barrier: F(x) = -Σ log(c_i^T x)  where c_i are rows of C.
// Gradient: ∇F(x) = -C^T (1/s)  where s = Cx (elementwise).
// Hessian: H(x) = C^T diag(1/s²) C.
// Barrier parameter: ν = m (number of rows of C).
//
// This is NOT a singleton — each instance stores its own C matrix.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "conex/common/symmetric_cone_operations.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class PolyhedralConeOps : public BarrierConeOperations {
 public:
  explicit PolyhedralConeOps(const Eigen::MatrixXd& C,
                             bool use_symmetric = false)
      : C_(C), use_symmetric_(use_symmetric) {}

  // --- z-space operations (z stores the primal point directly) ---

  void computeGradient(double* grad, const double* z, int n) const override {
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::VectorXd s = C_ * zv;
    Eigen::Map<Eigen::VectorXd> gv(grad, n);
    gv = -C_.transpose() * s.cwiseInverse();
  }

  void hessianProduct(double* out, const double* z, const double* v,
                      int n) const override {
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::Map<const Eigen::VectorXd> vv(v, n);
    Eigen::VectorXd s = C_ * zv;
    Eigen::VectorXd s_inv2 = s.cwiseInverse().cwiseAbs2();
    Eigen::Map<Eigen::VectorXd> ov(out, n);
    ov = C_.transpose() * (s_inv2.asDiagonal() * (C_ * vv));
  }

  void hessian(double* out, const double* z, int n) const override {
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::VectorXd s = C_ * zv;
    Eigen::VectorXd s_inv = s.cwiseInverse();
    // H = C^T diag(s^{-2}) C.  Build via WC = diag(s^{-1}) C, H = WC^T WC.
    Eigen::MatrixXd WC = s_inv.asDiagonal() * C_;
    Eigen::Map<Eigen::MatrixXd> H(out, n, n);
    H.noalias() = WC.transpose() * WC;
  }

  double hessianNormSquared(const double* z, const double* target,
                            int n) const override {
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::Map<const Eigen::VectorXd> tv(target, n);
    Eigen::VectorXd s = C_ * zv;
    Eigen::VectorXd d = C_ * (tv - zv);
    Eigen::VectorXd r = d.cwiseQuotient(s);
    return r.squaredNorm();
  }

  double stepSize(const double* z, const double* target,
                  int n) const override {
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::Map<const Eigen::VectorXd> tv(target, n);
    Eigen::VectorXd s = C_ * zv;
    Eigen::VectorXd r = (C_ * (tv - zv)).cwiseQuotient(s);
    double d_inf = r.cwiseAbs().maxCoeff();
    return std::min(1.0, 2.0 / (d_inf * d_inf));
  }

  void geodesicStepTarget(double* z, double alpha, const double* target,
                          int n) const override {
    Eigen::Map<Eigen::VectorXd> zv(z, n);
    Eigen::Map<const Eigen::VectorXd> tv(target, n);

    if (use_symmetric_) {
      symmetricStep(zv, alpha * (tv - zv), n);
    } else {
      verletStep(zv, alpha * (tv - zv), n);
    }
  }

 private:
  void verletStep(Eigen::Map<Eigen::VectorXd>& zv,
                  const Eigen::VectorXd& vel_in, int n) const {
    Eigen::VectorXd vel = vel_in;
    Eigen::VectorXd pos = zv;

    // Adaptive substeps based on Riemannian speed.
    Eigen::VectorXd s0 = C_ * pos;
    double speed = (C_ * vel).cwiseQuotient(s0).norm();
    int steps = std::max(1, (int)std::ceil(speed));
    double dt = 1.0 / steps;

    for (int step = 0; step < steps; ++step) {
      auto accel = [&](const Eigen::VectorXd& p, const Eigen::VectorXd& v)
          -> Eigen::VectorXd {
        Eigen::VectorXd s = C_ * p;
        if (s.minCoeff() <= 0) return Eigen::VectorXd::Zero(n);
        Eigen::VectorXd Cv = C_ * v;
        Eigen::VectorXd T = -2.0 * C_.transpose() *
            (Cv.cwiseAbs2().cwiseQuotient(s.cwiseAbs2().cwiseProduct(s)));
        Eigen::VectorXd s_inv = s.cwiseInverse();
        Eigen::MatrixXd WC = s_inv.asDiagonal() * C_;
        Eigen::MatrixXd H = WC.transpose() * WC;
        return H.ldlt().solve(-0.5 * T);
      };

      Eigen::VectorXd a = accel(pos, vel);
      vel += 0.5 * dt * a;
      pos += dt * vel;
      a = accel(pos, vel);
      vel += 0.5 * dt * a;
    }
    zv = pos;
  }

  // Symmetric integrator from integrator.tex Definition.
  // Solves: ∇φ(z1) - ∇φ(z0) = H(z0)(2h·v0 - (z1 - z0))
  // for z1 via Newton, using h=1, v0 = vel.
  void symmetricStep(Eigen::Map<Eigen::VectorXd>& zv,
                     const Eigen::VectorXd& vel_in, int n) const {
    // Adaptive substeps based on Riemannian speed.
    Eigen::VectorXd s0 = C_ * zv;
    double speed = (C_ * vel_in).cwiseQuotient(s0).norm();
    int steps = std::max(1, (int)std::ceil(speed));
    Eigen::VectorXd vel = vel_in / steps;

    for (int step = 0; step < steps; ++step) {
      symmetricSubstep(zv, vel, n);
    }
  }

  void symmetricSubstep(Eigen::Map<Eigen::VectorXd>& zv,
                        Eigen::VectorXd& vel, int n) const {
    Eigen::VectorXd z0 = zv;
    Eigen::VectorXd s0 = C_ * z0;
    Eigen::VectorXd lam0 = -C_.transpose() * s0.cwiseInverse();  // ∇φ(z0)

    // H(z0) = C^T diag(s0^{-2}) C.
    Eigen::VectorXd s0_inv = s0.cwiseInverse();
    Eigen::MatrixXd WC0 = s0_inv.asDiagonal() * C_;
    Eigen::MatrixXd H0 = WC0.transpose() * WC0;

    Eigen::VectorXd u = 2.0 * vel;  // 2h·v0 with h=1

    // Newton solve for z1. Initial guess: z1 = z0 + vel (tangent line).
    Eigen::VectorXd z1 = z0 + vel;
    for (int iter = 0; iter < 20; ++iter) {
      Eigen::VectorXd s1 = C_ * z1;
      if (s1.minCoeff() <= 0) {
        // Backtrack towards z0.
        z1 = 0.5 * (z1 + z0);
        continue;
      }
      Eigen::VectorXd lam1 = -C_.transpose() * s1.cwiseInverse();
      Eigen::VectorXd d = z1 - z0;

      // F(z1) = (lam1 - lam0) - H0*(u - d)
      Eigen::VectorXd F = (lam1 - lam0) - H0 * (u - d);

      if (F.norm() < 1e-13 * (1.0 + lam0.norm())) break;

      // Jacobian: H(z1) + H0.
      Eigen::VectorXd s1_inv = s1.cwiseInverse();
      Eigen::MatrixXd WC1 = s1_inv.asDiagonal() * C_;
      Eigen::MatrixXd H1 = WC1.transpose() * WC1;
      Eigen::MatrixXd J = H1 + H0;

      z1 -= J.ldlt().solve(F);
    }
    // Update velocity from second equation:
    // ∇φ(z1) - ∇φ(z0) = H(z1)(2·v1 - d) → v1 = (d + H(z1)^{-1}(lam1-lam0))/2
    Eigen::VectorXd s1 = C_ * z1;
    Eigen::VectorXd lam1 = -C_.transpose() * s1.cwiseInverse();
    Eigen::VectorXd s1_inv = s1.cwiseInverse();
    Eigen::MatrixXd WC1 = s1_inv.asDiagonal() * C_;
    Eigen::MatrixXd H1 = WC1.transpose() * WC1;
    Eigen::VectorXd d = z1 - z0;
    vel = 0.5 * (d + H1.ldlt().solve(lam1 - lam0));

    zv = z1;
  }

 public:

  double lineSearchTarget(const double* z, const double* target0,
                           const double* target1, int n) const override {
    // Primal-dual feasibility: |C*zdot / s|_i <= 1 for all i.
    // zdot(k) = target0 + k*target1 - z, so C*zdot = r0 + k*r1
    // where r0 = C*target0 - s, r1 = C*target1, s = Cz.
    // Rescale: d0 = r0/s, d1 = r1/s.  Find max k with ||d0+k*d1||_inf <= 1.
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::Map<const Eigen::VectorXd> t0(target0, n);
    Eigen::Map<const Eigen::VectorXd> t1(target1, n);
    Eigen::VectorXd s = C_ * zv;
    Eigen::VectorXd d0 = (C_ * t0 - s).cwiseQuotient(s);
    Eigen::VectorXd d1 = (C_ * t1).cwiseQuotient(s);
    double k_max = std::numeric_limits<double>::max();
    for (int i = 0; i < d0.size(); ++i) {
      if (d1(i) > 0)
        k_max = std::min(k_max, (1.0 - d0(i)) / d1(i));
      else if (d1(i) < 0)
        k_max = std::min(k_max, (-1.0 - d0(i)) / d1(i));
    }
    return k_max;
  }

  double barrierParameter(int /*size*/) const override {
    return static_cast<double>(C_.rows());
  }

  double barrierValue(const double* z, int n) const override {
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::VectorXd s = C_ * zv;
    double val = 0;
    for (int i = 0; i < s.size(); ++i) {
      if (s(i) <= 0) return std::numeric_limits<double>::infinity();
      val -= std::log(s(i));
    }
    return val;
  }

  // Interior point: solve C*z > 0 with z = C^T * ones (C^T * 1 gives
  // a point with all slacks = C * C^T * 1, which is PD if C has full
  // row rank). Falls back to ones if that fails.
  void getInteriorPoint(double* out, int n) const override {
    Eigen::Map<Eigen::VectorXd> zv(out, n);
    zv = C_.transpose() * Eigen::VectorXd::Ones(C_.rows());
    if ((C_ * zv).minCoeff() <= 0) {
      zv.setOnes();
    }
  }

  // --- BarrierOps-style methods ---

  bool isInterior(const double* z, int n) const {
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::VectorXd s = C_ * zv;
    return s.minCoeff() > 0;
  }


 private:
  Eigen::MatrixXd C_;
  bool use_symmetric_ = false;
};

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
