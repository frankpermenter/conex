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

class PolyhedralConeOps : public SymmetricConeOperations {
 public:
  explicit PolyhedralConeOps(const Eigen::MatrixXd& C) : C_(C) {}

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
    Eigen::VectorXd vel = alpha * (tv - zv);
    Eigen::VectorXd pos = zv;

    // Primal Verlet integrator with Christoffel symbols.
    // Adaptive substeps: each substep covers Riemannian distance ~1.
    Eigen::VectorXd s0 = C_ * pos;
    double speed = (C_ * vel).cwiseQuotient(s0).norm();  // ||ẋ||_H
    int steps = std::max(1, (int)std::ceil(speed));
    double dt = 1.0 / steps;

    for (int step = 0; step < steps; ++step) {
      // Acceleration: a = -(1/2) H^{-1} T(v, v).
      auto accel = [&](const Eigen::VectorXd& p, const Eigen::VectorXd& v)
          -> Eigen::VectorXd {
        Eigen::VectorXd s = C_ * p;
        if (s.minCoeff() <= 0) return Eigen::VectorXd::Zero(n);
        Eigen::VectorXd Cv = C_ * v;
        // T(v,v) = -2 C^T ((Cv)^2 / s^3).
        Eigen::VectorXd T = -2.0 * C_.transpose() *
            (Cv.cwiseAbs2().cwiseQuotient(s.cwiseAbs2().cwiseProduct(s)));
        // H = C^T diag(s^{-2}) C.
        Eigen::VectorXd s_inv = s.cwiseInverse();
        Eigen::MatrixXd WC = s_inv.asDiagonal() * C_;
        Eigen::MatrixXd H = WC.transpose() * WC;
        // a = -(1/2) H^{-1} T = H^{-1} (C^T ((Cv)^2 / s^3)).
        return H.ldlt().solve(-0.5 * T);
      };

      // Half-step velocity.
      Eigen::VectorXd a = accel(pos, vel);
      vel += 0.5 * dt * a;

      // Full-step position.
      pos += dt * vel;

      // Second half-step velocity.
      a = accel(pos, vel);
      vel += 0.5 * dt * a;
    }

    zv = pos;
  }

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

  // --- BarrierOps-style methods ---

  bool isInterior(const double* z, int n) const {
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::VectorXd s = C_ * zv;
    return s.minCoeff() > 0;
  }

  // --- SymmetricConeOperations stubs (not a symmetric cone) ---
  void product(double*, const double*, const double*, int) const override {}
  void geodesicUpdate(double*, const double*, double, const double*,
                      int) const override {}
  void setIdentity(double*, int) const override {}
  double normInf(const double*, int) const override { return 0; }
  double squaredNorm(const double*, int) const override { return 0; }
  double dot(const double*, const double*, int) const override { return 0; }
  void sqrt(double*, const double*, int) const override {}
  void quadraticRepresentation(double*, const double*, const double*,
                               int) const override {}
  void solveLyapunovForD(double*, const double*, const double*,
                         int) const override {}
  void abs(double*, const double*, int) const override {}
  double minEigenvalue(const double*, int) const override { return 0; }
  void updateAutomorphism(double*, double*, double, const double*,
                          int) const override {}
  void updateAutomorphismP(double*, double*, double, const double*,
                           int) const override {}
  void updateM(double*, double*, double, const double*, int) const override {}
  void applyM(double*, const double*, const double*, int) const override {}
  void applyMt(double*, const double*, const double*, int) const override {}
  void squareM(double*, const double*, int) const override {}
  double lineSearchK(const double*, const double*, int) const override {
    return 0;
  }
  void project(double*, const double*, int) const override {}

 private:
  Eigen::MatrixXd C_;
};

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
