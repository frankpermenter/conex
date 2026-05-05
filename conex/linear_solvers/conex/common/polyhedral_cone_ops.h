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
    double dn = std::sqrt(hessianNormSquared(z, target, n));
    return 1.0 / (1.0 + dn);
  }

  void geodesicStepTarget(double* z, double alpha, const double* target,
                          int n) const override {
    // Tangent d = target - z.  Integrate via Legendre midpoint.
    Eigen::Map<Eigen::VectorXd> zv(z, n);
    Eigen::Map<const Eigen::VectorXd> tv(target, n);
    Eigen::VectorXd d = tv - zv;

    // Half-step in primal.
    Eigen::VectorXd z_half = zv + 0.5 * alpha * d;

    // Dual at start and midpoint.
    Eigen::VectorXd lam0 = -C_.transpose() * (C_ * zv).cwiseInverse();
    Eigen::VectorXd lam_half = -C_.transpose() * (C_ * z_half).cwiseInverse();

    // Dual extrapolation: midpoint rule.
    Eigen::VectorXd lam1 = 2.0 * lam_half - lam0;

    // Invert gradient map: find z1 such that -∇F(z1) = -lam1,
    // i.e., C^T (1/(C z1)) = -lam1.
    // Newton solve starting from z + alpha*d.
    Eigen::VectorXd z1 = zv + alpha * d;
    for (int iter = 0; iter < 20; ++iter) {
      Eigen::VectorXd s1 = C_ * z1;
      Eigen::VectorXd g1 = -C_.transpose() * s1.cwiseInverse();
      Eigen::VectorXd res = g1 - lam1;
      if (res.norm() < 1e-13 * (1.0 + lam1.norm())) break;
      // H(z1) = C^T diag(s1^{-2}) C.
      Eigen::VectorXd s1_inv = s1.cwiseInverse();
      Eigen::MatrixXd WC = s1_inv.asDiagonal() * C_;
      Eigen::MatrixXd H = WC.transpose() * WC;
      z1 -= H.ldlt().solve(res);
    }
    zv = z1;
  }

  double lineSearchTarget(const double* z, const double* target0,
                           const double* target1, int n) const override {
    // Dikin bound: ||C(target0 + k*target1 - z) / s||² <= 1
    // where s = Cz.  Quadratic in k: a + 2fk + pk² <= 1.
    Eigen::Map<const Eigen::VectorXd> zv(z, n);
    Eigen::Map<const Eigen::VectorXd> t0(target0, n);
    Eigen::Map<const Eigen::VectorXd> t1(target1, n);
    Eigen::VectorXd s = C_ * zv;
    Eigen::VectorXd s_inv = s.cwiseInverse();
    Eigen::VectorXd r0 = (C_ * (t0 - zv)).cwiseProduct(s_inv);
    Eigen::VectorXd r1 = (C_ * t1).cwiseProduct(s_inv);
    double aa = r0.squaredNorm();
    double ff = r0.dot(r1);
    double pp = r1.squaredNorm();
    double disc = 4*ff*ff - 4*pp*(aa - 1);
    if (disc < 0 || pp < 1e-30) return 0;
    double k1 = (-2*ff + std::sqrt(disc)) / (2*pp);
    double k2 = (-2*ff - std::sqrt(disc)) / (2*pp);
    return std::max(std::max(k1, k2), 0.0);
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
