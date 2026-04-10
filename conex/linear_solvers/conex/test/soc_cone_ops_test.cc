#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <Eigen/Dense>
#include "conex/common/soc_cone_ops.h"

using Eigen::VectorXd;

namespace conex {
namespace EuclideanJordanAlgebra {
namespace {

class SOCConeOpsTest : public ::testing::Test {
 protected:
  SOCConeOps ops;
  static constexpr int n = 6;  // 1 + 5

  VectorXd Identity() {
    VectorXd e = VectorXd::Zero(n);
    e(0) = 1.0;
    return e;
  }

  VectorXd RandomInterior(int seed = 42) {
    srand(seed);
    VectorXd x(n);
    x.tail(n - 1) = 0.5 * VectorXd::Random(n - 1);
    x(0) = x.tail(n - 1).norm() + 1.0;
    return x;
  }

  double MaxDiff(const VectorXd& a, const VectorXd& b) {
    return (a - b).lpNorm<Eigen::Infinity>();
  }
};

// e ∘ x = x (identity element).
TEST_F(SOCConeOpsTest, ProductWithIdentity) {
  VectorXd e = Identity();
  VectorXd x = RandomInterior();
  VectorXd out(n);
  ops.product(out.data(), e.data(), x.data(), n);
  EXPECT_LT(MaxDiff(out, x), 1e-14);
}

// e ∘ e = e.
TEST_F(SOCConeOpsTest, IdentitySquared) {
  VectorXd e = Identity();
  VectorXd out(n);
  ops.product(out.data(), e.data(), e.data(), n);
  EXPECT_LT(MaxDiff(out, e), 1e-14);
}

// P(x)e = x ∘ x (quadratic rep of identity is Jordan square).
TEST_F(SOCConeOpsTest, QuadRepOfIdentity) {
  VectorXd e = Identity();
  VectorXd x = RandomInterior();
  VectorXd Pxe(n), x2(n);
  ops.quadraticRepresentation(Pxe.data(), x.data(), e.data(), n);
  ops.product(x2.data(), x.data(), x.data(), n);
  double err = MaxDiff(Pxe, x2);
  printf("P(x)e vs x*x: %.2e\n", err);
  EXPECT_LT(err, 1e-13);
}

// P(e)x = x (identity quadratic rep is identity map).
TEST_F(SOCConeOpsTest, QuadRepByIdentity) {
  VectorXd e = Identity();
  VectorXd x = RandomInterior();
  VectorXd out(n);
  ops.quadraticRepresentation(out.data(), e.data(), x.data(), n);
  EXPECT_LT(MaxDiff(out, x), 1e-14);
}

// P(x)(P(x^{-1})y) = y (quadratic rep is involutive).
TEST_F(SOCConeOpsTest, QuadRepInvolutive) {
  VectorXd x = RandomInterior();
  VectorXd y = RandomInterior(77);
  // x^{-1} = (t, -x1) / det(x) where det = t^2 - ||x1||^2.
  double t = x(0);
  double det = t * t - x.tail(n - 1).squaredNorm();
  VectorXd xinv(n);
  xinv(0) = t / det;
  xinv.tail(n - 1) = -x.tail(n - 1) / det;

  VectorXd Pinv_y(n), P_Pinv_y(n);
  ops.quadraticRepresentation(Pinv_y.data(), xinv.data(), y.data(), n);
  ops.quadraticRepresentation(P_Pinv_y.data(), x.data(), Pinv_y.data(), n);
  double err = MaxDiff(P_Pinv_y, y);
  printf("P(x)P(x^-1)y vs y: %.2e\n", err);
  EXPECT_LT(err, 1e-12);
}

// exp(0) = e.
TEST_F(SOCConeOpsTest, ExpOfZero) {
  VectorXd e = Identity();
  VectorXd zero = VectorXd::Zero(n);
  VectorXd out(n);
  // geodesicUpdate with a=e, alpha=1, d=0: out = P(e^{1/2}) exp(0) = e.
  ops.geodesicUpdate(out.data(), e.data(), 1.0, zero.data(), n);
  EXPECT_LT(MaxDiff(out, e), 1e-14);
}

// sqrt(e) = e.
TEST_F(SOCConeOpsTest, SqrtOfIdentity) {
  VectorXd e = Identity();
  VectorXd out(n);
  ops.sqrt(out.data(), e.data(), n);
  EXPECT_LT(MaxDiff(out, e), 1e-14);
}

// sqrt(x) ∘ sqrt(x) = x.
TEST_F(SOCConeOpsTest, SqrtSquared) {
  VectorXd x = RandomInterior();
  VectorXd sx(n), sx2(n);
  ops.sqrt(sx.data(), x.data(), n);
  ops.product(sx2.data(), sx.data(), sx.data(), n);
  double err = MaxDiff(sx2, x);
  printf("sqrt(x)^2 vs x: %.2e\n", err);
  EXPECT_LT(err, 1e-13);
}

// ||e||_inf = 1 (max eigenvalue of identity).
TEST_F(SOCConeOpsTest, NormInfIdentity) {
  VectorXd e = Identity();
  EXPECT_NEAR(ops.normInf(e.data(), n), 1.0, 1e-14);
}

// squaredNorm(e) = 2 (rank of SOC = 2, eigenvalues both 1).
TEST_F(SOCConeOpsTest, SquaredNormIdentity) {
  VectorXd e = Identity();
  EXPECT_NEAR(ops.squaredNorm(e.data(), n), 2.0, 1e-14);
}

// dot(e, e) = 2 (trace inner product of identity with itself).
TEST_F(SOCConeOpsTest, DotIdentity) {
  VectorXd e = Identity();
  EXPECT_NEAR(ops.dot(e.data(), e.data(), n), 2.0, 1e-14);
}

// <x, e> = trace(x) = 2*t (for SOC element (t, x1)).
TEST_F(SOCConeOpsTest, DotWithIdentity) {
  VectorXd e = Identity();
  VectorXd x = RandomInterior();
  double d = ops.dot(x.data(), e.data(), n);
  EXPECT_NEAR(d, 2.0 * x(0), 1e-14);
}

// project(x) = x when x is in the cone.
TEST_F(SOCConeOpsTest, ProjectInterior) {
  VectorXd x = RandomInterior();
  VectorXd out(n);
  ops.project(out.data(), x.data(), n);
  EXPECT_LT(MaxDiff(out, x), 1e-14);
}

// project(-e) = 0 (negative of identity projects to zero).
TEST_F(SOCConeOpsTest, ProjectNegativeIdentity) {
  VectorXd e = Identity();
  VectorXd neg_e = -e;
  VectorXd out(n);
  ops.project(out.data(), neg_e.data(), n);
  EXPECT_LT(out.norm(), 1e-14);
}

// abs(x) has nonneg eigenvalues.
TEST_F(SOCConeOpsTest, AbsNonneg) {
  VectorXd x = RandomInterior();
  x(0) = 0.1;  // make smallest eigenvalue possibly negative
  VectorXd ax(n);
  ops.abs(ax.data(), x.data(), n);
  EXPECT_GE(ops.minEigenvalue(ax.data(), n), -1e-14);
}

// minEigenvalue(e) = 1.
TEST_F(SOCConeOpsTest, MinEigIdentity) {
  VectorXd e = Identity();
  EXPECT_NEAR(ops.minEigenvalue(e.data(), n), 1.0, 1e-14);
}

// minEigenvalue(x) = t - ||x1|| for SOC.
TEST_F(SOCConeOpsTest, MinEigFormula) {
  VectorXd x = RandomInterior();
  double expected = x(0) - x.tail(n - 1).norm();
  EXPECT_NEAR(ops.minEigenvalue(x.data(), n), expected, 1e-14);
}

// Lyapunov round-trip: solveLyapunovForD(r, r*d) = d.
// (Forward: delta = r*d. Inverse: d = delta/r.)
TEST_F(SOCConeOpsTest, LyapunovRoundTrip) {
  VectorXd r = RandomInterior(99);
  VectorXd d(n);
  d(0) = 0.3; d.tail(n - 1) = 0.1 * VectorXd::Random(n - 1);
  VectorXd delta(n), d_recovered(n);
  // Forward: delta = lyap(r, d) = r ∘ d.
  ops.product(delta.data(), r.data(), d.data(), n);
  // Inverse: d = solveLyapunovForD(r, delta).
  ops.solveLyapunovForD(d_recovered.data(), r.data(), delta.data(), n);
  double err = MaxDiff(d_recovered, d);
  printf("Lyapunov round-trip: %.2e\n", err);
  EXPECT_LT(err, 1e-12);
}

// lineSearchK: verify ||d0 + k*d1||_inf ≤ 1 at the returned k.
TEST_F(SOCConeOpsTest, LineSearchBasic) {
  VectorXd d0(n), d1(n);
  // Ensure ||d0||_inf < 1 so k=0 is feasible.
  d0(0) = 0.3; d0.tail(n - 1) = 0.1 * VectorXd::Random(n - 1);
  d1(0) = 0.5; d1.tail(n - 1) = 0.2 * VectorXd::Random(n - 1);
  double k = ops.lineSearchK(d0.data(), d1.data(), n);
  EXPECT_GT(k, 0);
  // At k, ||d0 + k*d1||_inf should be ≈ 1.
  VectorXd dk = d0 + k * d1;
  double nrm = ops.normInf(dk.data(), n);
  printf("lineSearchK=%.4f, ||d(k)||=%.6f\n", k, nrm);
  EXPECT_LE(nrm, 1.0 + 1e-8);
}

// geodesicUpdate preserves cone membership.
TEST_F(SOCConeOpsTest, GeodesicPreservesCone) {
  VectorXd x = RandomInterior();
  VectorXd d(n);
  d(0) = 0.3; d.tail(n - 1) = 0.2 * VectorXd::Random(n - 1);
  VectorXd out(n);
  ops.geodesicUpdate(out.data(), x.data(), 0.5, d.data(), n);
  double min_eig = ops.minEigenvalue(out.data(), n);
  printf("After geodesicUpdate: min_eig=%.6f\n", min_eig);
  EXPECT_GT(min_eig, -1e-10);
}

// geodesicUpdate(w, 0) = w (zero direction is identity).
TEST_F(SOCConeOpsTest, GeodesicZeroDirection) {
  VectorXd w = RandomInterior();
  VectorXd zero = VectorXd::Zero(n);
  VectorXd out(n);
  ops.geodesicUpdate(out.data(), w.data(), 1.0, zero.data(), n);
  double err = MaxDiff(out, w);
  printf("geodesicUpdate(w, 0) vs w: %.2e\n", err);
  EXPECT_LT(err, 1e-13);
}

// geodesicUpdate(w, alpha, d) at alpha=0 returns w.
TEST_F(SOCConeOpsTest, GeodesicZeroAlpha) {
  VectorXd w = RandomInterior();
  VectorXd d(n);
  d(0) = 0.5; d.tail(n-1) = 0.3 * VectorXd::Random(n-1);
  VectorXd out(n);
  ops.geodesicUpdate(out.data(), w.data(), 0.0, d.data(), n);
  double err = MaxDiff(out, w);
  printf("geodesicUpdate(w, alpha=0, d) vs w: %.2e\n", err);
  EXPECT_LT(err, 1e-13);
}

// Inverse: geodesicUpdate(w, d) then geodesicUpdate(result, -d) returns w.
// i.e., P(w^{1/2}) exp(d) P(w^{1/2}) then P(result^{1/2}) exp(-d) P(result^{1/2})
// Actually this isn't an inverse. The true inverse is:
// If w' = geodesicUpdate(w, d), then geodesicUpdate(w', -d') should give w
// for some d'. Instead test: geodesicUpdate(e, d) * geodesicUpdate(e, -d) = e*e = e.
TEST_F(SOCConeOpsTest, GeodesicForwardBackward) {
  VectorXd e = Identity();
  VectorXd d(n);
  d(0) = 0.3; d.tail(n-1) = 0.2 * VectorXd::Random(n-1);
  VectorXd fwd(n), bwd(n), product(n);
  ops.geodesicUpdate(fwd.data(), e.data(), 1.0, d.data(), n);
  ops.geodesicUpdate(bwd.data(), e.data(), -1.0, d.data(), n);
  // fwd = exp(d), bwd = exp(-d). Product should be e.
  ops.product(product.data(), fwd.data(), bwd.data(), n);
  double err = MaxDiff(product, e);
  printf("exp(d) * exp(-d) vs e: %.2e\n", err);
  EXPECT_LT(err, 1e-13);
}

// P(w)e * P(w^{-1})e = e (quadratic rep of inverses).
// P(w)e = w*w, P(w^{-1})e = w^{-1}*w^{-1}. Product = e.
TEST_F(SOCConeOpsTest, QuadRepInverseProduct) {
  VectorXd w = RandomInterior();
  double t = w(0);
  double det = t * t - w.tail(n-1).squaredNorm();
  VectorXd winv(n);
  winv(0) = t / det;
  winv.tail(n-1) = -w.tail(n-1) / det;

  VectorXd e = Identity();
  VectorXd Pwe(n), Pinve(n), product(n);
  ops.quadraticRepresentation(Pwe.data(), w.data(), e.data(), n);
  ops.quadraticRepresentation(Pinve.data(), winv.data(), e.data(), n);
  ops.product(product.data(), Pwe.data(), Pinve.data(), n);
  double err = MaxDiff(product, e);
  printf("P(w)e * P(w^-1)e vs e: %.2e\n", err);
  EXPECT_LT(err, 1e-12);
}

// Verify geodesicUpdate formula: geodesicUpdate(w, 1, d) = P(w^{1/2}) exp(d).
// Check by computing both sides independently.
TEST_F(SOCConeOpsTest, GeodesicFormulaCheck) {
  VectorXd w = RandomInterior();
  VectorXd d(n);
  d(0) = 0.2; d.tail(n-1) = 0.1 * VectorXd::Random(n-1);

  // LHS: geodesicUpdate(w, 1, d).
  VectorXd lhs(n);
  ops.geodesicUpdate(lhs.data(), w.data(), 1.0, d.data(), n);

  // RHS: P(sqrt(w)) exp(d).
  VectorXd sqrtw(n), expd(n), rhs(n);
  ops.sqrt(sqrtw.data(), w.data(), n);
  VectorXd e = Identity();
  // exp(d) = geodesicUpdate(e, 1, d) since P(e^{1/2}) = P(e) = identity.
  ops.geodesicUpdate(expd.data(), e.data(), 1.0, d.data(), n);
  ops.quadraticRepresentation(rhs.data(), sqrtw.data(), expd.data(), n);

  double err = MaxDiff(lhs, rhs);
  printf("geodesicUpdate(w,d) vs P(sqrt(w))exp(d): %.2e\n", err);
  EXPECT_LT(err, 1e-12);
}

// Verify exp(d) is in the cone interior for small d.
TEST_F(SOCConeOpsTest, ExpInterior) {
  VectorXd e = Identity();
  VectorXd d(n);
  d(0) = 0.5; d.tail(n-1) = 0.3 * VectorXd::Random(n-1);
  VectorXd expd(n);
  ops.geodesicUpdate(expd.data(), e.data(), 1.0, d.data(), n);
  double min_eig = ops.minEigenvalue(expd.data(), n);
  printf("exp(d) min_eig: %.6f (should be > 0)\n", min_eig);
  EXPECT_GT(min_eig, 0);
}

// Verify P(w)y for non-identity w against the formula 2ww'y - det(w)Ry.
TEST_F(SOCConeOpsTest, QuadRepFormulaCheck) {
  VectorXd w = RandomInterior();
  VectorXd y = RandomInterior(77);

  // ConeOps result.
  VectorXd Pwy(n);
  ops.quadraticRepresentation(Pwy.data(), w.data(), y.data(), n);

  // Manual formula: P(w)y = 2(w∘y)∘w - w²∘y.
  // Or equivalently: 2<w,y>w - det(w)*Ry where R=diag(1,-1,...,-1).
  double w0 = w(0), y0 = y(0);
  double w_dot_y = ops.dot(w.data(), y.data(), n) / 2.0;  // trace ip / 2
  double det_w = w0*w0 - w.tail(n-1).squaredNorm();
  VectorXd Ry = y;
  Ry.tail(n-1) *= -1;
  VectorXd manual = 2.0 * w_dot_y * w - det_w * Ry;

  double err = MaxDiff(Pwy, manual);
  printf("P(w)y (ConeOps vs formula): %.2e\n", err);
  EXPECT_LT(err, 1e-13);
}

}  // namespace
}  // namespace EuclideanJordanAlgebra
}  // namespace conex
