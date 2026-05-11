// Profile with exact Verlet reference for exp cone.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <Eigen/Dense>
#include "conex/common/barrier_integrators.h"
#include "conex/common/exp_cone_ops.h"

using conex::EuclideanJordanAlgebra::ExpConeOps;
using Ops = conex::EuclideanJordanAlgebra::BarrierConeOperations;
using TDOps = conex::EuclideanJordanAlgebra::BarrierConeOpsThirdDeriv;

static void exactVerletRef(const double* z0, double alpha, const double* d,
                            double* out, int steps = 10000) {
  ExpConeOps ops;
  double pos[3] = {z0[0], z0[1], z0[2]};
  double vel[3] = {alpha*d[0], alpha*d[1], alpha*d[2]};
  double dt = 1.0 / steps;
  for (int s = 0; s < steps; ++s) {
    auto kick = [&]() {
      double T[3], H[9];
      ops.thirdDerivContract(T, pos, vel, 3);
      ops.hessian(H, pos, 3);
      Eigen::Map<Eigen::MatrixXd> Hm(H, 3, 3);
      Eigen::Map<Eigen::VectorXd> Tv(T, 3);
      Eigen::Vector3d a = Hm.ldlt().solve(-0.5 * Tv);
      for (int i = 0; i < 3; ++i) vel[i] += 0.5 * dt * a(i);
    };
    kick();
    for (int i = 0; i < 3; ++i) pos[i] += dt * vel[i];
    kick();
  }
  for (int i = 0; i < 3; ++i) out[i] = pos[i];
}

static double dist(const double* a, const double* b, int n) {
  double s = 0;
  for (int i = 0; i < n; ++i) s += (a[i]-b[i])*(a[i]-b[i]);
  return std::sqrt(s);
}

int main() {
  ExpConeOps ops;
  double z0[] = {0.1, 1.0, 2.5};
  double d_raw[] = {0.2, -0.1, 0.15};
  int n = 3;

  // Scale direction so ||h*v||_H = target_norm.
  auto run_at_norm = [&](double target_norm) {
    // Compute ||d_raw||_H at z0.
    double Hd[3];
    ops.hessianProduct(Hd, z0, d_raw, n);
    double d_norm = 0;
    for (int i = 0; i < n; ++i) d_norm += d_raw[i] * Hd[i];
    d_norm = std::sqrt(d_norm);
    double alpha = target_norm / d_norm;  // so ||alpha*d||_H = target_norm

    double d[3];
    for (int i = 0; i < n; ++i) d[i] = alpha * d_raw[i];

    double ref[3];
    exactVerletRef(z0, 1.0, d, ref);  // alpha already baked into d

    const int REPS = 10000;

    printf("ExpCone: ||h*v||_H = %.2f (exact Verlet reference, 10000 steps)\n\n",
           target_norm);
    printf("%-20s %10s %10s %10s\n", "Integrator", "time(us)", "error", "err/us");
    printf("%s\n", std::string(55, '-').c_str());

    auto bench = [&](const char* name, auto fn) {
      double z[3], vel[3];
      std::memcpy(z, z0, sizeof(z));
      for (int i = 0; i < n; ++i) vel[i] = d[i];
      fn(z, vel);
      double err = dist(z, ref, n);

      auto t0 = std::chrono::high_resolution_clock::now();
      for (int r = 0; r < REPS; ++r) {
        std::memcpy(z, z0, sizeof(z));
        for (int i = 0; i < n; ++i) vel[i] = d[i];
        fn(z, vel);
      }
      auto t1 = std::chrono::high_resolution_clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / REPS;
      printf("%-20s %10.2f %10.2e %10.2e\n", name, us, err, err / std::max(us, 1e-6));
    };

    bench("Verlet (1 step)", [&](double* z, double* vel) {
      double target[3];
      for (int i = 0; i < 3; ++i) target[i] = z[i] + vel[i];
      ops.geodesicStepTarget(z, 1.0, target, 3);
    });

    bench("Symmetric", [&](double* z, double* vel) {
      conex::symmetricStep(&ops, z, vel, 1.0, 3);
    });

    bench("Primal midpoint", [&](double* z, double* vel) {
      conex::primalMidpointStep(&ops, z, vel, 1.0, 3);
    });

    bench("Dual midpoint", [&](double* z, double* vel) {
      conex::dualMidpointStep(&ops, z, vel, 1.0, 3);
    });

    bench("Y4-symmetric", [&](double* z, double* vel) {
      conex::yoshida4Step(&ops, z, vel, 1.0, 3);
    });

    bench("Y4-primal", [&](double* z, double* vel) {
      conex::yoshida4PrimalStep(&ops, z, vel, 1.0, 3);
    });

    bench("Y4-dual", [&](double* z, double* vel) {
      conex::yoshida4DualStep(&ops, z, vel, 1.0, 3);
    });

    bench("Euler", [&](double* z, double* vel) {
      for (int i = 0; i < 3; ++i) z[i] += vel[i];
    });

    printf("\n");
  };

  run_at_norm(0.064);  // original test
  run_at_norm(0.5);
  run_at_norm(1.0);
  run_at_norm(1.5);

  return 0;
}
