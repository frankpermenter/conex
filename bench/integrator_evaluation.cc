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
  double d[] = {0.2, -0.1, 0.15};
  double alpha = 0.3;
  int n = 3;

  double ref[3];
  exactVerletRef(z0, alpha, d, ref);

  const int REPS = 10000;

  printf("ExpCone: exact Verlet reference (10000 steps with thirdDerivContract)\n\n");
  printf("%-20s %10s %10s %10s\n", "Integrator", "time(us)", "error", "err/us");
  printf("%s\n", std::string(55, '-').c_str());

  auto bench = [&](const char* name, auto fn) {
    // Error
    double z[3], vel[3];
    std::memcpy(z, z0, sizeof(z));
    for (int i = 0; i < n; ++i) vel[i] = alpha * d[i];
    fn(z, vel);
    double err = dist(z, ref, n);

    // Time
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int r = 0; r < REPS; ++r) {
      std::memcpy(z, z0, sizeof(z));
      for (int i = 0; i < n; ++i) vel[i] = alpha * d[i];
      fn(z, vel);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / REPS;
    printf("%-20s %10.2f %10.2e %10.2e\n", name, us, err, err/us);
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

  return 0;
}
