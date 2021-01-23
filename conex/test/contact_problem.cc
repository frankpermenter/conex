#include "conex/cone_program.h"
#include <Eigen/Dense>
using Eigen::MatrixXd;
namespace conex {
// Initializing:
// Assemble(us):Sparsity(us): 52,  56,
// i:  0, Assemble(us): 21, Factor(us): 8, Solve(us): 2, Update(us): 9,
// mu: 3.99e+00, d_2: 4.07e+00, d_inf: 2.59e+00, i:  1, Assemble(us): 11,
// Factor(us): 5, Solve(us): 2, Update(us): 7, mu: 9.34e-01, d_2: 2.43e+00,
// d_inf: 1.67e+00, i:  2, Assemble(us): 10, Factor(us): 4, Solve(us): 1,
// Update(us): 7, mu: 2.14e-01, d_2: 2.53e+00, d_inf: 1.79e+00, i:  3,
// Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.32e-01,
// d_2: 2.53e+00, d_inf: 1.79e+00, i:  4, Assemble(us): 10, Factor(us): 3,
// Solve(us): 1, Update(us): 7, mu: 9.27e-02, d_2: 2.56e+00, d_inf: 1.81e+00, i:
// 5, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7,
// mu: 6.98e-02, d_2: 2.58e+00, d_inf: 1.82e+00, i:  6, Assemble(us): 10,
// Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 5.53e-02, d_2: 2.58e+00,
// d_inf: 1.83e+00, i:  7, Assemble(us): 10, Factor(us): 3, Solve(us): 1,
// Update(us): 7, mu: 4.50e-02, d_2: 2.59e+00, d_inf: 1.83e+00, i:  8,
// Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 3.75e-02,
// d_2: 2.60e+00, d_inf: 1.83e+00, i:  9, Assemble(us): 10, Factor(us): 3,
// Solve(us): 1, Update(us): 7, mu: 3.17e-02, d_2: 2.60e+00, d_inf: 1.84e+00, i:
// 10, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7,
// mu: 2.72e-02, d_2: 2.60e+00, d_inf: 1.84e+00, i: 11, Assemble(us): 10,
// Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.35e-02, d_2: 2.60e+00,
// d_inf: 1.84e+00, i: 12, Assemble(us): 10, Factor(us): 3, Solve(us): 1,
// Update(us): 7, mu: 2.05e-02, d_2: 2.61e+00, d_inf: 1.84e+00, i: 13,
// Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.80e-02,
// d_2: 2.61e+00, d_inf: 1.84e+00, i: 14, Assemble(us): 10, Factor(us): 3,
// Solve(us): 1, Update(us): 7, mu: 1.59e-02, d_2: 2.61e+00, d_inf: 1.84e+00, i:
// 15, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7,
// mu: 1.41e-02, d_2: 2.61e+00, d_inf: 1.84e+00, i: 16, Assemble(us): 10,
// Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.26e-02, d_2: 2.61e+00,
// d_inf: 1.85e+00, i: 17, Assemble(us): 10, Factor(us): 3, Solve(us): 1,
// Update(us): 7, mu: 1.13e-02, d_2: 2.61e+00, d_inf: 1.85e+00, i: 18,
// Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.01e-02,
// d_2: 2.61e+00, d_inf: 1.85e+00, i: 19, Assemble(us): 10, Factor(us): 3,
// Solve(us): 1, Update(us): 7, mu: 9.09e-03, d_2: 2.62e+00, d_inf: 1.85e+00, i:
// 20, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7,
// mu: 8.20e-03, d_2: 2.62e+00, d_inf: 1.85e+00, i: 21, Assemble(us): 10,
// Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 7.40e-03, d_2: 2.62e+00,
// d_inf: 1.85e+00, i: 22, Assemble(us): 10, Factor(us): 3, Solve(us): 1,
// Update(us): 7, mu: 6.70e-03, d_2: 2.62e+00, d_inf: 1.85e+00, i: 23,
// Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 6.07e-03,
// d_2: 2.62e+00, d_inf: 1.85e+00, i: 24, Assemble(us): 10, Factor(us): 3,
// Solve(us): 1, Update(us): 7, mu: 5.50e-03, d_2: 2.63e+00, d_inf: 1.86e+00, i:
// 25, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7,
// mu: 4.99e-03, d_2: 2.63e+00, d_inf: 1.86e+00, i: 26, Assemble(us): 10,
// Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 4.53e-03, d_2: 2.63e+00,
// d_inf: 1.86e+00, i: 27, Assemble(us): 10, Factor(us): 3, Solve(us): 1,
// Update(us): 7, mu: 4.12e-03, d_2: 2.64e+00, d_inf: 1.86e+00, i: 28,
// Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 3.75e-03,
// d_2: 2.64e+00, d_inf: 1.87e+00, i: 29, Assemble(us): 10, Factor(us): 3,
// Solve(us): 1, Update(us): 7, mu: 3.41e-03, d_2: 2.65e+00, d_inf: 1.87e+00, i:
// 30, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7,
// mu: 3.10e-03, d_2: 2.65e+00, d_inf: 1.88e+00, i: 31, Assemble(us): 10,
// Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.82e-03, d_2: 2.66e+00,
// d_inf: 1.88e+00, i: 32, Assemble(us): 10, Factor(us): 3, Solve(us): 1,
// Update(us): 7, mu: 2.57e-03, d_2: 2.67e+00, d_inf: 1.88e+00, i: 33,
// Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.34e-03,
// d_2: 2.68e+00, d_inf: 1.89e+00, i: 34, Assemble(us): 10, Factor(us): 3,
// Solve(us): 1, Update(us): 7, mu: 2.12e-03, d_2: 2.68e+00, d_inf: 1.90e+00, i:
// 35, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7,
// mu: 1.93e-03, d_2: 2.69e+00, d_inf: 1.90e+00, i: 36, Assemble(us): 10,
// Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.76e-03, d_2: 2.70e+00,
// d_inf: 1.91e+00, i: 37, Assemble(us): 10, Factor(us): 3, Solve(us): 1,
// Update(us): 7, mu: 1.59e-03, d_2: 2.72e+00, d_inf: 1.92e+00, i: 38,
// Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.45e-03,
// d_2: 2.73e+00, d_inf: 1.93e+00, i: 39, Assemble(us): 10, Factor(us): 3,
// Solve(us): 1, Update(us): 7, mu: 1.31e-03, d_2: 2.74e+00, d_inf: 1.94e+00, i:
// 40, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7,
// mu: 1.19e-03, d_2: 2.76e+00, d_inf: 1.95e+00, i: 41, Assemble(us): 10,
// Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.08e-03, d_2: 2.77e+00,
// d_inf: 1.96e+00, i: 42, Assemble(us): 10, Factor(us): 3, Solve(us): 1,
// Update(us): 7, mu: 9.74e-04, d_2: 2.79e+00, d_inf: 1.97e+00, i: 43,
// Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 8.80e-04,
// d_2: 2.81e+00, d_inf: 1.98e+00, i: 44, Assemble(us): 10, Factor(us): 3,
// Solve(us): 1, Update(us): 7, mu: 7.94e-04, d_2: 2.83e+00, d_inf: 2.00e+00, i:
// 45, Assemble(us): 10, Factor(us): 3, Solve(us): 2, Update(us): 7,
// mu: 7.94e-04, d_2: 2.62e+00, d_inf: 1.85e+00, i: 46, Assemble(us): 10,
// Factor(us): 3, Solve(us): 2, Update(us): 7, mu: 7.94e-04, d_2: 2.41e+00,
// d_inf: 1.70e+00, i: 47, Assemble(us): 10, Factor(us): 3, Solve(us): 2,
// Update(us): 7, mu: 7.94e-04, d_2: 2.20e+00, d_inf: 1.56e+00, i: 48,
// Assemble(us): 10, Factor(us): 3, Solve(us): 2, Update(us): 7, mu: 7.94e-04,
// d_2: 1.98e+00, d_inf: 1.40e+00, i: 49, Assemble(us): 10, Factor(us): 3,
// Solve(us): 2, Update(us): 7, mu: 7.94e-04, d_2: 1.75e+00, d_inf: 1.24e+00,
// Status: Solved.

void AddQuadraticCost(Program* prog);
void AddConstraint1(Program* prog);
void AddConstraint2(Program* prog);
void AddConstraint3(Program* prog);
void AddConstraint4(Program* prog);
void AddConstraint5(Program* prog);
void AddConstraint6(Program* prog);
void AddConstraint7(Program* prog);
void AddConstraint8(Program* prog);

void Solve() {
  conex::Program prog(25);

  Eigen::VectorXd b(25, 1);
  b << -0.173869, -0.166628, 0.0809795, -0, -0, 0.0891557, -0.173869, -0.166628,
      0.0809795, -0, -0, 0.0891557, -0.173869, -0.166628, 0.0809795, -0, -0,
      0.0891557, -0.173869, -0.166628, 0.0809795, -0, -0, 0.0891557, -1;

  Eigen::VectorXd x(25);

  AddQuadraticCost(&prog);
  AddConstraint1(&prog);
  AddConstraint2(&prog);
  AddConstraint3(&prog);
  AddConstraint4(&prog);
  AddConstraint5(&prog);
  AddConstraint6(&prog);
  AddConstraint7(&prog);
  AddConstraint8(&prog);

  auto b_feas = GetFeasibleObjective(&prog);
  double theta = 1;

  b = theta * b + (1 - theta) * b_feas;

  conex::SolverConfiguration config;
  config.prepare_dual_variables = 0;
  config.max_iterations = 45;
  config.divergence_upper_bound = 1000;
  config.final_centering_steps = 5;
  config.maximum_mu = 1000000;
  config.inv_sqrt_mu_max = 4000000;
  config.maximum_dinf = std::sqrt(2);
  conex::Solve(b, prog, config, x.data());
}

}  // namespace conex

int main() { conex::Solve(); }
