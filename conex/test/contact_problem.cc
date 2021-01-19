#include <Eigen/Dense>
#include "conex/cone_program.h"
#include "conex/quadratic_cone_constraint.h"

namespace conex {
// Initializing: 
// Assemble(us):Sparsity(us): 52,  56, 
// i:  0, Assemble(us): 21, Factor(us): 8, Solve(us): 2, Update(us): 9, mu: 3.99e+00, d_2: 4.07e+00, d_inf: 2.59e+00, 
// i:  1, Assemble(us): 11, Factor(us): 5, Solve(us): 2, Update(us): 7, mu: 9.34e-01, d_2: 2.43e+00, d_inf: 1.67e+00, 
// i:  2, Assemble(us): 10, Factor(us): 4, Solve(us): 1, Update(us): 7, mu: 2.14e-01, d_2: 2.53e+00, d_inf: 1.79e+00, 
// i:  3, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.32e-01, d_2: 2.53e+00, d_inf: 1.79e+00, 
// i:  4, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 9.27e-02, d_2: 2.56e+00, d_inf: 1.81e+00, 
// i:  5, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 6.98e-02, d_2: 2.58e+00, d_inf: 1.82e+00, 
// i:  6, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 5.53e-02, d_2: 2.58e+00, d_inf: 1.83e+00, 
// i:  7, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 4.50e-02, d_2: 2.59e+00, d_inf: 1.83e+00, 
// i:  8, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 3.75e-02, d_2: 2.60e+00, d_inf: 1.83e+00, 
// i:  9, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 3.17e-02, d_2: 2.60e+00, d_inf: 1.84e+00, 
// i: 10, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.72e-02, d_2: 2.60e+00, d_inf: 1.84e+00, 
// i: 11, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.35e-02, d_2: 2.60e+00, d_inf: 1.84e+00, 
// i: 12, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.05e-02, d_2: 2.61e+00, d_inf: 1.84e+00, 
// i: 13, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.80e-02, d_2: 2.61e+00, d_inf: 1.84e+00, 
// i: 14, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.59e-02, d_2: 2.61e+00, d_inf: 1.84e+00, 
// i: 15, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.41e-02, d_2: 2.61e+00, d_inf: 1.84e+00, 
// i: 16, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.26e-02, d_2: 2.61e+00, d_inf: 1.85e+00, 
// i: 17, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.13e-02, d_2: 2.61e+00, d_inf: 1.85e+00, 
// i: 18, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.01e-02, d_2: 2.61e+00, d_inf: 1.85e+00, 
// i: 19, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 9.09e-03, d_2: 2.62e+00, d_inf: 1.85e+00, 
// i: 20, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 8.20e-03, d_2: 2.62e+00, d_inf: 1.85e+00, 
// i: 21, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 7.40e-03, d_2: 2.62e+00, d_inf: 1.85e+00, 
// i: 22, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 6.70e-03, d_2: 2.62e+00, d_inf: 1.85e+00, 
// i: 23, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 6.07e-03, d_2: 2.62e+00, d_inf: 1.85e+00, 
// i: 24, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 5.50e-03, d_2: 2.63e+00, d_inf: 1.86e+00, 
// i: 25, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 4.99e-03, d_2: 2.63e+00, d_inf: 1.86e+00, 
// i: 26, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 4.53e-03, d_2: 2.63e+00, d_inf: 1.86e+00, 
// i: 27, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 4.12e-03, d_2: 2.64e+00, d_inf: 1.86e+00, 
// i: 28, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 3.75e-03, d_2: 2.64e+00, d_inf: 1.87e+00, 
// i: 29, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 3.41e-03, d_2: 2.65e+00, d_inf: 1.87e+00, 
// i: 30, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 3.10e-03, d_2: 2.65e+00, d_inf: 1.88e+00, 
// i: 31, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.82e-03, d_2: 2.66e+00, d_inf: 1.88e+00, 
// i: 32, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.57e-03, d_2: 2.67e+00, d_inf: 1.88e+00, 
// i: 33, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.34e-03, d_2: 2.68e+00, d_inf: 1.89e+00, 
// i: 34, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 2.12e-03, d_2: 2.68e+00, d_inf: 1.90e+00, 
// i: 35, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.93e-03, d_2: 2.69e+00, d_inf: 1.90e+00, 
// i: 36, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.76e-03, d_2: 2.70e+00, d_inf: 1.91e+00, 
// i: 37, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.59e-03, d_2: 2.72e+00, d_inf: 1.92e+00, 
// i: 38, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.45e-03, d_2: 2.73e+00, d_inf: 1.93e+00, 
// i: 39, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.31e-03, d_2: 2.74e+00, d_inf: 1.94e+00, 
// i: 40, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.19e-03, d_2: 2.76e+00, d_inf: 1.95e+00, 
// i: 41, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 1.08e-03, d_2: 2.77e+00, d_inf: 1.96e+00, 
// i: 42, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 9.74e-04, d_2: 2.79e+00, d_inf: 1.97e+00, 
// i: 43, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 8.80e-04, d_2: 2.81e+00, d_inf: 1.98e+00, 
// i: 44, Assemble(us): 10, Factor(us): 3, Solve(us): 1, Update(us): 7, mu: 7.94e-04, d_2: 2.83e+00, d_inf: 2.00e+00, 
// i: 45, Assemble(us): 10, Factor(us): 3, Solve(us): 2, Update(us): 7, mu: 7.94e-04, d_2: 2.62e+00, d_inf: 1.85e+00, 
// i: 46, Assemble(us): 10, Factor(us): 3, Solve(us): 2, Update(us): 7, mu: 7.94e-04, d_2: 2.41e+00, d_inf: 1.70e+00, 
// i: 47, Assemble(us): 10, Factor(us): 3, Solve(us): 2, Update(us): 7, mu: 7.94e-04, d_2: 2.20e+00, d_inf: 1.56e+00, 
// i: 48, Assemble(us): 10, Factor(us): 3, Solve(us): 2, Update(us): 7, mu: 7.94e-04, d_2: 1.98e+00, d_inf: 1.40e+00, 
// i: 49, Assemble(us): 10, Factor(us): 3, Solve(us): 2, Update(us): 7, mu: 7.94e-04, d_2: 1.75e+00, d_inf: 1.24e+00, 
// Status: Solved.

Eigen::MatrixXd CostMatrix() {
  Eigen::MatrixXd Q(25, 25);
  Q << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 41.884, -1.88984, 5.54026, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8.3199, 0.505813, 0.980003, 0, -1.88984, 42.0448, 5.30951, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.187722, 8.30367, 0.939186, 0, 5.54026, 5.30951, 13.7913, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.961855, -0.957763, -4.81223, 0, 0, 0, 0, 17.5122, 0, 0, 8.3199, 0.187722, -0.961855, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17.5122, 0, 0.505813, 8.30367, -0.957763, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5.00338, 0.980003, 0.939186, -4.81223, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8.3199, 0.505813, 0.980003, 41.884, -1.88984, 5.54026, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.187722, 8.30367, 0.939186, -1.88984, 42.0448, 5.30951, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.961855, -0.957763, -4.81223, 5.54026, 5.30951, 13.7913, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17.5122, 0, 0, 8.3199, 0.187722, -0.961855, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17.5122, 0, 0.505813, 8.30367, -0.957763, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5.00338, 0.980003, 0.939186, -4.81223, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8.3199, 0.505813, 0.980003, 41.884, -1.88984, 5.54026, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.187722, 8.30367, 0.939186, -1.88984, 42.0448, 5.30951, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.961855, -0.957763, -4.81223, 5.54026, 5.30951, 13.7913, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17.5122, 0, 0, 8.3199, 0.187722, -0.961855, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17.5122, 0, 0.505813, 8.30367, -0.957763, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5.00338, 0.980003, 0.939186, -4.81223, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8.3199, 0.505813, 0.980003, 41.884, -1.88984, 5.54026, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.187722, 8.30367, 0.939186, -1.88984, 42.0448, 5.30951, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.961855, -0.957763, -4.81223, 5.54026, 5.30951, 13.7913, 0, 0, 0, 0, 8.3199, 0.187722, -0.961855, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17.5122, 0, 0, 0, 0.505813, 8.30367, -0.957763, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17.5122, 0, 0, 0.980003, 0.939186, -4.81223, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5.00338;
return Q;
}

void AddQuadraticCost(Program* prog) {
  Eigen::MatrixXd A(26, 25);
  Eigen::MatrixXd Q = CostMatrix();
  A << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;

  Eigen::MatrixXd b(26, 1);
  //   0   1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
  b << 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  std::vector<int> z_indices{
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  13,
  14,
  15,
  16,
  17,
  18,
  19,
  20,
  21,
  22,
  23,
  24};

prog->AddConstraint(conex::QuadraticConstraint(Q, A, b), z_indices);
}

void AddConstraint1(Program* prog) {
Eigen::MatrixXd Ai(3, 3);
Eigen::MatrixXd bi(3, 1);
Ai << 0, 0, -1, -1, 0, 0, 0, -1, 0;
bi << 0, 0, 0;
std::vector<int> x_indices{0, 1, 2};
prog->AddConstraint(conex::QuadraticConstraint(Ai, bi), x_indices);
}


void AddConstraint2(Program* prog) {
Eigen::MatrixXd Ai(3, 3);
Eigen::MatrixXd bi(3, 1);
Ai << 0, 0, -1, -1, 0, 0, 0, -1, 0;
bi << 0, 0, 0;
std::vector<int> x_indices{ 3, 4, 5};
prog->AddConstraint(conex::QuadraticConstraint(Ai, bi), x_indices);
}


void AddConstraint3(Program* prog) {
Eigen::MatrixXd Ai(3, 3);
Eigen::MatrixXd bi(3, 1);
Ai << 0, 0, -1, -1, 0, 0, 0, -1, 0;
bi << 0, 0, 0;
std::vector<int> x_indices{6, 7, 8};
prog->AddConstraint(conex::QuadraticConstraint(Ai, bi), x_indices);
}

void AddConstraint4(Program* prog) {
Eigen::MatrixXd Ai(3, 3);
Eigen::MatrixXd bi(3, 1);
Ai << 0, 0, -1, -1, 0, 0, 0, -1, 0;
bi << 0, 0, 0;
std::vector<int> x_indices{9, 10, 11} ;
prog->AddConstraint(conex::QuadraticConstraint(Ai, bi), x_indices);
}

void AddConstraint5(Program* prog) {
Eigen::MatrixXd Ai(3, 3);
Eigen::MatrixXd bi(3, 1);
Ai << 0, 0, -1, -1, 0, 0, 0, -1, 0;
bi << 0, 0, 0;
std::vector<int> x_indices{12, 13, 14};
prog->AddConstraint(conex::QuadraticConstraint(Ai, bi), x_indices);

}

void AddConstraint6(Program* prog) {
Eigen::MatrixXd Ai(3, 3);
Eigen::MatrixXd bi(3, 1);
Ai << 0, 0, -1, -1, 0, 0, 0, -1, 0;
bi << 0, 0, 0;
std::vector<int> x_indices{15, 16, 17};
prog->AddConstraint(conex::QuadraticConstraint(Ai, bi), x_indices);
}


void AddConstraint7(Program* prog) {
Eigen::MatrixXd Ai(3, 3);
Eigen::MatrixXd bi(3, 1);
Ai << 0, 0, -1, -1, 0, 0, 0, -1, 0;
bi << 0, 0, 0;
std::vector<int> x_indices{ 18, 19, 20};
prog->AddConstraint(conex::QuadraticConstraint(Ai, bi), x_indices);
}


void AddConstraint8(Program* prog) {
Eigen::MatrixXd Ai(3, 3);
Eigen::MatrixXd bi(3, 1);
Ai << 0, 0, -1, -1, 0, 0, 0, -1, 0;
bi << 0, 0, 0;
std::vector<int> x_indices{21, 22, 23};
prog->AddConstraint(conex::QuadraticConstraint(Ai, bi), x_indices);
}

void Solve() {

conex::Program prog(25);



Eigen::VectorXd b(25, 1);
  b << -0.173869,
    -0.166628,
  0.0809795,
         -0,
         -0,
  0.0891557,
  -0.173869,
  -0.166628,
  0.0809795,
         -0,
         -0,
  0.0891557,
  -0.173869,
  -0.166628,
  0.0809795,
         -0,
         -0,
  0.0891557,
  -0.173869,
  -0.166628,
  0.0809795,
         -0,
         -0,
  0.0891557,
         -1;

  Eigen::VectorXd x(25);

  AddQuadraticCost(&prog);
  AddConstraint1(&prog);
  AddConstraint2(&prog);
  AddConstraint3(&prog);
  AddConstraint4(&prog);
  // AddConstraint5(&prog);
  // AddConstraint6(&prog);
  // AddConstraint7(&prog);
  // AddConstraint8(&prog);

  conex::SolverConfiguration config;
  config.prepare_dual_variables = 0;
  config.max_iterations = 50;
  config.divergence_upper_bound = 1;
  config.final_centering_steps = 5;
  config.inv_sqrt_mu_max = 100;
  conex::Solve(b, prog, config, x.data());

}

}

int main() {
  conex::Solve();
}
