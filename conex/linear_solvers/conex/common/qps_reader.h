// QPS/MPS format reader: converts QP benchmarks to Problem.
//
// Standard QPS format:
//   min  c'x + (1/2) x'Qx
//   s.t. Ax {=, <=, >=} b    (from ROWS, COLUMNS, RHS)
//        l <= x <= u          (from BOUNDS; default l=0, u=+inf)
//
// Sections: NAME, ROWS, COLUMNS, RHS, RANGES, BOUNDS, QUADOBJ, ENDATA.
//
// Row types: N (objective), E (equality), L (<=), G (>=).
// Bound types: LO, UP, FX (fixed), FR (free), MI (minus infinity),
//              BV (binary), PL (plus infinity).
//
// Mapping to Problem:
//   E rows  → AddEqualityConstraint(C, d, vars)
//   L rows  → AddLinearConstraint(A, b, vars) with  b - Ax >= 0
//   G rows  → AddLinearConstraint(A, b, vars) with  Ax - b >= 0
//   Bounds  → AddLinearConstraint for box constraints
//   QUADOBJ → AddQuadraticCost(Q, vars)
//   N row   → SetLinearCost(c)

#pragma once
#include <string>
#include <vector>
#include "conex/common/problem.h"

namespace conex {

struct QPSInfo {
  std::string name;
  int num_variables = 0;
  int num_equality_rows = 0;
  int num_inequality_rows = 0;
  int num_quadratic_entries = 0;
  int num_bounded_vars = 0;  // variables with non-default bounds
  double objective_constant = 0;  // c_0 from RHS entry for the N row
};

std::pair<Problem, QPSInfo> ReadQPS(const std::string& filename);

}  // namespace conex
