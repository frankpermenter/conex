// MPS file reader: converts standard LP benchmark format to Problem.
//
// Supports fixed-format MPS:
//   ROWS, COLUMNS, RHS, RANGES (ignored), BOUNDS, ENDATA
//
// Row types: N (objective), L (<=), G (>=), E (=).
// Bounds: LO, UP, FX, FR, MI, BV (BV treated as continuous).
//
// Usage:
//   auto [problem, info] = ReadMPS("problem.mps");

#pragma once
#include <string>
#include <Eigen/Sparse>
#include "conex/common/problem.h"

namespace conex {

struct MPSInfo {
  std::string name;
  int num_variables = 0;
  int num_le_rows = 0;
  int num_ge_rows = 0;
  int num_eq_rows = 0;
  int num_free_rows = 0;
  bool has_bounds = false;
};

// Read an MPS file and return a Problem + metadata.
// Converts to our convention:
//   LE rows (Ax <= b): stored as -Ax + b >= 0
//   GE rows (Ax >= b): stored as  Ax - b >= 0
//   EQ rows (Ax = b):  AddEqualityConstraint
//   Objective (N row): SetLinearCost
//   Bounds: converted to additional inequality constraints
std::pair<Problem, MPSInfo> ReadMPS(const std::string& filename);

}  // namespace conex
