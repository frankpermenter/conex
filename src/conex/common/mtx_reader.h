#pragma once
#include <string>
#include <utility>

#include <Eigen/Sparse>

#include "conex/common/model.h"

namespace conex {

struct MTXInfo {
  std::string name;
  int rows = 0;
  int cols = 0;
  int nnz = 0;
  bool is_quadratic = false;   // true if the matrix was square
  bool was_transposed = false; // true if original was wide (cols > rows)
};

// Read a MatrixMarket file and build a Model.
//
// Square matrices are treated as quadratic cost: min (1/2) x^T Q x
// where Q = A + A^T + n*I (symmetrized, diagonal-shifted for SPD).
//
// Rectangular matrices are treated as least-squares: min ||Ax||^2
// (wide matrices are transposed to tall before adding as a constraint).
//
// If randomize is true, nonzero values are replaced with random values.
std::pair<Model, MTXInfo> ReadMTX(const std::string& path,
                                    bool randomize = false);

}  // namespace conex
