#pragma once
#include <vector>

#include "conex/constraint.h"
#include "conex/constraint_interface.h"
#include "conex/newton_step.h"
#include "conex/supernodal_assembler_base.h"
#include "conex/supernodal_cholesky_data.h"
#include <Eigen/Dense>

namespace conex {

inline OffDiagonalBlock BuildBlock(const std::vector<int>* r,
                                   const std::vector<int>* c,
                                   double* matrix_data) {
  OffDiagonalBlock block;
  block.num_rows = r->size();
  block.row_data = r->data();
  block.num_cols = c->size();
  block.col_data = c->data();
  block.stride = 1;
  block.data = matrix_data;
  block.assign = 1;
  return block;
}

inline DiagonalBlock BuildBlock(const std::vector<int>* r,
                                double* matrix_data) {
  DiagonalBlock block;
  block.num_vars = r->size();
  block.var_data = r->data();
  block.stride = 1;
  block.data = matrix_data;
  block.assign = 1;
  return block;
}

inline OffDiagonalBlock BuildBlock(const std::vector<int>* r,
                                   const std::vector<int>* c,
                                   std::vector<double*>* mat) {
  OffDiagonalBlock block;
  block.num_rows = r->size();
  block.row_data = r->data();
  block.num_cols = c->size();
  block.col_data = c->data();
  block.stride = -1;
  block.data_pointers = mat->data();
  block.assign = 0;
  return block;
}

}  // namespace conex
