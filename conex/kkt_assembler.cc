#include "conex/kkt_assembler.h"
#include "conex/debug_macros.h"
#include "conex/newton_step.h"

namespace conex {
using T = LinearKKTAssemblerBase;
using Eigen::MatrixXd;
using std::vector;

namespace {

template <typename T>
vector<T> InitVector(const T* data, int N) {
  vector<T> y;
  for (int i = 0; i < N; i++) {
    y.push_back(data[i]);
  }
  return y;
}

}  // namespace

void T::Increment(const int* r, int sizer, const int* c, int sizec,
                  Eigen::Map<MatrixXd>* data) {
  for (int i = 0; i < sizer; i++) {
    for (int j = 0; j < sizec; j++) {
      (*data)(i, j) += GetCoeff(*(r + i), *(c + j));
    }
  }
}

void T::IncrementLowerTri(const int* r, int sizer, const int* c, int sizec,
                          Eigen::Map<MatrixXd>* data) {
  for (int j = 0; j < sizec; j++) {
    for (int i = j; i < sizer; i++) {
      (*data)(i, j) += GetCoeff(*(r + i), *(c + j));
    }
  }
}

void T::Set(const int* r, int sizer, const int* c, int sizec,
            Eigen::Map<MatrixXd>* data) {
  for (int j = 0; j < sizec; j++) {
    for (int i = 0; i < sizer; i++) {
      (*data)(i, j) = GetCoeff(*(r + i), *(c + j));
    }
  }
}

void T::SetLowerTri(const int* r, int sizer, const int* c, int sizec,
                    Eigen::Map<MatrixXd>* data) {
  for (int j = 0; j < sizec; j++) {
    for (int i = j; i < sizer; i++) {
      (*data)(i, j) = GetCoeff(*(r + i), *(c + j));
    }
  }
}

double T::GetCoeff(int i, int j) {
  bool fill_in = (i < 0) || (j < 0);
  if (fill_in) {
    // Fill in is
    return 0;
  }
  if (i >= j) {
    return schur_complement_data.G(i, j);
  } else {
    return schur_complement_data.G(j, i);
  }
}

void T::BindDiagonalBlock(const DiagonalBlock* data) {
  if (direct_update) {
    throw std::runtime_error("Cannot bind multiple diagonal blocks");
  }
  diag.push_back(*data);
  int m = NumberOfVariables();

  if (m == data->num_vars) {
    direct_update = true;
    for (int i = 1; i < data->num_vars; i++) {
      if (*(data->var_data + i) <= *(data->var_data + i - 1)) {
        direct_update = false;
        break;
      }
    }
  }

  if (direct_update) {
    new (&schur_complement_data.G)
        Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>(data->data, m, m);
  }
}

void T::BindOffDiagonalBlock(const OffDiagonalBlock* data) {
  if (direct_update) {
    throw std::runtime_error("Cannot bind multiple off-diagonal blocks");
  }
  if (data->stride != -1) {
    off_diag.push_back(*data);
  } else {
    scatter_block.push_back(*data);
  }
}

void T::Scatter(const int* r, int sizer, const int* c, int sizec,
                double** data) {
  int cnt = 0;
  for (int j = 0; j < sizec; j++) {
    for (int i = j; i < sizer; i++) {
      *data[cnt++] += GetCoeff(*(r + i), *(c + j));
    }
  }
}

void T::UpdateBlocks() {
  SetDenseData();

  if (direct_update) {
    return;
  }

  for (const auto& d : diag) {
    Eigen::Map<MatrixXd> data(d.data, d.num_vars, d.num_vars);
    if (d.assign) {
      SetLowerTri(d.var_data, d.num_vars, d.var_data, d.num_vars, &data);
    } else {
      IncrementLowerTri(d.var_data, d.num_vars, d.var_data, d.num_vars, &data);
    }
  }

  for (const auto& d : off_diag) {
    Eigen::Map<MatrixXd> data(d.data, d.num_rows, d.num_cols);
    if (d.assign) {
      Set(d.row_data, d.num_rows, d.col_data, d.num_cols, &data);
    } else {
      Increment(d.row_data, d.num_rows, d.col_data, d.num_cols, &data);
    }
  }

  for (const auto& d : scatter_block) {
    Scatter(d.row_data, d.num_rows, d.col_data, d.num_cols, d.data_pointers);
  }
}

}  // namespace conex
