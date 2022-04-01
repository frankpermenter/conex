#pragma once
#include <vector>

#include <Eigen/Dense>

#include "conex/constraint.h"
#include "conex/newton_step.h"
#include "conex/supernodal_cholesky_data.h"

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

// Manages the transfer of clique submatrix to supernodal data structure.
// The SetDenseData triggers an update the submatrix which
// is store in an Eigen::Map.
class SupernodalAssemblerBase {
 public:
  SupernodalAssemblerBase(int n) { SetNumberOfVariables(n); }
  SupernodalAssemblerBase(){};

  // Entries of diagonal block to update
  void BindDiagonalBlock(const DiagonalBlock* data);
  // Entries of off diagonal block to update
  void BindOffDiagonalBlock(const OffDiagonalBlock* data);
  void Reset() {
    diag.clear();
    off_diag.clear();
    scatter_block.clear();
    direct_update = false;
  }

  void UpdateBlocks();
  virtual void SetDenseData() = 0;

  int NumberOfVariables() { return num_variables_; };
  void SetNumberOfVariables(int n) {
    num_variables_ = n;
    submatrix_data_.m_ = n;
  };

  WorkspaceSchurComplement submatrix_data_;
  SchurComplementSystem* GetWorkspace() { return &submatrix_data_; }

 protected:
  double GetCoeff(int i, int j);

  void Increment(const int* r, int sizer, const int* c, int sizec,
                 Eigen::Map<Eigen::MatrixXd>* data);
  void Set(const int* r, int sizer, const int* c, int sizec,
           Eigen::Map<Eigen::MatrixXd>* data);

  void IncrementLowerTri(const int* r, int sizer, const int* c, int sizec,
                         Eigen::Map<Eigen::MatrixXd>* data);
  void SetLowerTri(const int* r, int sizer, const int* c, int sizec,
                   Eigen::Map<Eigen::MatrixXd>* data);

  void SetDiagonalBlock(const std::vector<int>& r,
                        Eigen::Map<Eigen::MatrixXd>* data);

  void Scatter(const int* r, int sizer, const int* c, int sizec, double** data);
  int num_variables_;

  bool direct_update = false;
  std::vector<DiagonalBlock> diag;
  std::vector<OffDiagonalBlock> off_diag;
  std::vector<OffDiagonalBlock> scatter_block;
  virtual ~SupernodalAssemblerBase(){};
};

class SupernodalAssembler : public SupernodalAssemblerBase {
 public:
  SupernodalAssembler(int num_variables, Constraint* W)
      : SupernodalAssemblerBase(num_variables) {
    workspace_ = W;
    assert(W);
  }
  virtual void SetDenseData() {
    if (!submatrix_data_.initialized) {
#if CONEX_DEBUG_MESSAGES
      std::cerr << "Performing self initialization of SupernodalAssembler. Did "
                   "you forget to initialize workspace?";
#endif
      Workspace workspace = Workspace(&submatrix_data_);
      memory_.resize(SizeOf(workspace));
      Initialize(&workspace, memory_.data());
    }

    if (workspace_) {
      ConstructSchurComplementSystem(workspace_, true, &submatrix_data_);
    } else {
      throw std::runtime_error("Supernodal assembler data source is not set.");
    }
  }

  SupernodalAssembler(){};
  Constraint* workspace_ = NULL;
  Eigen::VectorXd memory_;
};

class SupernodalAssemblerStatic : public SupernodalAssemblerBase {
 public:
  SupernodalAssemblerStatic(const Eigen::MatrixXd& A)
      : SupernodalAssemblerBase(A.rows()), A_(A) {
    if (A_.rows() != A.cols()) {
      throw std::runtime_error("Static assembler requires square matrix");
    }
  }

  int UpdateMatrix(double value, int row, int col) {
    CONEX_RETURN_ON_FAIL(row < A_.rows() && col < A_.cols(),
                         "Indices are out of bounds.");

    A_(row, col) = value;
    return CONEX_SUCCESS;
  }

  virtual void SetDenseData() override {
    if (!submatrix_data_.initialized) {
#if CONEX_DEBUG_MESSAGES
      std::cerr
          << "Performing self initialization of SupernodalAssemblerStatic. Did "
             "you forget to initialize workspace?";
#endif
      Workspace workspace = Workspace(&submatrix_data_);
      memory_.resize(SizeOf(workspace));
      Initialize(&workspace, memory_.data());
    }
    submatrix_data_.setZero();
    submatrix_data_.G = A_;
  }
  Eigen::MatrixXd A_;
  Eigen::VectorXd memory_;
};

}  // namespace conex
