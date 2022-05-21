#pragma once
#include <vector>

#include "conex/constraint.h"
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

class SupernodalAssembler : public SupernodalAssemblerBase {
 public:
  SupernodalAssembler(const std::vector<int>& variables, Constraint* W)
      : SupernodalAssemblerBase(variables, 0 /*no private variables*/) {
    workspace_ = W;
    assert(W);
  }

  virtual bool is_dynamic() const override { return true; }
  virtual bool is_positive_definite() const override { return true; }
  virtual int number_of_auxiliary_variables() const override { return 0; }
  Constraint* constraint() { return workspace_; }

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
  SupernodalAssemblerStatic(const Eigen::MatrixXd& A,
                            const std::vector<int>& variables)
      : SupernodalAssemblerBase(variables, 0 /*no private variables*/), A_(A) {
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

class SupernodalAssemblerEqualities final : public SupernodalAssemblerBase {
 public:
  SupernodalAssemblerEqualities(const Eigen::MatrixXd& A,
                                const Eigen::VectorXd& b,
                                const std::vector<int>& primal_variables,
                                const std::vector<int>& dual_variables);

  const std::vector<int>& dual_variables() const { return dual_variables_; }
  int UpdateMatrix(double value, int row, int col) {
    CONEX_RETURN_ON_FAIL(row < A_.rows() && col < A_.cols(),
                         "Indices are out of bounds.");

    A_(row, col) = value;
    return CONEX_SUCCESS;
  }

  const Eigen::VectorXd& affine_term() const { return b_; }
  const Eigen::MatrixXd& constraint_matrix() const { return A_; }

  virtual bool is_dynamic() const override { return false; }
  virtual bool is_positive_definite() const override { return false; }

  virtual void SetDenseData() override {
    if (!submatrix_data_.initialized) {
#if CONEX_DEBUG_MESSAGES
      std::cerr << "Performing self initialization of "
                   "SupernodalAssemblerStatic. Did "
                   "you forget to initialize workspace?";
#endif
      Workspace workspace = Workspace(&submatrix_data_);
      memory_.resize(SizeOf(workspace));
      Initialize(&workspace, memory_.data());
    }
    submatrix_data_.setZero();
    submatrix_data_.G.bottomLeftCorner(A_.rows(), A_.cols()) = A_;
    submatrix_data_.AQc.bottomRows(A_.rows()) = b_;
  }

 private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  Eigen::VectorXd memory_;
  std::vector<int> dual_variables_;
};

}  // namespace conex
