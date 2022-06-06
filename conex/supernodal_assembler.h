#pragma once
#include <vector>

#include "conex/constraint.h"
#include "conex/constraint_interface.h"
#include "conex/newton_step.h"
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

// Manages the transfer of clique submatrix to supernodal data structure.
// The SetDenseData triggers an update the submatrix which
// is store in an Eigen::Map.
class SupernodalAssemblerBase : public ConstraintBase {
 public:
  SupernodalAssemblerBase(const std::vector<int>& shared_variables) {
    SetPrimalVariables(shared_variables);
  }
  SupernodalAssemblerBase(const std::vector<int>& primal_variables,
                          const std::vector<int>& dual_variables) {
    std::vector<int> variables = primal_variables;
    variables.insert(variables.end(), dual_variables.begin(),
                     dual_variables.end());
    SetPrimalVariables(primal_variables);
    SetDualVariables(dual_variables);
  }
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

  void accept(Visitor*) override {
    throw std::runtime_error("Not implemented.");
  }

  int number_of_variables() const override {
    return primal_variables().size() + dual_variables().size();
  }

  virtual bool is_dynamic() const { return false; }
  virtual bool is_positive_definite() const { return true; }
  virtual std::vector<int> variables() const {
    std::vector<int> variables = primal_variables_;
    variables.insert(variables.end(), dual_variables_.begin(),
                     dual_variables_.end());
    return variables;
  }
  virtual const std::vector<int>& primal_variables() const {
    return primal_variables_;
  }
  virtual const std::vector<int>& dual_variables() const {
    return dual_variables_;
  }

  void UpdateBlocks();
  virtual void SetDenseData() = 0;

  Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> PrimalSubvector(
      const Eigen::MatrixXd& x) const {
    ysegment.resize(primal_variables_.size(), 1);
    Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> z(ysegment.data(),
                                                  ysegment.size(), 1);
    int cnt = 0;
    for (auto i : primal_variables_) {
      z(cnt++) = x(i);
    }
    return z;
  }

  void SetPrimalVariables(const std::vector<int>& variables) {
    primal_variables_ = variables;
    num_variables_ = primal_variables_.size() + dual_variables_.size();
    submatrix_data_.m_ = number_of_variables();
  };
  void SetDualVariables(const std::vector<int>& variables) {
    dual_variables_ = variables;
    num_variables_ = primal_variables_.size() + dual_variables_.size();
    submatrix_data_.m_ = number_of_variables();
  };

  SchurComplementSystem* submatrix_data() { return &submatrix_data_; }

 protected:
  WorkspaceSchurComplement submatrix_data_;
  mutable Eigen::VectorXd ysegment;
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
  std::vector<int> primal_variables_;
  std::vector<int> dual_variables_;

  bool direct_update = false;
  std::vector<DiagonalBlock> diag;
  std::vector<OffDiagonalBlock> off_diag;
  std::vector<OffDiagonalBlock> scatter_block;
  virtual ~SupernodalAssemblerBase(){};
};

class SupernodalAssemblerConstraint : public SupernodalAssemblerBase {
 public:
  SupernodalAssemblerConstraint(const std::vector<int>& variables,
                                Constraint* W, ConstraintBase* serializer)
      : SupernodalAssemblerBase(variables) {
    workspace_ = W;
    serializer_ = serializer;
    CONEX_CHECK(W);
    CONEX_CHECK(serializer_);
  }

  void accept(Visitor* v) override { serializer_->accept(v); }
  virtual bool is_dynamic() const override { return true; }
  virtual bool is_positive_definite() const override { return true; }
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

  SupernodalAssemblerConstraint(){};
  Constraint* workspace_ = NULL;
  ConstraintBase* serializer_ = NULL;
  Eigen::VectorXd memory_;
};

class SupernodalAssemblerStatic : public SupernodalAssemblerBase {
 public:
  SupernodalAssemblerStatic(const Eigen::MatrixXd& A,
                            const std::vector<int>& variables)
      : SupernodalAssemblerBase(variables), A_(A) {
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
};

class SupernodalAssemblerQuadratic : public SupernodalAssemblerStatic {
 public:
  SupernodalAssemblerQuadratic(const Eigen::MatrixXd& A,
                               const std::vector<int>& variables)
      : SupernodalAssemblerStatic(A, variables) {}
  double EvaluateQuadraticCost(const Eigen::Ref<const Eigen::MatrixXd> x) const;
  Eigen::MatrixXd CostMatrix() const { return A_; }
};
}  // namespace conex
