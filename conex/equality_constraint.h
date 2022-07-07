#pragma once

#include <vector>

#include "conex/constraint_interface.h"
#include "conex/error_checking_macros.h"
#include "conex/newton_step.h"
#include "conex/supernodal_assembler_base.h"
#include "conex/workspace.h"
#include <Eigen/Dense>
namespace conex {

struct WorkspaceEqualityConstraints {
  using DenseMatrix = Eigen::MatrixXd;

  friend int SizeOf(const WorkspaceEqualityConstraints&) { return 0; }

  friend void Initialize(WorkspaceEqualityConstraints*, double*) {}

  friend void print(const WorkspaceEqualityConstraints&) {}
  Eigen::Map<DenseMatrix, Eigen::Aligned> W{NULL, 0, 0};
};

class EqualityConstraints : public ConstraintBase {
 public:
  void accept(Visitor* v) const override { v->visit(*this); }
  EqualityConstraints(){};
  EqualityConstraints(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b);

  int SizeOfDualVariable() { return A_.rows(); }
  Eigen::MatrixXd constraint_matrix() const { return A_; }
  Eigen::MatrixXd affine_term() const { return b_; }
  Eigen::MatrixXd A_;
  Eigen::MatrixXd b_;

  int number_of_variables() const override { return A_.cols(); }

  friend int Rank(const EqualityConstraints&) { return 0; };
  friend void SetIdentity(EqualityConstraints*){};
  friend void PrepareStep(EqualityConstraints* o, const StepOptions&,
                          const Ref& y, StepInfo*);

  friend bool TakeStep(EqualityConstraints*, const StepOptions&) {
    return true;
  };

  friend void GetWeightedSlackEigenvalues(EqualityConstraints*, const Ref&,
                                          double, WeightedSlackEigenvalues*){};

  friend void ConstructSchurComplementSystem(EqualityConstraints* o,
                                             bool initialize,
                                             SchurComplementSystem* sys_);

  int number_of_variables() { return 0; }
  WorkspaceEqualityConstraints workspace_;
  WorkspaceEqualityConstraints* workspace() { return &workspace_; }
  friend bool PerformLineSearch(EqualityConstraints*,
                                const LineSearchParameters&, const Ref&,
                                const Ref&, LineSearchOutput*) {
    bool failure = false;
    return failure;
  }
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

}  // namespace conex
