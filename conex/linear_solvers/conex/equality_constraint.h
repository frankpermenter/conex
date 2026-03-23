#pragma once

#include <vector>

#include "conex/constraint.h"
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

class EqualityConstraints : public Constraint {
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

  WorkspaceEqualityConstraints workspace_;
  WorkspaceEqualityConstraints* workspace() { return &workspace_; }

 private:
  void do_schur_complement(bool initialize,
                           SchurComplementSystem* sys) override {
    ConstructSchurComplementSystemImpl(initialize, sys);
  }

  void do_set_identity() override {}

  void do_weighted_slack_eigenvalues(const Ref& y, double c_weight,
                                     WeightedSlackEigenvalues* p) override {}

  Workspace do_get_workspace() override { return Workspace(workspace()); }

  void do_prepare_step(const StepOptions& opt, const Ref& y,
                       StepInfo* info) override {
    PrepareStepImpl(opt, y, info);
  }

  void do_get_dual_variable(double* var) override {
    CopyDualVariableFromWorkspace(workspace(), var);
  }

  bool do_take_step(const StepOptions& opts) override { return true; }

  int do_dual_variable_size() override {
    return DualVariableSizeFromWorkspace(workspace());
  }

  int do_number_of_variables() const override { return number_of_variables(); }

  bool do_perform_line_search(const LineSearchParameters& params,
                              const Eigen::Ref<const Eigen::MatrixXd>& y0,
                              const Eigen::Ref<const Eigen::MatrixXd>& y1,
                              LineSearchOutput* output) override {
    return false;
  }

  int do_rank() const override { return 0; }

  void ConstructSchurComplementSystemImpl(bool initialize,
                                          SchurComplementSystem* sys_);
  void PrepareStepImpl(const StepOptions&, const Ref& y, StepInfo* info_i);
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
