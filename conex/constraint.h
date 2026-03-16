#pragma once
#include <cstring>
#include <memory>
#include <vector>

#include "conex/constraint_interface.h"
#include "conex/error_checking_macros.h"
#include "conex/error_codes.h"
#include "conex/newton_step.h"
#include "conex/supernodal_assembler_base.h"
#include "conex/workspace.h"
#include <Eigen/Dense>

namespace conex {

class Constraint : public IVisitable, public IVariableShape {
 public:
  virtual ~Constraint() = default;
  virtual bool supports_line_search() const { return false; }

  void BuildSchurComplement(bool initialize, SchurComplementSystem* sys) {
    do_schur_complement(initialize, sys);
  }

  void SetIdentity() { do_set_identity(); }

  void PrepareStep(const StepOptions& opt, const Ref& y, StepInfo* info) {
    do_prepare_step(opt, y, info);
  }

  void GetWeightedSlackEigenvalues(const Ref& y, double c_weight,
                                   WeightedSlackEigenvalues* p) {
    do_weighted_slack_eigenvalues(y, c_weight, p);
  }

  int Rank() const { return do_rank(); }

  Workspace workspace() { return do_get_workspace(); }

  void get_dual_variable(double* v) { return do_get_dual_variable(v); }

  void ApplyRescaling(Eigen::Ref<Eigen::MatrixXd> ArW,
                      double* inner_product_of_c_and_rW) {
    do_apply_rescaling(ArW, inner_product_of_c_and_rW);
  }

  int dual_variable_size() { return do_dual_variable_size(); }

  int number_of_variables() const override { return do_number_of_variables(); }

  CONEX_STATUS UpdateLinearOperator(double val, int var, int row, int col,
                                    int hyper_complex_dim) {
    return do_update_linear_operator(val, var, row, col, hyper_complex_dim);
  }

  CONEX_STATUS UpdateAffineTerm(double val, int row, int col,
                                int hyper_complex_dim) {
    return do_update_affine_term(val, row, col, hyper_complex_dim);
  }

  bool TakeStep(const StepOptions& opts) { return do_take_step(opts); }

  bool PerformLineSearch(const LineSearchParameters& params,
                         const Eigen::Ref<const Eigen::MatrixXd>& y0,
                         const Eigen::Ref<const Eigen::MatrixXd>& y1,
                         LineSearchOutput* output) {
    return do_perform_line_search(params, y0, y1, output);
  }

  virtual LazySymmetricMatrix* GetLazyEvaluator() { return nullptr; }

 protected:
  template <typename WorkspaceType>
  static void CopyDualVariableFromWorkspace(WorkspaceType* workspace,
                                            double* var) {
    memcpy(static_cast<void*>(var), static_cast<void*>(workspace->W.data()),
           sizeof(double) * DualVariableSizeFromWorkspace(workspace));
  }

  template <typename WorkspaceType>
  static int DualVariableSizeFromWorkspace(WorkspaceType* workspace) {
    return workspace->W.rows() * workspace->W.cols();
  }

 private:
  virtual void do_schur_complement(bool initialize,
                                   SchurComplementSystem* sys) = 0;
  virtual void do_set_identity() = 0;
  virtual void do_weighted_slack_eigenvalues(const Ref& y, double c_weight,
                                             WeightedSlackEigenvalues* p) = 0;
  virtual Workspace do_get_workspace() = 0;
  virtual void do_prepare_step(const StepOptions& opt, const Ref& y,
                               StepInfo* info) = 0;
  virtual void do_get_dual_variable(double*) = 0;
  virtual bool do_take_step(const StepOptions&) = 0;
  virtual int do_dual_variable_size() = 0;
  virtual int do_number_of_variables() const = 0;

  virtual void do_apply_rescaling(Eigen::Ref<Eigen::MatrixXd> ArW,
                                  double* inner_product_of_c_and_rW) {
    CONEX_DEMAND(false, "Constraint does not support rescaling.");
  }

  virtual CONEX_STATUS do_update_linear_operator(double val, int var, int row,
                                                 int col,
                                                 int hyper_complex_dim) {
    CONEX_RETURN_ON_FAIL(
        false, "Constraint does not support updates of linear operator.");
  }

  virtual CONEX_STATUS do_update_affine_term(double val, int row, int col,
                                             int hyper_complex_dim) {
    CONEX_RETURN_ON_FAIL(false,
                         "Constraint does not support updates of affine term.");
  }

  virtual bool do_perform_line_search(
      const LineSearchParameters& params,
      const Eigen::Ref<const Eigen::MatrixXd>& y0,
      const Eigen::Ref<const Eigen::MatrixXd>& y1, LineSearchOutput* output) {
    CONEX_RETURN_ON_FAIL(false, "Constraint does not support line search.");
  }

  virtual int do_rank() const = 0;
};

class SupernodalAssemblerConstraint : public SupernodalAssemblerBase {
 public:
  SupernodalAssemblerConstraint(const std::vector<int>& variables,
                                Constraint* W)
      : SupernodalAssemblerBase(variables) {
    workspace_ = W;
    CONEX_CHECK(W);
  }

  void accept(Visitor* v) const override { workspace_->accept(v); }
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
      workspace_->BuildSchurComplement(true, &submatrix_data_);
    } else {
      throw std::runtime_error("Supernodal assembler data source is not set.");
    }
  }

  virtual bool supports_line_search() const override {
    return workspace_->supports_line_search();
  }

  LazySymmetricMatrix* GetLazyEvaluator() override {
    return workspace_->GetLazyEvaluator();
  }

  SupernodalAssemblerConstraint(){};
  Constraint* workspace_ = NULL;
  Eigen::VectorXd memory_;
};

}  // namespace conex
