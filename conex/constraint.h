#pragma once
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "conex/constraint_interface.h"
#include "conex/error_checking_macros.h"
#include "conex/error_codes.h"
#include "conex/newton_step.h"
#include "conex/supernodal_assembler_base.h"
#include "conex/workspace.h"
#include <Eigen/Dense>

namespace conex {

template <typename T>
void ApplyRescaling(T*, Eigen::Ref<Eigen::MatrixXd>, double*) {
  throw std::runtime_error("Constraint does not support rescaling");
}

template <typename T>
CONEX_STATUS UpdateLinearOperator(T*, double, int, int, int, int) {
  CONEX_RETURN_ON_FAIL(
      false, "Constraint does not support updates of linear operator.");
}

template <typename T>
CONEX_STATUS UpdateAffineTerm(T*, double, int, int, int) {
  CONEX_RETURN_ON_FAIL(false,
                       "Constraint does not support updates of affine term.");
}

template <typename T>
bool PerformLineSearch(T*, const LineSearchParameters&,
                       const Eigen::Ref<const Eigen::MatrixXd>&,
                       const Eigen::Ref<const Eigen::MatrixXd>&,
                       LineSearchOutput*) {
  CONEX_RETURN_ON_FAIL(false, "Constraint does not support line search.");
}

// A helper class for forwarding to different implementations of an "interface."
// With this approach, implementations do not need to use inheritance and
// virtual functions. Instead, they simply provide functions of appropriate name
// and signature, e.g.,
//
//    void PrepareStep(Implementation1*, {arguments});
//    void Rank(Implementation1*, {arguments});
//    ..
//    void PrepareStep(Implementation2*, {arguments});
//    void Rank(Implementation2*, {arguments});
//
// Note that implementations can be ANSI C compliant when the signature is.
//
// Reference: "Inheritance is the base-class of evil" by Sean Parent.

class Constraint : public ConstraintBase {
 public:
  virtual ~Constraint() = default;

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

  friend void ConstraintConstructSchurComplementSystem(
      Constraint* o, bool initialize, SchurComplementSystem* sys) {
    o->do_schur_complement(initialize, sys);
  }

  friend void ConstraintSetIdentity(Constraint* o) { o->do_set_identity(); }

  friend void ConstraintPrepareStep(Constraint* o, const StepOptions& opt,
                                    const Ref& y, StepInfo* info) {
    o->do_prepare_step(opt, y, info);
  }

  friend void ConstraintGetWeightedSlackEigenvalues(
      Constraint* o, const Ref& y, double c_weight,
      WeightedSlackEigenvalues* p) {
    o->do_weighted_slack_eigenvalues(y, c_weight, p);
  }

  friend int ConstraintRank(const Constraint& o) { return o.do_rank(); }

  Workspace workspace() { return do_get_workspace(); }

  void get_dual_variable(double* v) { return do_get_dual_variable(v); }

  friend void ConstraintApplyRescaling(
      Constraint* o, Eigen::Ref<Eigen::MatrixXd> ArW,
      double* inner_product_of_c_and_rW) {
    o->do_apply_rescaling(ArW, inner_product_of_c_and_rW);
  }

  int dual_variable_size() { return do_dual_variable_size(); }

  int number_of_variables() const override { return do_number_of_variables(); }

  friend CONEX_STATUS ConstraintUpdateLinearOperator(
      Constraint* o, double val, int var, int row, int col,
      int hyper_complex_dim) {
    return o->do_update_linear_operator(val, var, row, col,
                                        hyper_complex_dim);
  }

  friend CONEX_STATUS ConstraintUpdateAffineTerm(
      Constraint* o, double val, int row, int col, int hyper_complex_dim) {
    return o->do_update_affine_term(val, row, col, hyper_complex_dim);
  }

  friend bool ConstraintTakeStep(Constraint* o, const StepOptions& opts) {
    return o->do_take_step(opts);
  }

  friend bool ConstraintPerformLineSearch(
      Constraint* o, const LineSearchParameters& params,
      const Eigen::Ref<const Eigen::MatrixXd>& y0,
      const Eigen::Ref<const Eigen::MatrixXd>& y1, LineSearchOutput* output) {
    return o->do_perform_line_search(params, y0, y1, output);
  }
};

template <typename Implementation>
class ConstraintAdapter final : public Constraint {
 public:
  explicit ConstraintAdapter(const Implementation& t) : data_(t) {}
  explicit ConstraintAdapter(Implementation&& t) : data_(std::move(t)) {}

  void accept(Visitor* v) const override { data_.accept(v); }
  bool supports_line_search() const override { return data_.supports_line_search(); }
  int do_number_of_variables() const override { return data_.number_of_variables(); }

  void do_schur_complement(bool initialize,
                           SchurComplementSystem* sys) override {
    ConstructSchurComplementSystem(&data_, initialize, sys);
  }

  void do_set_identity() override { SetIdentity(&data_); }

  void do_weighted_slack_eigenvalues(const Ref& y, double c_weight,
                                     WeightedSlackEigenvalues* p) override {
    GetWeightedSlackEigenvalues(&data_, y, c_weight, p);
  }

  Workspace do_get_workspace() override { return Workspace(data_.workspace()); }

  void do_prepare_step(const StepOptions& opt, const Ref& y,
                       StepInfo* info) override {
    PrepareStep(&data_, opt, y, info);
  }

  void do_get_dual_variable(double* var) override {
    memcpy(static_cast<void*>(var), static_cast<void*>(data_.workspace()->W.data()),
           sizeof(double) * do_dual_variable_size());
  }

  bool do_take_step(const StepOptions& opts) override {
    return TakeStep(&data_, opts);
  }

  int do_dual_variable_size() override {
    return data_.workspace()->W.rows() * data_.workspace()->W.cols();
  }

  void do_apply_rescaling(Eigen::Ref<Eigen::MatrixXd> ArW,
                          double* inner_product_of_c_and_rW) override {
    ApplyRescaling(&data_, ArW, inner_product_of_c_and_rW);
  }

  CONEX_STATUS do_update_linear_operator(double val, int var, int row, int col,
                                         int hyper_complex_dim) override {
    return UpdateLinearOperator(&data_, val, var, row, col, hyper_complex_dim);
  }

  CONEX_STATUS do_update_affine_term(double val, int row, int col,
                                     int hyper_complex_dim) override {
    return UpdateAffineTerm(&data_, val, row, col, hyper_complex_dim);
  }

  bool do_perform_line_search(const LineSearchParameters& params,
                              const Eigen::Ref<const Eigen::MatrixXd>& y0,
                              const Eigen::Ref<const Eigen::MatrixXd>& y1,
                              LineSearchOutput* output) override {
    return PerformLineSearch(&data_, params, y0, y1, output);
  }

  int do_rank() const override { return Rank(data_); }

 private:
  Implementation data_;
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
      ConstraintConstructSchurComplementSystem(workspace_, true,
                                               &submatrix_data_);
    } else {
      throw std::runtime_error("Supernodal assembler data source is not set.");
    }
  }

  virtual bool supports_line_search() const override {
    return workspace_->supports_line_search();
  }
  SupernodalAssemblerConstraint(){};
  Constraint* workspace_ = NULL;
  Eigen::VectorXd memory_;
};

}  // namespace conex
