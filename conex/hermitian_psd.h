#pragma once
#include "conex/constraint.h"
#include "conex/error_codes.h"
#include "conex/jordan_matrix_algebra.h"
#include "conex/newton_step.h"
#include "conex/workspace.h"
#include <Eigen/Dense>

namespace conex {

struct WorkspaceDenseHermitian {
  WorkspaceDenseHermitian(int n) : n_(n) {}
  WorkspaceDenseHermitian(int n, double* data)
      : W(data, n, n),
        temp_1(data + get_size_aligned(n * n), n, n),
        temp_2(data + get_size_aligned(n * n), n, n) {}
  static constexpr int size_of(int n) { return 3 * (get_size_aligned(n * n)); }

  friend int SizeOf(const WorkspaceDenseHermitian& o) { return size_of(o.n_); }

  friend void Initialize(WorkspaceDenseHermitian* o, double* data) {
    using Map = Eigen::Map<DenseMatrix, Eigen::Aligned>;
    int n = o->n_;
    new (&o->W) Map(data, n, n);
    new (&o->temp_1) Map(data + 1 * get_size_aligned(n * n), n, n);
    new (&o->temp_2) Map(data + 2 * get_size_aligned(n * n), n, n);
  }

  friend void print(const WorkspaceDenseHermitian& o) {
    DUMP(o.W);
    DUMP(o.temp_1);
    DUMP(o.temp_2);
  }

  Eigen::Map<DenseMatrix, Eigen::Aligned> W{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> temp_1{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> temp_2{NULL, 0, 0};
  int n_;
};

template <typename T = Real>
class HermitianPsdConstraint : public Constraint {
 public:
  using Matrix = typename T::Matrix;

  void accept(Visitor* v) const override { v->visit(*this); }
  HermitianPsdConstraint(int n, int number_of_variables)
      : rank_(n), workspace_(n), constraint_matrices_(number_of_variables) {
    for (auto& c : constraint_matrices_) {
      c = T::Zero(n, n);
    }
    constraint_affine_ = T::Zero(n, n);
  }

  HermitianPsdConstraint(int n, const std::vector<Matrix>& a, const Matrix& c)
      : rank_(n),
        workspace_(n),
        constraint_matrices_(a),
        constraint_affine_(c) {}

  WorkspaceDenseHermitian* workspace() { return &workspace_; }
  friend void SetIdentity(HermitianPsdConstraint* o) {
    o->W = T::Identity(o->rank_);
  }
  friend int Rank(const HermitianPsdConstraint& o) { return o.rank_; };

  template <typename H>
  friend void GetWeightedSlackEigenvalues(HermitianPsdConstraint<H>* o,
                                          const Ref& y, double c_weight,
                                          WeightedSlackEigenvalues* p);

  int number_of_variables() const override {
    return constraint_matrices_.size();
  }

  template <typename H>
  friend void PrepareStep(HermitianPsdConstraint<H>* o, const StepOptions& opt,
                          const Ref& y, StepInfo*);

  template <typename H>
  friend bool TakeStep(HermitianPsdConstraint<H>* o, const StepOptions& opt);

  template <typename H>
  friend void ConstructSchurComplementSystem(HermitianPsdConstraint<H>* o,
                                             bool initialize,
                                             SchurComplementSystem* sys);

  template <typename H>
  friend CONEX_STATUS UpdateLinearOperator(HermitianPsdConstraint<H>* o,
                                           double val, int var, int r, int c,
                                           int dim);

  template <typename H>
  friend CONEX_STATUS UpdateAffineTerm(HermitianPsdConstraint<H>* o, double val,
                                       int r, int c, int dim);

  void do_schur_complement(bool initialize,
                           SchurComplementSystem* sys) override {
    ConstructSchurComplementSystem(this, initialize, sys);
  }

  void do_set_identity() override { SetIdentity(this); }

  void do_weighted_slack_eigenvalues(const Ref& y, double c_weight,
                                     WeightedSlackEigenvalues* p) override {
    GetWeightedSlackEigenvalues(this, y, c_weight, p);
  }

  Workspace do_get_workspace() override { return Workspace(workspace()); }

  void do_prepare_step(const StepOptions& opt, const Ref& y,
                       StepInfo* info) override {
    PrepareStep(this, opt, y, info);
  }

  void do_get_dual_variable(double* var) override {
    memcpy(static_cast<void*>(var), static_cast<void*>(workspace()->W.data()),
           sizeof(double) * do_dual_variable_size());
  }

  bool do_take_step(const StepOptions& opts) override {
    return TakeStep(this, opts);
  }

  int do_dual_variable_size() override {
    return workspace()->W.rows() * workspace()->W.cols();
  }

  int do_number_of_variables() const override { return number_of_variables(); }

  CONEX_STATUS do_update_linear_operator(double val, int var, int row, int col,
                                         int hyper_complex_dim) override {
    return UpdateLinearOperator(this, val, var, row, col, hyper_complex_dim);
  }

  CONEX_STATUS do_update_affine_term(double val, int row, int col,
                                     int hyper_complex_dim) override {
    return UpdateAffineTerm(this, val, row, col, hyper_complex_dim);
  }

  int do_rank() const override { return Rank(*this); }

 private:
  int rank_;
  WorkspaceDenseHermitian workspace_;
  std::vector<Matrix> constraint_matrices_;
  Matrix constraint_affine_;

  // TODO(FrankPermenter): Move to workspace.
  Matrix W;
  Matrix WS;
  Matrix minus_s;

  double EvalDualConstraint(int j, const Matrix& W) {
    return T::TraceInnerProduct(constraint_matrices_.at(j), W);
  }
  double EvalDualObjective(const Matrix& W) {
    return T::TraceInnerProduct(constraint_affine_, W);
  }

  void ComputeNegativeSlack(double k, const Ref& y, Matrix* S) {
    *S = T::ScalarMultiply(constraint_affine_, -k);
    for (unsigned int i = 0; i < constraint_matrices_.size(); i++) {
      *S = T::Add(*S, T::ScalarMultiply(constraint_matrices_.at(i), y(i)));
    }
  }
};

using RealLMIConstraint = HermitianPsdConstraint<Real>;
using ComplexLMIConstraint = HermitianPsdConstraint<Complex>;
using QuaternicLMIConstraint = HermitianPsdConstraint<Quaternions>;
using OctonicLMIConstraint = HermitianPsdConstraint<Octonions>;

}  // namespace conex
