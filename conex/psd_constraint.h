#pragma once
#include "conex/constraint.h"
#include "newton_step.h"
#include "workspace.h"
#include <Eigen/Dense>

namespace conex {

struct WorkspaceDensePSD {
  WorkspaceDensePSD(int n) : n_(n) {}
  WorkspaceDensePSD(int n, double* data)
      : W(data, n, n),
        temp_1(data + get_size_aligned(n * n), n, n),
        temp_2(data + get_size_aligned(n * n), n, n) {}
  static constexpr int size_of(int n) { return 3 * (get_size_aligned(n * n)); }

  friend int SizeOf(const WorkspaceDensePSD& o) { return size_of(o.n_); }

  friend void Initialize(WorkspaceDensePSD* o, double* data) {
    using Map = Eigen::Map<DenseMatrix, Eigen::Aligned>;
    int n = o->n_;
    new (&o->W) Map(data, n, n);
    new (&o->temp_1) Map(data + 1 * get_size_aligned(n * n), n, n);
    new (&o->temp_2) Map(data + 2 * get_size_aligned(n * n), n, n);
  }

  friend void print(const WorkspaceDensePSD& o) {
    DUMP(o.W);
    DUMP(o.temp_1);
    DUMP(o.temp_2);
  }

  Eigen::Map<DenseMatrix, Eigen::Aligned> W{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> temp_1{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> temp_2{NULL, 0, 0};
  int n_;
};

class PsdConstraint : public Constraint {
 public:
  WorkspaceDensePSD* workspace() { return &workspace_; }
  int number_of_variables() const override { return num_dual_constraints_; }

  virtual void do_schur_complement(bool initialize,
                                   SchurComplementSystem* sys) override = 0;

  void do_set_identity() override { SetIdentityImpl(); }

  void do_weighted_slack_eigenvalues(const Ref& y, double c_weight,
                                     WeightedSlackEigenvalues* p) override {
    GetWeightedSlackEigenvaluesImpl(y, c_weight, p);
  }

  Workspace do_get_workspace() override { return Workspace(workspace()); }

  void do_prepare_step(const StepOptions& opt, const Ref& y,
                       StepInfo* info) override {
    PrepareStepImpl(opt, y, info);
  }

  void do_get_dual_variable(double* var) override {
    memcpy(static_cast<void*>(var), static_cast<void*>(workspace()->W.data()),
           sizeof(double) * do_dual_variable_size());
  }

  bool do_take_step(const StepOptions& opts) override { return TakeStepImpl(opts); }

  int do_dual_variable_size() override {
    return workspace()->W.rows() * workspace()->W.cols();
  }

  int do_number_of_variables() const override { return number_of_variables(); }

  int do_rank() const override { return workspace_.n_; }

 protected:
  PsdConstraint(int n, int m) : workspace_(n), num_dual_constraints_{m} {}
  void GeodesicUpdate(double scale, const StepOptions&, Ref* sw);
  void AffineUpdate(double e_weight, Ref* sw);

  WorkspaceDensePSD workspace_;
  int num_dual_constraints_;
  virtual double EvalDualConstraint(int j, const Ref& W) = 0;
  virtual double EvalDualObjective(const Ref& W) = 0;
  virtual void ComputeAW(int i, const Ref& W, Ref* AW, Ref* WAW) = 0;
  virtual void ComputeNegativeSlack(double k, const Ref& y, Ref* s) = 0;
  void SetIdentityImpl();
  void PrepareStepImpl(const StepOptions& opt, const Ref& y, StepInfo* info);
  bool TakeStepImpl(const StepOptions& options);
  void GetWeightedSlackEigenvaluesImpl(const Ref& y, double c_weight,
                                       WeightedSlackEigenvalues* p);
  virtual ~PsdConstraint(){};
};

}  // namespace conex
