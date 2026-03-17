#pragma once
#include "newton_step.h"
#include "psd_constraint.h"
#include "conex/supernodal_assembler_base.h"

namespace conex {

class DenseLMIGramEvaluator : public LazySymmetricMatrix {
 public:
  DenseLMIGramEvaluator() = default;
  void bind(WorkspaceDensePSD* ws, const Eigen::MatrixXd* A_vect,
            int num_vars) {
    ws_ = ws;
    A_vect_ = A_vect;
    num_vars_ = num_vars;
  }

  void set_order(const std::vector<int>& perm) override;
  void update_weights();

  void add_block(int row, int col, int rows, int cols,
                 Eigen::Ref<Eigen::MatrixXd> dest) const override;
  void add_block_lower(int pos, int size,
                       Eigen::Ref<Eigen::MatrixXd> dest) const override;

  int rows() const override { return num_vars_; }
  int cols() const override { return num_vars_; }

  bool is_active() const { return order_set_; }
  void invalidate_order() { order_set_ = false; }

 private:
  WorkspaceDensePSD* ws_ = nullptr;
  const Eigen::MatrixXd* A_vect_ = nullptr;
  int num_vars_ = 0;
  Eigen::MatrixXd A_vect_perm_;
  Eigen::MatrixXd WAW_vect_perm_;
  bool order_set_ = false;
};

class MatrixLMIConstraint : public PsdConstraint {
 public:
  MatrixLMIConstraint(int n,
                      const std::vector<DenseMatrix>& constraint_matrices,
                      const DenseMatrix& constraint_affine);

  Eigen::MatrixXd constraint_matrices_vect_;
  const std::vector<DenseMatrix> constraint_matrices() const {
    return constraint_matrices_;
  }
  const DenseMatrix affine_term() const { return constraint_affine_; }
  const std::vector<DenseMatrix> constraint_matrices_;
  const DenseMatrix constraint_affine_;

 protected:
  void ComputeAW(int i, const Ref& W, Ref* AW, Ref* WAW);
  void ComputeWCW(const Ref& W, Ref* CW, Ref* WCW);
  double EvalDualConstraint(int j, const Ref& W);
  double EvalDualObjective(const Ref& W);
};

class DenseLMIConstraint final : public MatrixLMIConstraint {
 public:
  DenseLMIConstraint(int n, const std::vector<DenseMatrix>& constraint_matrices,
                     const DenseMatrix& constraint_affine)
      : MatrixLMIConstraint(n, constraint_matrices, constraint_affine) {}

  DenseLMIConstraint(const std::vector<DenseMatrix>& constraint_matrices,
                     const DenseMatrix& constraint_affine)
      : MatrixLMIConstraint(constraint_affine.rows(), constraint_matrices,
                            constraint_affine) {}

  void accept(Visitor* v) const override { v->visit(*this); }

  LazySymmetricMatrix* GetLazyEvaluator() override {
    gram_evaluator_.bind(&workspace_, &constraint_matrices_vect_,
                         num_dual_constraints_);
    gram_evaluator_.update_weights();
    return &gram_evaluator_;
  }

 private:
  void do_schur_complement(bool initialize,
                           SchurComplementSystem* sys) override {
    ConstructSchurComplementSystemImpl(initialize, sys);
  }

  void ComputeNegativeSlack(double k, const Ref& y, Ref* s) override;
  void ConstructSchurComplementSystemImpl(bool initialize,
                                          SchurComplementSystem* sys);

  DenseLMIGramEvaluator gram_evaluator_;
};
}  // namespace conex
