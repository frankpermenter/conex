#include "newton_step.h"
#include "psd_constraint.h"

namespace conex {


// using InputRefType = Eigen::Map<const DenseMatrix>;
using InputRefType = DenseMatrix;

class MatrixLMIConstraint : public PsdConstraint {
 public:
  MatrixLMIConstraint(int n,
                      const std::vector<DenseMatrix>& constraint_matrices,
                      const DenseMatrix& constraint_affine)
      : PsdConstraint(n, static_cast<int>(constraint_matrices.size())),
        constraint_matrices_(constraint_matrices),
        constraint_affine_(constraint_affine) {
    int m = constraint_matrices_.size();
    constraint_matrices_vect_.resize(n * n, m);
    for (int i = 0; i < m; i++) {
      memcpy(&(constraint_matrices_vect_(0, i)),
             constraint_matrices_.at(i).data(), sizeof(double) * n * n);
    }
  }

  Eigen::MatrixXd constraint_matrices_vect_;
  const std::vector<DenseMatrix> constraint_matrices_;
  const DenseMatrix constraint_affine_;

 protected:
  void ComputeNegativeSlack(double k, const Ref& y, Ref* s);
  void ComputeAW(int i, const Ref& W, Ref* AW, Ref* WAW);
  double EvalDualConstraint(int j, const Ref& W);
  double EvalDualObjective(const Ref& W);
  void MultByA(const Ref& x, Ref* Y);
  virtual int variable(int i) { return i; }
};

class DenseLMIConstraint final : public MatrixLMIConstraint {
 public:
  DenseLMIConstraint(int n, const std::vector<DenseMatrix>& constraint_matrices,
                     const DenseMatrix& constraint_affine)
      : MatrixLMIConstraint(n, constraint_matrices, constraint_affine) {}

  friend void ConstructSchurComplementSystem<DenseLMIConstraint>(
      DenseLMIConstraint* o, bool initialize, SchurComplementSystem* sys);
  int variable(int i) override { return i; }
};

template <>
void ConstructSchurComplementSystem(DenseLMIConstraint* o, bool initialize,
                                    SchurComplementSystem* sys);

class SparseLMIConstraint final : public MatrixLMIConstraint {
 public:
  SparseLMIConstraint(const std::vector<DenseMatrix>& constraint_matrices,
                      const DenseMatrix& constraint_affine,
                      const std::vector<int>& variables)
      : MatrixLMIConstraint(constraint_affine.rows(), constraint_matrices,
                            constraint_affine),
        variables_(variables) {}

  friend void ConstructSchurComplementSystem<SparseLMIConstraint>(
      SparseLMIConstraint* o, bool initialize, SchurComplementSystem* sys);
  std::vector<int> variables_;
  int variable(int i) override {
    // TODO(FrankPermenter): remove range check.
    return variables_.at(i);
  }
};

} // namespace conex
