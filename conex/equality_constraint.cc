#include "conex/equality_constraint.h"

namespace conex {

using T = EqualityConstraints;
using Eigen::MatrixXd;
using std::vector;

T::EqualityConstraints(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b)
    : A_(A), b_(b) {}

void ConstructSchurComplementSystem(EqualityConstraints* o, bool initialize,
                                    SchurComplementSystem* sys_) {
  auto& sys = *sys_;
  auto& A_ = o->A_;
  auto& b_ = o->b_;

  if (!sys_->initialized) {
    throw std::runtime_error("Schur complement workspace is not initialized");
  }

  // Fills lower-triangular part of
  //    0 A'
  //    A 0
  if (initialize) {
    sys.setZero();
    sys.G.bottomLeftCorner(A_.rows(), A_.cols()) = A_;
    sys.AQc.bottomRows(A_.rows()) = b_;
  } else {
    sys.G.bottomLeftCorner(A_.rows(), A_.cols()) += A_;
    sys.AQc.bottomRows(A_.rows()) += b_;
  }
}

void PrepareStep(EqualityConstraints* o, const StepOptions&, const Ref& y,
                 StepInfo* info_i) {
  info_i->normsqrd = 0;
  info_i->norminfd = 0;
}

}  // namespace conex
