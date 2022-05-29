#undef EIGEN_MPL2_ONLY
#include "conex/kkt_tree_solver.h"
#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>

namespace conex {
using Eigen::MatrixXd;

template <typename T>
bool FactorizationSucceed(const T&) {
  return true;
}

template <>
bool FactorizationSucceed(
    const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>& x) {
  return x.info() == Eigen::Success;
}

#if 1
class SparseFactorization {
 public:
  SparseFactorization(const Eigen::SparseMatrix<double>& x)
      : factorization(x.selfadjointView<Eigen::Lower>()) {}
  // Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> factorization;
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
      factorization;
};
#else

class SparseFactorization {
 public:
  SparseFactorization(const Eigen::MatrixXd& x)
      : factorization(x.selfadjointView<Eigen::Lower>()) {}
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> factorization;
};
#endif

using T = EigenSparseCholesky;

T::~EigenSparseCholesky() {}
T::EigenSparseCholesky(
    std::unique_ptr<SymmetricLinearSystemTreeSolver>&& solver)
    : solver_(std::move(solver)) {}

void T::DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                       bool in_original_order) const {
  CONEX_CHECK(in_original_order);
  Eigen::MatrixXd rhs = b;
  b = factorization_->factorization.solve(rhs);
}

void T::DoAssemble() { solver_->Assemble(); }

bool T::DoAssembleAndFactor() {
  solver_->Assemble();
  Eigen::SparseMatrix<double> matrix =
      solver_
          ->MakeSparseKKTMatrix(false /* true = permute to elimination order*/)
          .selfadjointView<Eigen::Lower>();
  Eigen::VectorXd d = matrix.diagonal();
  double max_d = 0;
  double min_d = 1e15;
  for (int i = 0; i < d.rows(); i++) {
    if (d(i) == 0) {
      continue;
    }
    if (std::fabs(d(i)) > max_d) {
      max_d = std::fabs(d(i));
    }
    if (std::fabs(d(i)) < min_d) {
      min_d = std::fabs(d(i));
    }
  }
  factorization_ = std::make_unique<SparseFactorization>(matrix);
  return FactorizationSucceed(factorization_->factorization);
}

bool T::DoFactor() { return DoAssembleAndFactor(); }

Eigen::MatrixXd T::DoKKTMatrix(bool order) const {
  return solver_->KKTMatrix(order);
}

}  // namespace conex
