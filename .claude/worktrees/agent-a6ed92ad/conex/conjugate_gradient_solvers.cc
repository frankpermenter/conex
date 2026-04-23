#include "conex/conjugate_gradient_solvers.h"

#include "conex/error_checking_macros.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::vector;

namespace conex {

namespace {

template <typename T1, typename T2>
MatrixXd MakeDenseMatrix(const T1& non_zero_columns, const T2& entries,
                         int num_columns) {
  MatrixXd B(non_zero_columns.size(), num_columns);
  B.setZero();
  for (size_t i = 0; i < non_zero_columns.size(); i++) {
    int cnt = 0;
    for (int c : non_zero_columns.at(i)) {
      B(i, c) = entries.at(i).at(cnt++);
    }
  }
  return B;
}

MatrixXd SparseTransposeProduct(const vector<vector<int>>& non_zero_columns,
                                const vector<vector<double>>& entries,
                                int num_columns, const MatrixXd& x) {
  MatrixXd y(num_columns, x.cols());
  y.setZero();
  for (size_t i = 0; i < non_zero_columns.size(); i++) {
    int cnt = 0;
    for (int c : non_zero_columns.at(i)) {
      y.row(c) += entries[i][cnt++] * x.row(i);
    }
  }
  return y;
}

MatrixXd SparseMatrixProduct(const vector<vector<int>>& non_zero_columns,
                             const vector<vector<double>>& entries,
                             const MatrixXd& x) {
  MatrixXd y(non_zero_columns.size(), x.cols());
  y.setZero();
  for (size_t i = 0; i < non_zero_columns.size(); i++) {
    int cnt = 0;
    for (int c : non_zero_columns.at(i)) {
      y.row(i) += entries[i][cnt++] * x.row(c);
    }
  }
  return y;
}

}  // namespace

using T = ConstrainedLeastSquaresConjugateGradientSolver;

T::ConstrainedLeastSquaresConjugateGradientSolver(
    std::unique_ptr<SupernodalKKTSolver>&& solver,
    const std::vector<std::vector<int>>& non_zero_columns_of_B,
    const std::vector<std::vector<double>>& entries_of_B)
    : inverse_of_G_(std::move(solver)),
      non_zero_columns_of_B_(non_zero_columns_of_B),
      entries_of_B_(entries_of_B) {
  for (auto& e : non_zero_columns_of_B_) {
    for (int& ei : e) {
      ei = inverse_of_G_->permutation_to_elimination_order().indices()(ei);
    }
  }
}

bool T::DoFactor() { return inverse_of_G_->Factor(); }

void T::DoAssemble() { inverse_of_G_->Assemble(); }

Eigen::VectorXd T::EvaluateEquationOperator(
    const Eigen::VectorXd& residual) const {
  return SparseMatrixProduct(non_zero_columns_of_B_, entries_of_B_, residual);
}

Eigen::VectorXd T::EvaluateEquationOperatorTranspose(
    const Eigen::VectorXd& residual) const {
  int num_cols_of_B = inverse_of_G_->SizeOfSystem();
  return SparseTransposeProduct(non_zero_columns_of_B_, entries_of_B_,
                                num_cols_of_B, residual);
}

Eigen::VectorXd T::ApplyPreconditioner(const Eigen::VectorXd& x) const {
  // Skip preconditioning
  return x;
  // Implements a diagonal preconditioner. This is for testing effectiveness
  // of preconditioning. TODO(FrankPermenter): Remove explicit construction
  // of schur_complement.
  MatrixXd B = MakeDenseMatrix(non_zero_columns_of_B_, entries_of_B_,
                               number_of_variables());
  MatrixXd Ginv_Bt(B.cols(), B.rows());
  for (int i = 0; i < B.rows(); i++) {
    Ginv_Bt.col(i) =
        inverse_of_G_->Solve(B.row(i).transpose(), false /*permute*/);
  }

  MatrixXd schur_complement =
      SparseMatrixProduct(non_zero_columns_of_B_, entries_of_B_, Ginv_Bt);

  VectorXd y(x.rows());
  for (int i = 0; i < x.rows(); i++) {
    y(i) = x(i) / schur_complement(i, i);
  }
  return y;
}

Eigen::VectorXd T::SchurComplementConjugateGradientSolver(
    const Eigen::VectorXd& rhs) const {
  ConstrainedLeastSquaresConjugateGradientSolverConfig config;
  config.iteration_limit = rhs.rows();

  auto f = [this](const VectorXd& s) -> VectorXd {
    return EvaluateEquationOperator(inverse_of_G_->Solve(
        EvaluateEquationOperatorTranspose(s), false /*permute*/));
  };

  int num_rows = rhs.rows();

  VectorXd x(num_rows);
  x.setZero();

  double eps = 1e-12;

  int n = num_rows;

  VectorXd residual = rhs;

  if (rhs.squaredNorm() == 0) {
    x.setZero();
    return x;
  }

  double threshold = eps * eps * rhs.squaredNorm();
  VectorXd p(n);
  p = ApplyPreconditioner(residual);

  VectorXd z(n), tmp(n);
  double absNew = (residual.dot(p));
  int i = 0;
  while (i < config.iteration_limit) {
    tmp.noalias() = f(p);

    double alpha = absNew / p.dot(tmp);
    x += alpha * p;
    residual -= alpha * tmp;

    if (residual.squaredNorm() < threshold) {
      break;
    }

    z = ApplyPreconditioner(residual);

    double absOld = absNew;
    absNew = residual.dot(z);
    double beta = absNew / absOld;
    p = z + beta * p;
    i++;
  }
  return x;
}

}  // namespace conex

namespace conex {

void T::DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> y,
                       bool permutation_to_elimination_order) const {
  if (y.rows() != number_of_variables() + number_of_equations()) {
    throw std::runtime_error(
        "Cannot perform solve in place. Input dimensions disagree with system "
        "size.");
  }
  const VectorXd f = y.topRows(number_of_variables());
  const VectorXd g = y.bottomRows(number_of_equations());
  VectorXd s1;
  VectorXd s2;
  Solve(f, g, &s1, &s2, /*use llt*/ false);
  y.topRows(number_of_variables()) = s1;
  y.bottomRows(number_of_equations()) = s2;
}

void T::Solve(const VectorXd& fin, const VectorXd& g, VectorXd* y, VectorXd* z,
              bool use_llt) const {
  VectorXd f_permuted = inverse_of_G_->permutation_to_elimination_order() * fin;

  VectorXd Ginv_f = inverse_of_G_->Solve(f_permuted, false /*permute*/);
  int num_columns_of_B = f_permuted.rows();

  VectorXd schur_complement_residual =
      SparseMatrixProduct(non_zero_columns_of_B_, entries_of_B_, Ginv_f) - g;

  if (use_llt) {
    MatrixXd B = MakeDenseMatrix(non_zero_columns_of_B_, entries_of_B_,
                                 num_columns_of_B);
    MatrixXd Ginv_Bt(B.cols(), B.rows());
    for (int i = 0; i < B.rows(); i++) {
      Ginv_Bt.col(i) = inverse_of_G_->Solve(B.row(i).transpose());
    }

    MatrixXd schur_complement =
        SparseMatrixProduct(non_zero_columns_of_B_, entries_of_B_, Ginv_Bt);
    Eigen::LLT<MatrixXd> llt(schur_complement);
    CONEX_DEMAND(llt.info() == Eigen::Success, "LLT Factorization failed.");
    *z = llt.solve(schur_complement_residual);
  } else {
    *z = SchurComplementConjugateGradientSolver(schur_complement_residual);
  }

  *y = inverse_of_G_->Solve(
      f_permuted - SparseTransposeProduct(non_zero_columns_of_B_, entries_of_B_,
                                          num_columns_of_B, *z),
      false /*permuted*/);
  *y = inverse_of_G_->permutation_from_elimination_order() * (*y);
}

MatrixXd T::DoKKTMatrix(bool permute_to_elimination_order) const {
  MatrixXd G = inverse_of_G_->KKTMatrix(permute_to_elimination_order);
  MatrixXd B = MakeDenseMatrix(non_zero_columns_of_B_, entries_of_B_, G.cols());
  if (!permute_to_elimination_order) {
    B = B * inverse_of_G_->permutation_from_elimination_order();
  }
  int dim = G.rows() + B.rows();
  MatrixXd M(dim, dim);
  M << G, B.transpose(), B, MatrixXd::Zero(B.rows(), B.rows());
  return M;
}
}  // namespace conex
