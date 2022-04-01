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
  // x
  // x
  // x
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
    const std::vector<std::vector<int>>& cliques_of_G,
    const std::vector<SupernodalAssemblerBase*>& clique_assemblers_of_G,
    const std::vector<std::vector<int>>& non_zero_columns_of_B,
    const std::vector<std::vector<double>>& entries_of_B)
    : inverse_of_G_(cliques_of_G),
      non_zero_columns_of_B_(non_zero_columns_of_B),
      entries_of_B_(entries_of_B) {
  inverse_of_G_.Bind(clique_assemblers_of_G);
}

bool T::Factor() {
  if (!inverse_of_G_.Factor()) {
    std::runtime_error("Failed to factor KKT system");
  }
  factored_ = true;
  assembled_ = false;
  return CONEX_SUCCESS;
}

void T::Assemble() {
  factored_ = false;
  inverse_of_G_.Assemble();
  assembled_ = true;
}

Eigen::VectorXd T::EvaluateEquationOperator(const Eigen::VectorXd& residual) {
  return SparseMatrixProduct(non_zero_columns_of_B_, entries_of_B_, residual);
}

Eigen::VectorXd T::EvaluateEquationOperatorTranspose(
    const Eigen::VectorXd& residual) {
  int num_cols_of_B = inverse_of_G_.SizeOfSystem();
  return SparseTransposeProduct(non_zero_columns_of_B_, entries_of_B_,
                                num_cols_of_B, residual);
}

Eigen::VectorXd T::SchurComplementConjugateGradientSolver(
    const Eigen::VectorXd& residual) {
  ConstrainedLeastSquaresConjugateGradientSolverConfig config;
  auto f = [this](const VectorXd& s) -> VectorXd {
    return EvaluateEquationOperator(
        inverse_of_G_.Solve(EvaluateEquationOperatorTranspose(s)));
  };

  int num_rows = residual.rows();
  VectorXd s(num_rows);
  s.setZero();
  {
    VectorXd r(num_rows);
    r = residual;
    VectorXd p(num_rows);
    p = r;

    for (int i = 0; i < config.iteration_limit; i++) {
      double norm_sqr_r = r.dot(r);
      double alpha = norm_sqr_r / p.dot(f(p));
      s += alpha * p;
      r -= alpha * f(p);
      double beta = r.dot(r) / norm_sqr_r;
      p = r + beta * p;
      std::cout << "norm: " << r.norm() << std::endl;
      if (r.norm() < 1e-8) {
        std::cout << "\nTerminating. ";
        break;
      }
    }
  }
  return s;
}

void T::Solve(const VectorXd& f, const VectorXd& g, VectorXd* y, VectorXd* z,
              bool use_llt) {
  CONEX_DEMAND(factored_, "System has not been factored.");
  VectorXd Ginv_f = inverse_of_G_.Solve(f);
  int num_columns_of_B = f.rows();
  MatrixXd B =
      MakeDenseMatrix(non_zero_columns_of_B_, entries_of_B_, num_columns_of_B);
  MatrixXd Ginv_Bt(B.cols(), B.rows());
  for (int i = 0; i < B.rows(); i++) {
    Ginv_Bt.col(i) = inverse_of_G_.Solve(B.row(i).transpose());
  }

  VectorXd schur_complement_residual =
      SparseMatrixProduct(non_zero_columns_of_B_, entries_of_B_, Ginv_f) - g;

  if (use_llt) {
    MatrixXd schur_complement =
        SparseMatrixProduct(non_zero_columns_of_B_, entries_of_B_, Ginv_Bt);
    Eigen::LLT<MatrixXd> llt(schur_complement);
    CONEX_DEMAND(llt.info() == Eigen::Success, "LLT Factorization failed.");
    *z = llt.solve(schur_complement_residual);
  } else {
    *z = SchurComplementConjugateGradientSolver(schur_complement_residual);
  }

  *y = inverse_of_G_.Solve(f - SparseTransposeProduct(non_zero_columns_of_B_,
                                                      entries_of_B_,
                                                      num_columns_of_B, *z));
}

MatrixXd T::KKTMatrix() {
  CONEX_DEMAND(assembled_,
               "System has not been assembled or is factored in place.");
  MatrixXd G = inverse_of_G_.KKTMatrix();
  MatrixXd B = MakeDenseMatrix(non_zero_columns_of_B_, entries_of_B_, G.cols());
  int dim = G.rows() + B.rows();
  MatrixXd M(dim, dim);
  M << G, B.transpose(), B, MatrixXd::Zero(B.rows(), B.rows());
  return M;
}
}  // namespace conex
