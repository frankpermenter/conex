#include "conex/dense_lmi_constraint.h"

namespace conex {

namespace {
using Eigen::MatrixXd;

template <bool sparse>
void MultByA(const Ref& x, Ref* Y, std::vector<MatrixXd> constraint_matrices,
             std::vector<int> variable = {}) {
  int i = 0;
  Y->setZero();
  for (const auto& matrix : constraint_matrices) {
    if constexpr (sparse) {
      (*Y) += x(variable.at(i)) * matrix;
    } else {
      (*Y) += x(i) * matrix;
    }
    i++;
  }
}
}  // namespace

void DenseLMIConstraint::ComputeNegativeSlack(double k, const Ref& y, Ref* s) {
  MultByA<false>(y, s, constraint_matrices_);
  (*s) -= k * (constraint_affine_);
}

void MatrixLMIConstraint::ComputeAW(int i, const Ref& W, Ref* AW, Ref* WAW) {
  auto& constraint_matrix = constraint_matrices_.at(i);
  AW->noalias() = constraint_matrix * W;
  WAW->noalias() = W * (*AW);
}

void MatrixLMIConstraint::ComputeWCW(const Ref& W, Ref* CW, Ref* WCW) {
  auto& constraint_matrix = constraint_affine_;
  CW->noalias() = constraint_matrix * W;
  WCW->noalias() = W * (*CW);
}
MatrixLMIConstraint::MatrixLMIConstraint(
    int n, const std::vector<DenseMatrix>& constraint_matrices,
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

double TraceInnerProduct(const Eigen::MatrixXd& X, const Ref& Y) {
  double val = 0;
  for (int i = 0; i < X.rows(); i++) {
    val += X.col(i).dot(Y.col(i));
  }
  return val;
}

double MatrixLMIConstraint::EvalDualConstraint(int j, const Ref& W) {
  const auto& constraint_matrix = constraint_matrices_.at(j);
  return TraceInnerProduct(constraint_matrix, W);
}

double MatrixLMIConstraint::EvalDualObjective(const Ref& W) {
  const auto& constraint_matrix = constraint_affine_;
  return TraceInnerProduct(constraint_matrix, W);
}

#define SCHUR_COMPLEMENT_FUNCTION(OP)                    \
  int n = ws->n_;                                        \
  Eigen::Map<Eigen::VectorXd> vectWAW(WAW.data(), n* n); \
  for (int i = 0; i < m; i++) {                          \
    ComputeAW(i, W, &AW, &WAW);                          \
    sys->G.row(i).head(i + 1) OP vectWAW.transpose() *   \
        constraint_matrices_vect_.leftCols(i + 1);       \
    sys->AW(i, 0) OP AW.trace();                         \
    sys->AQc(i, 0) OP EvalDualObjective(WAW);            \
  }                                                      \
  sys->inner_product_of_w_and_c OP EvalDualObjective(W); \
                                                         \
  auto& WCW = WAW;                                       \
  auto& CW = AW;                                         \
  ComputeWCW(W, &CW, &WCW);                              \
  sys->inner_product_of_c_and_Qc OP TraceInnerProduct(constraint_affine_, WCW);

void DenseLMIConstraint::ConstructSchurComplementSystemImpl(
    bool initialize, SchurComplementSystem* sys) {
  auto* ws = workspace();
  auto& W = ws->W;
  auto& AW = ws->temp_1;
  auto& WAW = ws->temp_2;
  int m = num_dual_constraints_;

  if (initialize) {
    SCHUR_COMPLEMENT_FUNCTION(=);
  } else {
    SCHUR_COMPLEMENT_FUNCTION(+=);
  }
}

void DenseLMIGramEvaluator::set_order(const std::vector<int>& perm) {
  if (order_set_) return;
  const int nn = A_vect_->rows();
  const int m = static_cast<int>(perm.size());
  A_vect_perm_.resize(nn, m);
  for (int i = 0; i < m; ++i) {
    A_vect_perm_.col(i) = A_vect_->col(perm[i]);
  }
  WAW_vect_perm_.resize(nn, m);
  order_set_ = true;
  update_weights();
}

void DenseLMIGramEvaluator::update_weights() {
  if (!order_set_) return;
  const auto& W = ws_->W;
  const int n = ws_->n_;
  const int m = A_vect_perm_.cols();
  Eigen::MatrixXd AW(n, n);
  for (int i = 0; i < m; ++i) {
    Eigen::Map<const Eigen::MatrixXd> A_i(A_vect_perm_.col(i).data(), n, n);
    Eigen::Map<Eigen::MatrixXd> WAW_i(WAW_vect_perm_.col(i).data(), n, n);
    AW.noalias() = A_i * W;
    WAW_i.noalias() = W * AW;
  }
}

void DenseLMIGramEvaluator::add_block(
    int row, int col, int rows, int cols,
    Eigen::Ref<Eigen::MatrixXd> dest) const {
  dest.noalias() += WAW_vect_perm_.middleCols(row, rows).transpose() *
                    A_vect_perm_.middleCols(col, cols);
}

void DenseLMIGramEvaluator::add_block_lower(
    int pos, int size, Eigen::Ref<Eigen::MatrixXd> dest) const {
  Eigen::MatrixXd block = WAW_vect_perm_.middleCols(pos, size).transpose() *
                          A_vect_perm_.middleCols(pos, size);
  dest.triangularView<Eigen::Lower>() += block;
}

}  // namespace conex
