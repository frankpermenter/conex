#pragma once
#include "conex/kkt_subsystem.h"

namespace conex {

#define CONEX_NOOP(x) (void)x;
using Eigen::MatrixXd;
using Eigen::VectorXd;
template <typename FactorizationMethod, bool schur_complement_mode>
class CholeskySolver {
 public:
  CholeskySolver(Eigen::MatrixXd& supernode_submatrix,  
                 Eigen::MatrixXd& separator_rows,
                 Eigen::MatrixXd& separator_schur_complement) :  
                 supernode_submatrix_(supernode_submatrix),
                 separator_rows_(separator_rows),
                 separator_schur_complement_(separator_schur_complement),
                 llt_(supernode_submatrix.rows()) {}

  void DoEliminateSupernodeColumns() {
    Eigen::internal::set_is_malloc_allowed(false);
    llt_.compute(supernode_submatrix_);
    if (llt_.info() != Eigen::Success) {
      throw std::runtime_error("Factorization failed.");
    }
    factored_ = true;
    Eigen::internal::set_is_malloc_allowed(true);
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const {
    CONEX_CHECK(factored_);
    if (llt_.info() != Eigen::Success) {
      throw std::runtime_error("Factorization failed.");
    }
    if constexpr (schur_complement_mode) {
      llt_.solveInPlace(y);
    } else {
      llt_.matrixL().solveInPlace(y);
    }
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const {
    CONEX_CHECK(factored_);
    if constexpr (schur_complement_mode) {
      CONEX_NOOP(y);
      return;
    } else {
      llt_.matrixL().transpose().solveInPlace(y);
    }
  }

  void DoComputeSeparatorSchurComplement() {
    if (temp_row_major_.size() == 0) {
      temp_row_major_.resize(separator_rows_.rows(), separator_rows_.cols());
    } 
    Eigen::internal::set_is_malloc_allowed(false);
    if (separator_rows_.size()) {
      temp_row_major_ = llt_.solve(separator_rows_.transpose());
      int n = separator_schur_complement_.rows();
      int d = separator_rows_.cols();
       if (OnlyLowerTriangularPart(n, d)) {
        for (int j = 0; j < temp_row_major_.cols(); j++) {
          separator_schur_complement_.col(j).tail(n - j).noalias() -= separator_rows_.bottomRows(n - j) * temp_row_major_.col(j);
        }
        } else {
          separator_schur_complement_.noalias() -= separator_rows_ * temp_row_major_;
      }
    }
    Eigen::internal::set_is_malloc_allowed(true);
  }

  bool OnlyLowerTriangularPart(int num_vectors, int cost_of_inner_product) {
    return true;
    //return num_vectors * cost_of_inner_product > 100; 
  }

  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> temp_row_major_;
  Eigen::Ref<Eigen::MatrixXd> supernode_submatrix_; 
  Eigen::Ref<Eigen::MatrixXd> separator_rows_; 
  Eigen::Ref<Eigen::MatrixXd> separator_schur_complement_; 
  FactorizationMethod llt_;
  bool factored_ = false;
};

} // namespace conex
