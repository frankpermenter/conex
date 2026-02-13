#pragma once
#include "conex/kkt_subsystem.h"

namespace conex {

#define CONEX_NOOP(x) (void)x;
using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
constexpr bool ClassSupportSymmetricFactorization() {
  return false;
}

template <>
constexpr bool
ClassSupportSymmetricFactorization<Eigen::LLT<Eigen::MatrixXd>>() {
  return true;
}

template <>
constexpr bool
ClassSupportSymmetricFactorization<Eigen::LLT<Eigen::Ref<MatrixXd>>>() {
  return true;
}

template <typename FactorizationMethod, bool schur_complement_mode>
class CholeskySolver : public KKTSubsystemBase {
  static_assert(schur_complement_mode ||
                    ClassSupportSymmetricFactorization<FactorizationMethod>(),
                "Invalid template parameters. Must use schur complement mode "
                "if symmetric factorization is not supported.");

 public:
  CholeskySolver(Eigen::Ref<Eigen::MatrixXd> supernode_submatrix,
                 Eigen::Ref<Eigen::MatrixXd> separator_rows,
                 Eigen::Ref<Eigen::MatrixXd> separator_schur_complement)
      : supernode_submatrix_(supernode_submatrix),
        separator_rows_(separator_rows),
        separator_schur_complement_(separator_schur_complement) {}

  Eigen::Ref<Eigen::MatrixXd> supernode_submatrix() override {
    return supernode_submatrix_;
  }
  Eigen::Ref<Eigen::MatrixXd> separator_schur_complement() override {
    return separator_schur_complement_;
  }
  Eigen::Ref<Eigen::MatrixXd> separator_rows() override {
    return separator_rows_;
  }

  Eigen::Ref<const Eigen::MatrixXd> supernode_submatrix() const override {
    return supernode_submatrix_;
  }
  Eigen::Ref<const Eigen::MatrixXd> separator_schur_complement()
      const override {
    return separator_schur_complement_;
  }
  Eigen::Ref<const Eigen::MatrixXd> separator_rows() const override {
    return separator_rows_;
  }

  void DoComputeSeparatorSchurComplement() override {
    if (temp_row_major_.size() == 0) {
      temp_row_major_.resize(separator_rows_.rows(), separator_rows_.cols());
    }
    if constexpr (!schur_complement_mode) {
      if (separator_rows_.size()) {
        temp_row_major_ = llt_->matrixL().solve(separator_rows_.transpose());
        int n = separator_schur_complement_.rows();
        int d = separator_rows_.cols();
        if (OnlyLowerTriangularPart(n, d)) {
          for (int j = 0; j < n; j++) {
            separator_schur_complement_.col(j).tail(n - j).noalias() -=
                temp_row_major_.rightCols(n - j).transpose() *
                temp_row_major_.col(j);
          }
        } else {
          separator_schur_complement_.noalias() -=
              separator_columns_.transpose() * separator_columns_;
        }
      }
    } else {
      if (temp_row_major_.size() == 0) {
        temp_row_major_.resize(separator_rows_.rows(), separator_rows_.cols());
      }
      if (separator_rows_.size()) {
        temp_row_major_ = llt_->solve(separator_rows_.transpose());
        int n = separator_schur_complement_.rows();
        int d = separator_rows_.cols();
        if (OnlyLowerTriangularPart(n, d)) {
          for (int j = 0; j < temp_row_major_.cols(); j++) {
            separator_schur_complement_.col(j).tail(n - j).noalias() -=
                separator_rows_.bottomRows(n - j) * temp_row_major_.col(j);
          }
        } else {
          separator_schur_complement_.noalias() -=
              separator_rows_ * temp_row_major_;
        }
      }
    }
  }

  bool DoEliminateSupernodeColumns() override {
    llt_ = std::make_unique<FactorizationMethod>(supernode_submatrix_);
    if (llt_->info() != Eigen::Success) {
      factored_ = false;
    } else {
      factored_ = true;
    }
    return factored_;
  }
#if 0
  void DoComputeSeparatorSchurComplement() override {
    separator_columns_ = separator_rows_.transpose();
    if (separator_rows_.size()) {
      llt_->matrixL().solveInPlace(separator_columns_);
      int n = separator_schur_complement_.rows();
      int d = separator_rows_.cols();
       if (OnlyLowerTriangularPart(n, d)) {
        for (int j = 0; j < n; j++) {
          separator_schur_complement_.col(j).tail(n - j).noalias() -= separator_columns_.rightCols(n - j).transpose() * separator_columns_.col(j);
        }
        } else {
          separator_schur_complement_.noalias() -= separator_columns_.transpose() * separator_columns_;
      }
    }
  }
#endif

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    CONEX_CHECK(factored_);
    if (llt_->info() != Eigen::Success) {
      throw std::runtime_error("Factorization failed.");
    }
    if constexpr (schur_complement_mode) {
      llt_->solveInPlace(y);
    } else {
      llt_->matrixL().solveInPlace(y);
    }
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    CONEX_CHECK(factored_);
    if constexpr (schur_complement_mode) {
      CONEX_NOOP(y);
      return;
    } else {
      llt_->matrixL().transpose().solveInPlace(y);
    }
  }

  bool OnlyLowerTriangularPart(int /*num_vectors*/,
                               int /*cost_of_inner_product*/) {
    return true;
    // return num_vectors * cost_of_inner_product > 100;
  }

  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> temp_row_major_;
  Eigen::Ref<Eigen::MatrixXd> supernode_submatrix_;
  Eigen::MatrixXd separator_columns_;
  Eigen::Ref<Eigen::MatrixXd> separator_rows_;
  Eigen::Ref<Eigen::MatrixXd> separator_schur_complement_;
  std::unique_ptr<FactorizationMethod> llt_;
  bool factored_ = false;
};

template <typename FactorizationType>
class KKTCholeskySystem : public KKTSubsystem {
 public:
  bool DoEliminateSupernodeColumns() override {
    return factorization_->DoEliminateSupernodeColumns();
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    factorization_->DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    factorization_->DoApplyInverseOfRightFactorOfSupernodeSubmatrix(y);
  }

  void DoComputeSeparatorSchurComplement() override {
    factorization_->DoComputeSeparatorSchurComplement();
  }

  void DoInitialize() override {
    KKTSubsystem::DoInitialize();
    factorization_ = std::make_unique<FactorizationType>(
        supernode_submatrix(), separator_rows(), separator_schur_complement());
  }

 protected:
  std::unique_ptr<FactorizationType> factorization_;
};

class LUSolver : public KKTSubsystem {
 public:
  bool DoEliminateSupernodeColumns() override {
    lu_.compute(supernode_submatrix());
    return lu_.determinant() != 0;
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    y = lu_.solve(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    CONEX_NOOP(y);
  }

  void DoComputeSeparatorSchurComplement() override {
    separator_schur_complement() -=
        separator_rows() * lu_.solve(separator_rows().transpose());
  }

  Eigen::PartialPivLU<Eigen::MatrixXd> lu_;
};

using LLTSolver =
    KKTCholeskySystem<CholeskySolver<Eigen::LLT<Eigen::MatrixXd>, false>>;
template <bool is_positive_definite>
using FactorizationMethod =
    typename std::conditional<is_positive_definite, LLTSolver, LUSolver>::type;

template <bool is_positive_definite>
class StaticSubsystem : public FactorizationMethod<is_positive_definite> {
  using Base = FactorizationMethod<is_positive_definite>;

 public:
  StaticSubsystem(Eigen::MatrixXd Q, std::vector<int> vars)
      : Base(vars), Q_(Q) {}
  void DoInitialize() override {
    Base::DoInitialize();
    int n1 = Base::supernode_submatrix().rows();
    int n2 = Base::separator_rows().rows();
    Q_in_elimination_order_.resize(n1 + n2, n1 + n2);
    AssignSubmatrix(Q_, Q_in_elimination_order_,
                    Base::variable_to_local_elimination_rank());
    DoAssemble();
  }

 private:
  bool DoIsValidLeaf() override { return Q_.diagonal().norm() > 0; }
  void DoAssemble() {
    int n1 = Base::supernode_submatrix().rows();
    int n2 = Base::separator_rows().rows();
    CONEX_DEMAND(n1 >= 0 && n2 >= 0, "Invalid block sizes.");
    CONEX_DEMAND(n1 <= Q_in_elimination_order_.rows() &&
                     n1 <= Q_in_elimination_order_.cols(),
                 "Invalid top-left block bounds.");
    CONEX_DEMAND(n2 <= Q_in_elimination_order_.rows() &&
                     n1 <= Q_in_elimination_order_.cols(),
                 "Invalid bottom-left block bounds.");
    CONEX_DEMAND(n2 <= Q_in_elimination_order_.rows() &&
                     n2 <= Q_in_elimination_order_.cols(),
                 "Invalid bottom-right block bounds.");
    Base::supernode_submatrix() = Q_in_elimination_order_.topLeftCorner(n1, n1);
    Base::separator_rows() = Q_in_elimination_order_.bottomLeftCorner(n2, n1);
    Base::separator_schur_complement() =
        Q_in_elimination_order_.bottomRightCorner(n2, n2);
  }

  void AssignSubmatrix(const Eigen::MatrixXd& source,
                       Eigen::Ref<Eigen::MatrixXd> destination,
                       const std::vector<int>& source_to_dest_index) {
    destination.setZero();
    for (int i = 0; i < source.rows(); i++) {
      for (int j = 0; j < source.cols(); j++) {
        destination(source_to_dest_index.at(i), source_to_dest_index.at(j)) =
            source(i, j);
      }
    }
  }

 private:
  Eigen::MatrixXd Q_in_elimination_order_;
  Eigen::MatrixXd Q_;
};

}  // namespace conex
