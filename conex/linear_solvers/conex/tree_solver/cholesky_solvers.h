#pragma once
#include "conex/tree_solver/RLDLT.h"
#include "conex/tree_solver/kkt_subsystem.h"

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

  // No arena needed — memory is externally owned via Ref.
  size_t RequiredArenaBytes() const override { return 0; }
  void BindArenaMemory(double*, size_t) override {}

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
    const int sep = separator_rows_.rows();
    const int sn = separator_rows_.cols();
    if (temp_arena_ptr_) {
      // Use arena-backed buffers.
      auto temp = temp_map();
      if constexpr (!schur_complement_mode) {
        if (separator_rows_.size()) {
          temp = llt_->matrixL().solve(separator_rows_.transpose());
          int n = separator_schur_complement_.rows();
          if (OnlyLowerTriangularPart(n, sn)) {
            for (int j = 0; j < n; j++) {
              separator_schur_complement_.col(j).tail(n - j).noalias() -=
                  temp.rightCols(n - j).transpose() * temp.col(j);
            }
          } else {
            separator_schur_complement_.noalias() -=
                temp.transpose() * temp;
          }
          // Store S F^{-1} = S L^{-T} = (L^{-1} S^T)^T.
          auto cache = ColMajorMap(cache_arena_ptr_, arena_sep_, arena_sn_);
          cache = temp.transpose();
        }
      } else {
        if (separator_rows_.size()) {
          temp = llt_->solve(separator_rows_.transpose());
          int n = separator_schur_complement_.rows();
          if (OnlyLowerTriangularPart(n, sn)) {
            for (int j = 0; j < temp.cols(); j++) {
              separator_schur_complement_.col(j).tail(n - j).noalias() -=
                  separator_rows_.bottomRows(n - j) * temp.col(j);
            }
          } else {
            separator_schur_complement_.noalias() -=
                separator_rows_ * temp;
          }
        }
      }
    } else {
      // Fallback: use owned matrices.
      if (temp_row_major_.size() == 0) {
        temp_row_major_.resize(sep, sn);
      }
      if constexpr (!schur_complement_mode) {
        if (separator_rows_.size()) {
          temp_row_major_ = llt_->matrixL().solve(separator_rows_.transpose());
          int n = separator_schur_complement_.rows();
          if (OnlyLowerTriangularPart(n, sn)) {
            for (int j = 0; j < n; j++) {
              separator_schur_complement_.col(j).tail(n - j).noalias() -=
                  temp_row_major_.rightCols(n - j).transpose() *
                  temp_row_major_.col(j);
            }
          } else {
            separator_schur_complement_.noalias() -=
                separator_columns_.transpose() * separator_columns_;
          }
          schur_complement_factor_cached_ = temp_row_major_.transpose();
        }
      } else {
        if (temp_row_major_.size() == 0) {
          temp_row_major_.resize(sep, sn);
        }
        if (separator_rows_.size()) {
          temp_row_major_ = llt_->solve(separator_rows_.transpose());
          int n = separator_schur_complement_.rows();
          if (OnlyLowerTriangularPart(n, sn)) {
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
  }

  void DoMultiplyByCachedMatrix(
      Eigen::Ref<Eigen::MatrixXd> output,
      Eigen::Ref<const Eigen::MatrixXd> input) const {
    if (separators_.empty()) {
      output.setZero();
      return;
    }
    Eigen::Ref<Eigen::MatrixXd> gathered_separator_rows =
        ws3().topLeftCorner(static_cast<int>(separators_.size()),
                                        input.cols());
    for (int i = 0; i < gathered_separator_rows.rows(); ++i) {
      gathered_separator_rows.row(i) = input.row(separators_.at(i));
    }
    output.noalias() = cache_map().transpose() * gathered_separator_rows;
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

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    CONEX_CHECK(factored_);
    if (llt_->info() != Eigen::Success) {
      throw std::runtime_error("Factorization failed.");
    }
    if constexpr (schur_complement_mode) {
      llt_->solveInPlace(y);  // E^{-1} = A^{-1}.
    } else {
      if (y.cols() == 1) {
        llt_->matrixL().solveInPlace(y.col(0));  // dtrsv
      } else {
        llt_->matrixL().solveInPlace(y);  // dtrsm
      }
    }
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    CONEX_CHECK(factored_);
    if constexpr (schur_complement_mode) {
      CONEX_NOOP(y);  // F^{-1} = I.
      return;
    } else {
      if (y.cols() == 1) {
        llt_->matrixL().transpose().solveInPlace(y.col(0));  // dtrsv
      } else {
        llt_->matrixL().transpose().solveInPlace(y);  // dtrsm
      }
    }
  }


  // Schur complement mode (E=A, F=I): compute A^{-1} S^T x_sep using
  // original S in separator_rows_ (not cached).
  // Non-schur mode (LLT): default uses cached S L^{-T} in separator_rows_.
  void DoBackwardScatter(Eigen::Ref<MatrixXd> output,
                         Eigen::Ref<const MatrixXd> input) const override {
    if constexpr (schur_complement_mode) {
      KKTSubsystemBase::DoBackwardScatter(output, input);
    } else {
      //KKTSubsystemBase::DoBackwardScatter(output, input);
     // Use C
      //DoMultiplyByTransposeOfOffDiagonalSubMatrix(output, input);
      DoMultiplyByCachedMatrix(output, input);
    }
  }


  void DoBackwardScatterFromGatheredSeparator(
      Eigen::Ref<MatrixXd> output,
      Eigen::Ref<const MatrixXd> gathered_sep) const override {
    if constexpr (!schur_complement_mode) {
      // Cache stores S L^{-T}, so transpose gives L^{-1} S^T = E^{-1} S^T.
      output.noalias() = cache_map().transpose() * gathered_sep;
    } else {
      KKTSubsystemBase::DoBackwardScatterFromGatheredSeparator(output,
                                                               gathered_sep);
    }
  }

  bool OnlyLowerTriangularPart(int /*num_vectors*/,
                               int /*cost_of_inner_product*/) {
    return true;
    // return num_vectors * cost_of_inner_product > 100;
  }

  // Arena-backed factorization buffers (set by KKTCholeskySystem).
  double* temp_arena_ptr_ = nullptr;
  double* cache_arena_ptr_ = nullptr;
  int arena_sep_ = 0, arena_sn_ = 0;

  void BindFactorizationBuffers(double* temp, double* cache, int sep, int sn) {
    temp_arena_ptr_ = temp;
    cache_arena_ptr_ = cache;
    arena_sep_ = sep;
    arena_sn_ = sn;
  }

  using RowMajorMatrix = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;
  using RowMajorMap = Eigen::Map<RowMajorMatrix, Eigen::Aligned>;
  using ColMajorMap = Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>;

  RowMajorMap temp_map() {
    if (temp_arena_ptr_) return {temp_arena_ptr_, arena_sn_, arena_sep_};
    return {temp_row_major_.data(), temp_row_major_.rows(),
            temp_row_major_.cols()};
  }
  RowMajorMap temp_map() const {
    if (temp_arena_ptr_)
      return {const_cast<double*>(temp_arena_ptr_), arena_sn_, arena_sep_};
    return {const_cast<double*>(temp_row_major_.data()), temp_row_major_.rows(),
            temp_row_major_.cols()};
  }
  ColMajorMap cache_map() const {
    if (cache_arena_ptr_)
      return {const_cast<double*>(cache_arena_ptr_), arena_sep_, arena_sn_};
    return {const_cast<double*>(schur_complement_factor_cached_.data()),
            schur_complement_factor_cached_.rows(),
            schur_complement_factor_cached_.cols()};
  }

  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> temp_row_major_;
  Eigen::Ref<Eigen::MatrixXd> supernode_submatrix_;
  Eigen::MatrixXd separator_columns_;
  Eigen::MatrixXd schur_complement_factor_cached_;
  Eigen::Ref<Eigen::MatrixXd> separator_rows_;
  Eigen::Ref<Eigen::MatrixXd> separator_schur_complement_;
  std::unique_ptr<FactorizationMethod> llt_;
  bool factored_ = false;
};

// Factorization mode for DynamicSubsystem.
enum class IndefiniteFactorization { kRLDLT, kLU };

class DynamicSubsystem : public KKTSubsystem {
 public:
  void MarkIndefinite() override { indefinite_ = true; }
  bool is_indefinite() const { return indefinite_; }
  void SetIndefiniteFactorization(IndefiniteFactorization f) {
    indefinite_factorization_ = f;
  }

 private:
  bool use_lu() const {
    return indefinite_ &&
           indefinite_factorization_ == IndefiniteFactorization::kLU;
  }
  bool use_rldlt() const {
    return indefinite_ &&
           indefinite_factorization_ == IndefiniteFactorization::kRLDLT;
  }

  bool DoEliminateSupernodeColumns() override {
    if (use_lu()) {
      const int nr = supernode_submatrix().rows();
      if (nr == 0) return true;
      // Supernode submatrix is stored lower-triangular (symmetric).
      // LU needs the full symmetric matrix, copied to owned storage.
      MatrixXd sn_full(nr, nr);
      sn_full.triangularView<Eigen::Lower>() = supernode_submatrix();
      sn_full.triangularView<Eigen::StrictlyUpper>() =
          sn_full.transpose();
      MatrixXd sn_save = sn_full;
      lu_.compute(sn_full);
      // Check for singular or NaN blocks.
      double det = std::abs(lu_.determinant());
      if (!(det > 0)) {  // catches 0, NaN, -0
        fprintf(stderr, "LU singular: det=%.2e, size=%dx%d\n", det, nr, nr);
        Eigen::IOFormat fmt(6, 0, "  ", "\n", "  ", "");
        std::cerr << sn_save.format(fmt) << std::endl;
        return false;
      }
      return true;
    }
    if (use_rldlt()) {
      rldlt_.compute(supernode_submatrix());
      return rldlt_.info() == Eigen::Success;
    }
    llt_.compute(supernode_submatrix());
    return llt_.info() == Eigen::Success;
  }

  void DoComputeSeparatorSchurComplement() override {
    if (separator_rows().rows() == 0 || separator_rows().cols() == 0) return;
    const int sep = separator_rows().rows();
    if (use_lu()) {
      temp_ = lu_.solve(separator_rows().transpose());
      for (int j = 0; j < sep; j++) {
        separator_schur_complement().col(j).tail(sep - j).noalias() -=
            separator_rows().bottomRows(sep - j) * temp_.col(j);
      }
      return;
    }
    if (use_rldlt()) {
      temp_.noalias() = rldlt_.solve(separator_rows().transpose());
      for (int j = 0; j < sep; j++) {
        separator_schur_complement().col(j).tail(sep - j).noalias() -=
            separator_rows().bottomRows(sep - j) * temp_.col(j);
      }
      return;
    }
    temp_ = separator_rows().transpose();
    llt_.matrixL().solveInPlace(temp_);
    for (int j = 0; j < sep; j++) {
      separator_schur_complement().col(j).tail(sep - j).noalias() -=
          temp_.rightCols(sep - j).transpose() * temp_.col(j);
    }
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    if (y.rows() == 0) return;
    if (use_lu()) {
      y = lu_.solve(y);
    } else if (use_rldlt()) {
      y = rldlt_.solve(y);
    } else {
      llt_.matrixL().solveInPlace(y);
    }
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    if (y.rows() == 0) return;
    if (!indefinite_) {
      llt_.matrixL().transpose().solveInPlace(y);
    }
    // LU and RLDLT: right factor is identity (Schur complement mode).
  }

  bool indefinite_ = false;
  IndefiniteFactorization indefinite_factorization_ =
      IndefiniteFactorization::kRLDLT;
  Eigen::LLT<MatrixXd> llt_;
  Eigen::RLDLT<MatrixXd> rldlt_;
  Eigen::PartialPivLU<MatrixXd> lu_;
  MatrixXd temp_;
};


class WorkingLLTSubsystem : public KKTSubsystem {
 public:
  static size_t AlignUpBytes(size_t v) {
    constexpr size_t a = EIGEN_MAX_ALIGN_BYTES;
    return ((v + a - 1) / a) * a;
  }

  // Extra arena space for cached L^{-1} S^T (sn × sep).
  size_t RequiredArenaBytes() const override {
    size_t base = KKTSubsystem::RequiredArenaBytes();
    const size_t sn = supernodes_.size();
    const size_t sep = separators_.size();
    if (sn > 0 && sep > 0) {
      base = AlignUpBytes(base);
      base += AlignUpBytes(sn * sep * sizeof(double));
    }
    return base;
  }

  void BindArenaMemory(double* ptr, size_t bytes) override {
    size_t base_bytes = KKTSubsystem::RequiredArenaBytes();
    KKTSubsystem::BindArenaMemory(ptr, base_bytes);
    const int sn = static_cast<int>(supernodes_.size());
    const int sep = static_cast<int>(separators_.size());
    if (sn > 0 && sep > 0) {
      char* base = reinterpret_cast<char*>(ptr);
      size_t cursor = AlignUpBytes(base_bytes);
      temp_arena_ptr_ = reinterpret_cast<double*>(base + cursor);
      new (&temp_map_) Eigen::Map<MatrixXd, Eigen::Aligned>(
          temp_arena_ptr_, sn, sep);
      use_arena_temp_ = true;
    }
  }

  // No DoInitialize needed — temp map is created in BindArenaMemory,
  // which is also called on arena regrow.

 private:
  Eigen::Map<MatrixXd, Eigen::Aligned>& temp() {
    return use_arena_temp_ ? temp_map_ : owned_temp_map_;
  }
  const Eigen::Map<MatrixXd, Eigen::Aligned>& temp() const {
    return use_arena_temp_ ? temp_map_ : owned_temp_map_;
  }

  using MapType = DenseKKTSubsystemStorage::AlignedMatrixMap;
  using LLTType = Eigen::LLT<MapType>;

  bool DoEliminateSupernodeColumns() override {
    // Factor in-place: the arena-backed Map is overwritten with L.
    llt_ = std::make_unique<LLTType>(dense_storage().supernode_map());
    return llt_->info() == Eigen::Success;
  }

  void DoComputeSeparatorSchurComplement() override {
    if (separator_rows().rows() == 0 || separator_rows().cols() == 0) return;
    const int sep = separator_rows().rows();

    if (!use_arena_temp_) {
      temp_storage_.resize(separator_rows().cols(), sep);
      new (&owned_temp_map_) Eigen::Map<MatrixXd, Eigen::Aligned>(
          temp_storage_.data(), temp_storage_.rows(), temp_storage_.cols());
    }

    temp() = separator_rows().transpose();
    llt_->matrixL().solveInPlace(temp());
    for (int j = 0; j < sep; j++) {
      separator_schur_complement().col(j).tail(sep - j).noalias() -=
          temp().rightCols(sep - j).transpose() * temp().col(j);
    }
  }

  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    if (y.rows() == 0) return;
    llt_->matrixL().solveInPlace(y);
  }

  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<MatrixXd> y) const override {
    if (y.rows() == 0) return;
    llt_->matrixL().transpose().solveInPlace(y);
  }

  // Use cached L^{-1} S^T (computed in DoComputeSeparatorSchurComplement).
  void DoBackwardScatterFromGatheredSeparator(
      Eigen::Ref<MatrixXd> output,
      Eigen::Ref<const MatrixXd> gathered_sep) const override {
    output.noalias() = temp() * gathered_sep;
  }

  std::unique_ptr<LLTType> llt_;
  double* temp_arena_ptr_ = nullptr;
  bool use_arena_temp_ = false;
  Eigen::Map<MatrixXd, Eigen::Aligned> temp_map_{nullptr, 0, 0};
  // Fallback owned storage (used when arena not bound).
  MatrixXd temp_storage_;
  Eigen::Map<MatrixXd, Eigen::Aligned> owned_temp_map_{nullptr, 0, 0};
};









using LLTSolver = WorkingLLTSubsystem;

}  // namespace conex
