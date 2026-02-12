#include "conex/hermitian_psd.h"

#include "conex/conex.h"
#include "conex/error_checking_macros.h"
#include "conex/exponential_map.h"

namespace conex {

using Eigen::MatrixXd;

template <typename T>
bool HermitianPsdConstraint<T>::TakeStepImpl(const StepOptions& opt) {
  auto& ws = WS;
  ws.at(0).diagonal().array() += opt.e_weight;
  double scale = opt.step_size;
  if (scale != 1.0) {
    ws = T::ScalarMultiply(ws, scale);
  }
  int n = rank_;

  if (opt.step_type == CONEX_STEP_TYPE_DUAL_BARRIER) {
    auto WSW = T::Zero(n, n);
    WSW = T::Multiply(ws, W);
    if (opt.e_weight != 0) {
      W = T::ScalarMultiply(W, 1 + opt.e_weight);
    }
    W = T::Add(W, WSW);
  } else {
    int n = rank_;
    auto expWS = T::Zero(n, n);
    ExponentialMap(ws, &expWS);
    W = T::Multiply(expWS, W);
    W = T::ScalarMultiply(T::Add(W, T::ConjugateTranspose(W)), .5);
  }

  // TODO(FrankPermenter): Remove this hack, which provides
  // the dual-variable-interface access to real part of W.
  if (W.at(0).data() != workspace_.W.data()) {
    new (&workspace_.W)
        Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>(W.at(0).data(), n, n);
  }
  return true;
}

template <typename T>
void HermitianPsdConstraint<T>::PrepareStepImpl(const StepOptions& opt,
                                                const Ref& y, StepInfo* info) {
  auto& minus_s_local = minus_s;
  auto& ws = WS;
  ComputeNegativeSlack(opt.c_weight, y, &minus_s_local);

  ws = T::Multiply(W, minus_s_local);
  int n = rank_;

  auto gw_eig = T::ApproximateEigenvalues(ws, W, T::Random(n, 1), n / 2 + 1);
  const double lambda_1 = std::fabs(opt.e_weight + gw_eig.minCoeff());
  const double lambda_2 = std::fabs(opt.e_weight + gw_eig.maxCoeff());
  double norminf = lambda_1;
  if (norminf < lambda_2) {
    norminf = lambda_2;
  }

  auto WSWS = T::Multiply(ws, ws);

  info->norminfd = norminf;
  info->normsqrd = WSWS.at(0).trace() + 2 * ws.at(0).trace() + rank_;
}

template <typename T>
void HermitianPsdConstraint<T>::GetWeightedSlackEigenvaluesImpl(const Ref& y,
                                                                double c_weight, WeightedSlackEigenvalues* p) {
  typename T::Matrix minus_s;
  ComputeNegativeSlack(c_weight, y, &minus_s);

  int n = rank_;
  auto WS = T::Multiply(W, minus_s);
  auto gw_eig = T::ApproximateEigenvalues(WS, W, T::Random(n, 1), n / 2 + 1);

  const double lambda_max = -gw_eig.minCoeff();
  const double lambda_min = -gw_eig.maxCoeff();

  p->lambda_max = lambda_max;
  p->lambda_min = lambda_min;
  auto WSWS = T::Multiply(WS, WS);
  p->frobenius_norm_squared = WSWS.at(0).trace();
  p->trace = -WS.at(0).trace();
}

template void HermitianPsdConstraint<Real>::PrepareStepImpl(
    const StepOptions& opt, const Ref& y, StepInfo* info);
template void HermitianPsdConstraint<Complex>::PrepareStepImpl(
    const StepOptions& opt, const Ref& y, StepInfo* info);
template void HermitianPsdConstraint<Quaternions>::PrepareStepImpl(
    const StepOptions& opt, const Ref& y, StepInfo* info);

template bool HermitianPsdConstraint<Real>::TakeStepImpl(const StepOptions& opt);
template bool HermitianPsdConstraint<Complex>::TakeStepImpl(
    const StepOptions& opt);
template bool HermitianPsdConstraint<Quaternions>::TakeStepImpl(
    const StepOptions& opt);

template void HermitianPsdConstraint<Real>::GetWeightedSlackEigenvaluesImpl(
    const Ref& y, double c_weight, WeightedSlackEigenvalues* p);
template void HermitianPsdConstraint<Complex>::GetWeightedSlackEigenvaluesImpl(
    const Ref& y, double c_weight, WeightedSlackEigenvalues* p);
template void HermitianPsdConstraint<Quaternions>::GetWeightedSlackEigenvaluesImpl(
    const Ref& y, double c_weight, WeightedSlackEigenvalues* p);

template <>
bool HermitianPsdConstraint<Octonions>::TakeStepImpl(const StepOptions& opt) {
  using T = Octonions;
  auto& minus_s_local = minus_s;
  double scale = opt.step_size;

  if (scale != 1) {
    minus_s_local = T::ScalarMultiply(minus_s_local, scale);
  }

  if (opt.step_type == CONEX_STEP_TYPE_DUAL_BARRIER) {
    auto WSW = T::QuadraticRepresentation(W, minus_s_local);
    if (opt.e_weight != 0) {
      W = T::ScalarMultiply(W, 1 + opt.e_weight);
    }
    W = T::Add(W, WSW);
  } else {
    CONEX_DEMAND(opt.step_type == CONEX_STEP_TYPE_GEODESIC,
                 "Invalid step type");
    W = GeodesicUpdateScaled(W, minus_s_local);
  }
  return true;
}

template <>
void HermitianPsdConstraint<Octonions>::PrepareStepImpl(const StepOptions& opt,
                                                        const Ref& y, StepInfo* info) {
  using T = Octonions;
  auto& minus_s_local = minus_s;
  ComputeNegativeSlack(opt.c_weight, y, &minus_s_local);

  // || e - Q(w^{1/2}) s\|
  double minus_trace_ws = T::TraceInnerProduct(W, minus_s_local);
  info->normsqrd =
      T::TraceInnerProduct(T::QuadraticRepresentation(W, minus_s_local),
                           minus_s_local) +
      2 * minus_trace_ws + rank_;

  // TODO(FrankPermenter): replace this heuristic approximation.
  info->norminfd = std::sqrt(info->normsqrd);
}

template <>
void HermitianPsdConstraint<Octonions>::GetWeightedSlackEigenvaluesImpl(
    const Ref& y, double c_weight, WeightedSlackEigenvalues* p) {
  using T = Octonions;
  typename T::Matrix minus_s;
  ComputeNegativeSlack(c_weight, y, &minus_s);

  double normsqrd =
      T::TraceInnerProduct(T::QuadraticRepresentation(W, minus_s), minus_s);

  // Heuristic approximation based off of inequality:  |x|_1 |x|_{\infty} >=
  // |x|^2_2.
  p->lambda_max = std::fabs(normsqrd) /
                  (1e-15 + std::fabs(T::TraceInnerProduct(W, minus_s)));

  // Heuristic.
  p->lambda_min = p->lambda_max * .01;
  p->trace = -T::TraceInnerProduct(W, minus_s);
  p->frobenius_norm_squared =
      T::TraceInnerProduct(T::QuadraticRepresentation(W, minus_s), minus_s);
}

template <typename T>
void HermitianPsdConstraint<T>::ConstructSchurComplementSystemImpl(
    bool initialize, SchurComplementSystem* sys) {
  auto G = &sys->G;
  auto& w = W;
  int m = constraint_matrices_.size();

  typename T::Matrix AW;
  typename T::Matrix WAW;
  if (initialize) {
    for (int i = 0; i < m; i++) {
      if constexpr (std::is_same<T, Octonions>::value) {
        WAW = T::QuadraticRepresentation(w, constraint_matrices_.at(i));
      } else {
        AW = T::Multiply(constraint_matrices_.at(i), w);
        WAW = T::Multiply(w, AW);
      }
      for (int j = i; j < m; j++) {
        (*G)(j, i) = EvalDualConstraint(j, WAW);
      }
      if constexpr (std::is_same<T, Octonions>::value) {
        sys->AW(i, 0) = EvalDualConstraint(i, w);
      } else {
        sys->AW(i, 0) = AW.at(0).trace();
      }
      sys->AQc(i, 0) = EvalDualObjective(WAW);
    }
    sys->inner_product_of_w_and_c = 0;
  } else {
    for (int i = 0; i < m; i++) {
      if constexpr (std::is_same<T, Octonions>::value) {
        WAW = T::QuadraticRepresentation(w, constraint_matrices_.at(i));
      } else {
        AW = T::Multiply(constraint_matrices_.at(i), w);
        WAW = T::Multiply(w, AW);
      }

      for (int j = i; j < m; j++) {
        (*G)(j, i) += EvalDualConstraint(j, WAW);
      }

      if constexpr (std::is_same<T, Octonions>::value) {
        sys->AW(i, 0) += EvalDualConstraint(i, w);
      } else {
        sys->AW(i, 0) += AW.at(0).trace();
      }
      sys->AQc(i, 0) += EvalDualObjective(WAW);
    }
  }
  sys->inner_product_of_w_and_c += EvalDualObjective(w);

  // Reuse memory.
  auto& WCW = WAW;
  WCW = T::QuadraticRepresentation(w, constraint_affine_);
  if (initialize) {
    sys->inner_product_of_c_and_Qc = EvalDualObjective(WCW);
  } else {
    sys->inner_product_of_c_and_Qc += EvalDualObjective(WCW);
  }
}

template void HermitianPsdConstraint<Real>::ConstructSchurComplementSystemImpl(
    bool initialize, SchurComplementSystem* sys);

template void HermitianPsdConstraint<Complex>::ConstructSchurComplementSystemImpl(
    bool initialize, SchurComplementSystem* sys);

template void HermitianPsdConstraint<Quaternions>::ConstructSchurComplementSystemImpl(
    bool initialize, SchurComplementSystem* sys);

template void HermitianPsdConstraint<Octonions>::ConstructSchurComplementSystemImpl(
    bool initialize, SchurComplementSystem* sys);

template <typename H>
CONEX_STATUS HermitianPsdConstraint<H>::UpdateLinearOperatorImpl(
    double val, int var, int r, int c, int dim) {
  CONEX_RETURN_ON_FAIL(dim < H::HyperComplexDimension(),
                       "Complex dimension out of bounds.");
  CONEX_RETURN_ON_FAIL(r < rank_ && c < rank_,
                       "Matrix dimension out of bounds.");
  CONEX_RETURN_ON_FAIL(!(val != 0 && r == c && dim > 0),
                       "Imaginary components must be skew-symmetric.");

  using T = HermitianPsdConstraint<H>;
  if constexpr (std::is_same<T, Octonions>::value) {
    if (dim >= 3) {
      return false;
    }
  }

  constraint_matrices_.at(var).at(dim)(r, c) = val;
  if (dim == 0) {
    constraint_matrices_.at(var).at(dim)(c, r) = val;
  } else {
    constraint_matrices_.at(var).at(dim)(c, r) = -val;
  }
  return CONEX_SUCCESS;
}

template CONEX_STATUS HermitianPsdConstraint<Complex>::UpdateLinearOperatorImpl(
    double val, int var, int r, int c, int dim);
template CONEX_STATUS HermitianPsdConstraint<Real>::UpdateLinearOperatorImpl(
    double val, int var, int r, int c, int dim);
template CONEX_STATUS HermitianPsdConstraint<Quaternions>::UpdateLinearOperatorImpl(
    double val, int var, int r, int c, int dim);
template CONEX_STATUS HermitianPsdConstraint<Octonions>::UpdateLinearOperatorImpl(
    double val, int var, int r, int c, int dim);

template <typename H>
CONEX_STATUS HermitianPsdConstraint<H>::UpdateAffineTermImpl(
    double val, int r, int c, int dim) {
  CONEX_RETURN_ON_FAIL(dim < H::HyperComplexDimension(),
                       "Complex dimension out of bounds.");
  CONEX_RETURN_ON_FAIL(r < rank_ && c < rank_,
                       "Matrix dimension out of bounds.");
  CONEX_RETURN_ON_FAIL(!(val != 0 && r == c && dim > 0),
                       "Imaginary components must be skew-symmetric.");

  using T = HermitianPsdConstraint<H>;
  if constexpr (std::is_same<T, Octonions>::value) {
    if (dim >= 3) {
      return false;
    }
  }

  if (constraint_affine_.size() == 0) {
    constraint_affine_ = H::Zero(rank_, rank_);
  }

  constraint_affine_.at(dim)(r, c) = val;
  if (dim == 0) {
    constraint_affine_.at(dim)(c, r) = val;
  } else {
    constraint_affine_.at(dim)(c, r) = -val;
  }

  return CONEX_SUCCESS;
}

template CONEX_STATUS HermitianPsdConstraint<Complex>::UpdateAffineTermImpl(
    double val, int r, int c, int dim);
template CONEX_STATUS HermitianPsdConstraint<Real>::UpdateAffineTermImpl(
    double val, int r, int c, int dim);
template CONEX_STATUS HermitianPsdConstraint<Quaternions>::UpdateAffineTermImpl(
    double val, int r, int c, int dim);
template CONEX_STATUS HermitianPsdConstraint<Octonions>::UpdateAffineTermImpl(
    double val, int r, int c, int dim);

}  // namespace conex
