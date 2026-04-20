#pragma once
// Helper functions to compute KKT residuals against a Model + solution.
// Used by solver_solve_test.cc and the hybrid tuning program.

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include <vector>

#include "conex/common/model.h"

namespace conex {

struct KKTResiduals {
  double primal_infeasibility = 0;  // min slack (< 0 means infeasible)
  double equality_error = 0;        // max |Cx - d| across all equalities
  double dual_infeasibility = 0;    // min lambda (< 0 means infeasible)
  double complementarity = 0;       // sum |lambda_i . s_i|
  double stationarity = 0;          // ||c + Qx - A'lambda - C'nu||

  double MaxResidual() const {
    return std::max({-primal_infeasibility, equality_error,
                     -dual_infeasibility, complementarity, stationarity});
  }
};

// Compute KKT residuals for a linear-inequality-only problem:
//   min c'x + (1/2)x'Qx  s.t. A_i x + b_i >= 0
//
// model: the original Model (not reduced).
// x: primal solution in Model space.
// lambda: per-constraint dual vectors, in Model constraint order
//         (only for linear constraints; skip quadratic costs).
inline KKTResiduals ComputeLinearKKTResiduals(
    const Model& model,
    const Eigen::VectorXd& x,
    const std::vector<Eigen::VectorXd>& lambda) {
  KKTResiduals res;
  const int n = model.num_variables();

  // Accumulate stationarity: grad = c + Qx.
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(n);
  if (model.has_linear_cost()) grad += model.linear_cost();

  double min_slack = 1e30;
  double min_lambda = 1e30;
  double total_compl = 0;
  Eigen::VectorXd At_lambda = Eigen::VectorXd::Zero(n);

  int lam_idx = 0;
  for (int i = 0; i < model.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        // Slack: s = A * x[vars] + b.
        Eigen::VectorXd xv(data.vars.size());
        for (int j = 0; j < (int)data.vars.size(); ++j)
          xv(j) = x(data.vars[j]);
        Eigen::VectorXd s = data.A * xv + data.b;
        min_slack = std::min(min_slack, s.minCoeff());

        // Dual.
        const auto& lam = lambda[lam_idx];
        min_lambda = std::min(min_lambda, lam.minCoeff());
        total_compl += std::abs(lam.dot(s));

        // A' lambda contribution to stationarity.
        Eigen::VectorXd At_l = Eigen::MatrixXd(data.A).transpose() * lam;
        for (int j = 0; j < (int)data.vars.size(); ++j)
          At_lambda(data.vars[j]) += At_l(j);
        lam_idx++;

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        // Qx contribution to stationarity.
        for (int k = 0; k < data.Q_sparse.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.Q_sparse, k);
               it; ++it)
            grad(data.vars[it.row()]) += it.value() * x(data.vars[it.col()]);
      }
    }, model.constraint(i));
  }

  res.primal_infeasibility = min_slack;
  res.dual_infeasibility = min_lambda;
  res.complementarity = total_compl;
  res.stationarity = (grad - At_lambda).norm();
  return res;
}

}  // namespace conex
