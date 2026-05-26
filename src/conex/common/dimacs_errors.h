#pragma once
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Sparse>

#include "conex/common/model.h"
#include "conex/common/solve_result.h"

namespace conex {

// Compute DIMACS-like normalized errors from a Model and a candidate solution,
// recomputing all residuals independently from the raw problem data.
//
// Inputs:
//   model      — the original optimization problem
//   x          — primal solution (num_variables)
//   lambda     — dual multipliers for inequality constraints (Linear/SOC/Barrier),
//                matched by insertion order in Model::constraints()
//   nu         — dual multipliers for equality constraints
//   psd_lambda — dual matrices for PSD constraints
//
// Errors:
//   dual_err   = ||c + Qx - A'λ - C'ν - Σ tr(A_k·Λ)|| / max(1, ||c|| + ||Qx||)
//   eq_err     = max_k ||C_k x - d_k|| / max(1, max_k ||d_k||)
//   compl_err  = |Σ <s_i, λ_i> + Σ tr(S_j·Λ_j)| / max(1, |c'x + ½x'Qx|)
//   prim_err   = max(0, -min(s_i), -min_eig(S_j)) across all constraints
//
inline DimacsErrors ComputeDimacsErrors(
    const Model& model,
    const Eigen::VectorXd& x,
    const std::vector<Eigen::VectorXd>& lambda,
    const std::vector<Eigen::VectorXd>& nu,
    const std::vector<Eigen::MatrixXd>& psd_lambda) {
  const int n = model.num_variables();

  // Cost vector.
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n);
  if (model.has_linear_cost()) {
    const auto& lc = model.linear_cost();
    c.head(std::min(n, (int)lc.size())) = lc.head(std::min(n, (int)lc.size()));
  }

  // Stationarity gradient: starts as c, subtract A'λ + C'ν + tr(A_k·Λ), add Qx.
  Eigen::VectorXd grad = c;
  double c_norm = c.norm();
  double Qx_norm = 0;
  double objective = c.dot(x);

  int ineq_idx = 0;
  int eq_idx = 0;
  int psd_idx = 0;
  double max_d_norm = 0;
  double eq_err = 0;
  double compl_gap = 0;
  double min_slack = std::numeric_limits<double>::infinity();

  for (const auto& cdata : model.constraints()) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        Eigen::VectorXd xv(data.vars.size());
        for (int j = 0; j < (int)data.vars.size(); ++j)
          xv(j) = x(data.vars[j]);
        Eigen::VectorXd slack = data.A * xv + data.b;
        min_slack = std::min(min_slack, slack.minCoeff());

        if (ineq_idx < (int)lambda.size()) {
          const auto& lam = lambda[ineq_idx];
          compl_gap += slack.dot(lam);
          Eigen::VectorXd Atl = data.A.transpose() * lam;
          for (int j = 0; j < (int)data.vars.size(); ++j)
            grad(data.vars[j]) -= Atl(j);
        }
        ineq_idx++;

      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        Eigen::VectorXd xv(data.vars.size());
        for (int j = 0; j < (int)data.vars.size(); ++j)
          xv(j) = x(data.vars[j]);
        Eigen::VectorXd slack = data.A * xv + data.b;
        if (slack.size() > 1) {
          double s0 = slack(0);
          double s1_norm = slack.tail(slack.size() - 1).norm();
          min_slack = std::min(min_slack, s0 - s1_norm);
        }

        if (ineq_idx < (int)lambda.size()) {
          const auto& lam = lambda[ineq_idx];
          compl_gap += slack.dot(lam);
          Eigen::VectorXd Atl = data.A.transpose() * lam;
          for (int j = 0; j < (int)data.vars.size(); ++j)
            grad(data.vars[j]) -= Atl(j);
        }
        ineq_idx++;

      } else if constexpr (std::is_same_v<T, Model::BarrierConstraintData>) {
        Eigen::VectorXd xv(data.vars.size());
        for (int j = 0; j < (int)data.vars.size(); ++j)
          xv(j) = x(data.vars[j]);
        Eigen::VectorXd slack = data.A * xv + data.b;
        min_slack = std::min(min_slack, slack.minCoeff());

        if (ineq_idx < (int)lambda.size()) {
          const auto& lam = lambda[ineq_idx];
          compl_gap += slack.dot(lam);
          Eigen::VectorXd Atl = data.A.transpose() * lam;
          for (int j = 0; j < (int)data.vars.size(); ++j)
            grad(data.vars[j]) -= Atl(j);
        }
        ineq_idx++;

      } else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        Eigen::VectorXd xv(data.primal_vars.size());
        for (int j = 0; j < (int)data.primal_vars.size(); ++j)
          xv(j) = x(data.primal_vars[j]);
        Eigen::VectorXd res = data.C * xv - data.d;
        eq_err = std::max(eq_err, res.norm());
        max_d_norm = std::max(max_d_norm, data.d.norm());

        if (eq_idx < (int)nu.size()) {
          Eigen::VectorXd Ctnu = data.C.transpose() * nu[eq_idx];
          for (int j = 0; j < (int)data.primal_vars.size(); ++j)
            grad(data.primal_vars[j]) -= Ctnu(j);
        }
        eq_idx++;

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        Eigen::VectorXd xv(data.vars.size());
        for (int j = 0; j < (int)data.vars.size(); ++j)
          xv(j) = x(data.vars[j]);
        Eigen::VectorXd Qxv = data.Q_sparse * xv;
        Qx_norm += Qxv.norm();
        objective += 0.5 * xv.dot(Qxv);
        for (int j = 0; j < (int)data.vars.size(); ++j)
          grad(data.vars[j]) += Qxv(j);

      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        // Slack: S = Σ A_k x_k + B
        const int dim = data.B.rows();
        Eigen::MatrixXd S = Eigen::MatrixXd(data.B);
        for (int k = 0; k < (int)data.vars.size(); ++k)
          S += x(data.vars[k]) * Eigen::MatrixXd(data.A_list[k]);

        // Primal feasibility: min eigenvalue of S.
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S, Eigen::EigenvaluesOnly);
        min_slack = std::min(min_slack, es.eigenvalues().minCoeff());

        if (psd_idx < (int)psd_lambda.size()) {
          const auto& Lam = psd_lambda[psd_idx];
          // Complementarity: tr(S · Λ)
          compl_gap += (S.array() * Lam.array()).sum();
          // Stationarity: grad_k -= tr(A_k · Λ)
          for (int k = 0; k < (int)data.vars.size(); ++k) {
            Eigen::MatrixXd Ak = Eigen::MatrixXd(data.A_list[k]);
            grad(data.vars[k]) -= (Ak.array() * Lam.array()).sum();
          }
        }
        psd_idx++;
      }
    }, cdata);
  }

  DimacsErrors e;
  e.dual_err = grad.norm() / std::max(1.0, c_norm + Qx_norm);
  e.eq_err = eq_err / std::max(1.0, max_d_norm);
  e.compl_err = std::abs(compl_gap) / std::max(1.0, std::abs(objective));
  e.prim_err = (min_slack < std::numeric_limits<double>::infinity())
                   ? std::max(0.0, -min_slack) : 0.0;

  return e;
}

// Convenience: extract all duals from a SolveResult.
inline DimacsErrors ComputeDimacsErrors(const Model& model,
                                        const SolveResult& result) {
  return ComputeDimacsErrors(model, result.x,
                             result.duals.lambda, result.duals.nu,
                             result.duals.psd_lambda);
}

}  // namespace conex
