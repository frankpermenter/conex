#pragma once
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Sparse>

#include "conex/common/model.h"
#include "conex/common/solve_result.h"

namespace conex {

// Compute DIMACS-like normalized errors from a Model and a candidate solution.
//
// When infeasible=false (optimal solution):
//   dual_err  = ||c + Qx - A'λ - C'ν|| / max(1, ||c|| + ||Qx||)
//   eq_err    = max ||Cx - d|| / max(1, max ||d||)
//   compl_err = |<s,λ>| / max(1, |objective|)    where s = Ax + b
//   prim_err  = max(0, -min(s))                   where s = Ax + b
//
// When infeasible=true (certificate ray, not divided by tau):
//   dual_err  = ||Qx - A'λ - C'ν|| / max(1, ||Qx||)   (no c)
//   eq_err    = max ||Cx|| / max(1, ||x||)              (no d)
//   compl_err = |<Ax, λ>| / max(1, ||x|| * ||λ||)      (no b)
//   prim_err  = max(0, -min(Ax))                        (no b)
//
inline DimacsErrors ComputeDimacsErrors(
    const Model& model,
    const Eigen::VectorXd& x,
    const std::vector<Eigen::VectorXd>& lambda,
    const std::vector<Eigen::VectorXd>& nu,
    const std::vector<Eigen::MatrixXd>& psd_lambda,
    bool infeasible = false) {
  const int n = model.num_variables();

  // Cost vector (omitted for infeasibility certificate).
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n);
  if (!infeasible && model.has_linear_cost()) {
    const auto& lc = model.linear_cost();
    c.head(std::min(n, (int)lc.size())) = lc.head(std::min(n, (int)lc.size()));
  }

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
  double min_dual = std::numeric_limits<double>::infinity();
  double total_lam_norm = 0;

  for (const auto& cdata : model.constraints()) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        Eigen::VectorXd xv(data.vars.size());
        for (int j = 0; j < (int)data.vars.size(); ++j)
          xv(j) = x(data.vars[j]);
        Eigen::VectorXd slack = data.A * xv;
        if (!infeasible) slack += data.b;
        min_slack = std::min(min_slack, slack.minCoeff());

        if (ineq_idx < (int)lambda.size()) {
          const auto& lam = lambda[ineq_idx];
          min_dual = std::min(min_dual, lam.minCoeff());
          total_lam_norm += lam.norm();
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
        Eigen::VectorXd slack = data.A * xv;
        if (!infeasible) slack += data.b;
        if (slack.size() > 1) {
          double s0 = slack(0);
          double s1_norm = slack.tail(slack.size() - 1).norm();
          min_slack = std::min(min_slack, s0 - s1_norm);
        }

        if (ineq_idx < (int)lambda.size()) {
          const auto& lam = lambda[ineq_idx];
          if (lam.size() > 1) {
            min_dual = std::min(min_dual, lam(0) - lam.tail(lam.size()-1).norm());
          }
          total_lam_norm += lam.norm();
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
        Eigen::VectorXd slack = data.A * xv;
        if (!infeasible) slack += data.b;
        min_slack = std::min(min_slack, slack.minCoeff());

        if (ineq_idx < (int)lambda.size()) {
          const auto& lam = lambda[ineq_idx];
          min_dual = std::min(min_dual, lam.minCoeff());
          total_lam_norm += lam.norm();
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
        Eigen::VectorXd res = data.C * xv;
        if (!infeasible) res -= data.d;
        eq_err = std::max(eq_err, res.norm());
        if (infeasible)
          max_d_norm = std::max(max_d_norm, x.norm());
        else
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
        Eigen::MatrixXd S = infeasible
            ? Eigen::MatrixXd::Zero(data.B.rows(), data.B.cols())
            : Eigen::MatrixXd(data.B);
        for (int k = 0; k < (int)data.vars.size(); ++k)
          S += x(data.vars[k]) * Eigen::MatrixXd(data.A_list[k]);

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S, Eigen::EigenvaluesOnly);
        min_slack = std::min(min_slack, es.eigenvalues().minCoeff());

        if (psd_idx < (int)psd_lambda.size()) {
          const auto& Lam = psd_lambda[psd_idx];
          Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> el(Lam, Eigen::EigenvaluesOnly);
          min_dual = std::min(min_dual, el.eigenvalues().minCoeff());
          compl_gap += (S.array() * Lam.array()).sum();
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
  if (infeasible) {
    double norm_prod = std::max(1.0, x.norm() * total_lam_norm);
    e.compl_err = std::abs(compl_gap) / norm_prod;
  } else {
    e.compl_err = std::abs(compl_gap) / std::max(1.0, std::abs(objective));
  }
  e.prim_err = (min_slack < std::numeric_limits<double>::infinity())
                   ? std::max(0.0, -min_slack) : 0.0;
  e.min_dual = (min_dual < std::numeric_limits<double>::infinity())
                   ? min_dual : 0.0;

  return e;
}

// Convenience: extract all duals from a SolveResult.
inline DimacsErrors ComputeDimacsErrors(const Model& model,
                                        const SolveResult& result) {
  return ComputeDimacsErrors(model, result.x,
                             result.duals.lambda, result.duals.nu,
                             result.duals.psd_lambda, result.infeasible);
}

}  // namespace conex
