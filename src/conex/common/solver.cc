#include "conex/common/solver.h"

#include <numeric>

#include "conex/common/eja_ops.h"
#include "conex/common/structural_rank.h"

namespace conex {

Solver::Solver() = default;
Solver::~Solver() = default;
Solver::Solver(Solver&&) noexcept = default;
Solver& Solver::operator=(Solver&&) noexcept = default;

Solver Solver::Build(const Model& model,
                     const SolverConfiguration& config) {
  auto [reduced, expansion] = RemoveStructuralRankDeficiency(model);
  Solver s;
  s.expansion_ = std::move(expansion);
  if (config.row_scale) {
    auto [scaled, row_scaling] = RowScaleModel(reduced);
    s.row_scaling_ = std::move(row_scaling);
    s.reduced_linear_cost_ = scaled.linear_cost();
    s.reduced_model_ = std::move(scaled);
  } else {
    s.reduced_linear_cost_ = reduced.linear_cost();
    s.reduced_model_ = std::move(reduced);
  }
  s.system_ = KKTSystem::Build(s.reduced_model_, config, &s.arena_);
  s.kkt_cursor_ = s.arena_.SaveCursor();
  return s;
}

Solver Solver::Build(const Model& model,
                     const TreeSpec& tree,
                     const SolverConfiguration& config) {
  Solver s;
  s.expansion_.original_n = model.num_variables();
  s.expansion_.col_map.resize(model.num_variables());
  std::iota(s.expansion_.col_map.begin(), s.expansion_.col_map.end(), 0);
  s.reduced_linear_cost_ = model.linear_cost();
  s.reduced_model_ = model;
  s.system_ = KKTSystem::Build(s.reduced_model_, tree, config, &s.arena_);
  s.kkt_cursor_ = s.arena_.SaveCursor();
  return s;
}

Solver Solver::Build(const Model& model,
                     const CliqueTree& tree,
                     const SolverConfiguration& config) {
  Solver s;
  s.expansion_.original_n = model.num_variables();
  s.expansion_.col_map.resize(model.num_variables());
  std::iota(s.expansion_.col_map.begin(), s.expansion_.col_map.end(), 0);
  s.reduced_linear_cost_ = model.linear_cost();
  s.reduced_model_ = model;
  s.system_ = KKTSystem::Build(s.reduced_model_, tree, config, &s.arena_);
  s.kkt_cursor_ = s.arena_.SaveCursor();
  return s;
}

Solver Solver::BuildDense(const Model& model) {
  TreeSpec tree;
  int clique = tree.AddClique();
  for (int i = 0; i < model.num_constraints(); ++i)
    tree.Assign(i, clique);
  return Build(model, tree);
}

bool Solver::AssembleAndFactor() {
  auto model = MakeCompiledModel();
  RowSpace W = model.AllocRowSpace();
  setOnes(W);
  model.SetScaling(W);
  return model.AssembleAndFactor();
}

std::vector<double> Solver::SolveLinearSystem(const std::vector<double>& rhs) {
  auto* k = kkt();
  // Scatter rhs into block form.
  Eigen::Map<const Eigen::VectorXd> rhs_eigen(rhs.data(), rhs.size());
  auto rhs_blk = k->MakeSolverRHS();
  rhs_blk = k->MakeBlockVariable(rhs_eigen);
  // Solve in place.
  k->SolveSolverRHS(rhs_blk);
  // Gather to dense in reduced space.
  int nr = k->number_of_variables();
  Eigen::VectorXd x_reduced(nr);
  rhs_blk.supernodes->GatherInto(x_reduced);
  // Expand to original variable space.
  Eigen::VectorXd x_full = expansion_.Expand(x_reduced);
  return std::vector<double>(x_full.data(), x_full.data() + x_full.size());
}

SolverRHS Solver::MakeCostRHS() {
  auto* k = kkt();
  auto rhs = k->MakeSolverRHS();
  if (reduced_linear_cost_.size() > 0) {
    rhs = k->MakeBlockVariable(reduced_linear_cost_);
  } else {
    rhs.SetZero();
  }
  return rhs;
}

double Solver::ComputeObjective(const SolverRHS& cost_rhs,
                                const Eigen::VectorXd& x_reduced) {
  auto* k = kkt();
  auto x_rhs = k->MakeSolverRHS();
  x_rhs = k->MakeBlockVariable(x_reduced);

  auto qx = k->MakeSolverRHS();
  qx.SetZero();
  k->AccumulateQx(x_rhs, qx);
  double obj = 0.5 * k->dot(x_rhs, qx);
  obj += x_rhs.dot(cost_rhs);
  return obj;
}

OptimalitySummary Solver::ComputeOptimality(
    const SolverRHS& cost_rhs,
    const Eigen::VectorXd& x_reduced,
    const RowSpace& lambda) {
  auto* k = kkt();
  auto x_rhs = k->MakeSolverRHS();
  x_rhs = k->MakeBlockVariable(x_reduced);

  // s = Ax + b.
  RowSpace s = k->MakeRowSpace();
  k->MultiplyA(x_rhs, s);
  s += k->GetAffineTerm();

  // Dual residual: A'λ - Qx - c.
  auto dual_rhs = k->MakeSolverRHS();
  dual_rhs.SetZero();
  k->AccumulateAtranspose(lambda, dual_rhs);
  auto qx = k->MakeSolverRHS();
  qx.SetZero();
  k->AccumulateQx(x_rhs, qx);
  dual_rhs -= qx;
  dual_rhs -= cost_rhs;
  int n = k->number_of_variables();
  Eigen::VectorXd dual_res(n);
  dual_rhs.supernodes->GatherInto(dual_res);

  OptimalitySummary opt;
  opt.dual_residual = dual_res.norm();
  opt.complementarity = dot(s, lambda);
  opt.min_slack = minEigenvalue(s);
  opt.min_dual = minEigenvalue(lambda);
  return opt;
}

ConstraintDuals Solver::ExtractDuals(
    const Eigen::VectorXd& x_reduced,
    const RowSpace& lambda,
    const SolverRHS& cost_rhs) {
  ConstraintDuals duals;
  auto* k = kkt();
  const int nc = reduced_model_.num_constraints();

  // Compute slacks via KKT (handles tree decomposition correctly).
  auto x_rhs = k->MakeSolverRHS();
  x_rhs = k->MakeBlockVariable(x_reduced);
  RowSpace slack_rs = k->MakeRowSpace();
  k->MultiplyA(x_rhs, slack_rs);
  slack_rs += k->GetAffineTerm();

  // Extract per-constraint slacks and lambdas using GatherConstraintRows,
  // which uses the assembler's row_map to undo tree decomposition.
  for (int i = 0; i < nc; ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        auto s_gathered = system_.GatherConstraintRows(i, slack_rs);
        auto l_gathered = system_.GatherConstraintRows(i, lambda);
        // Unscale if row scaling was applied.
        if (i < (int)row_scaling_.row_scale.size() &&
            row_scaling_.row_scale[i].size() > 0) {
          const auto& sc = row_scaling_.row_scale[i];
          duals.slack.push_back(s_gathered.cwiseProduct(sc));
          duals.lambda.push_back(l_gathered.cwiseQuotient(sc));
        } else {
          duals.slack.push_back(s_gathered);
          duals.lambda.push_back(l_gathered);
        }

      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        auto s_gathered = system_.GatherConstraintRows(i, slack_rs);
        auto l_gathered = system_.GatherConstraintRows(i, lambda);
        duals.slack.push_back(s_gathered);
        // SOC quadratic representation P(W) = 2W² introduces a factor of 2.
        duals.lambda.push_back(2.0 * l_gathered);

      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        int n = data.B.rows();
        auto lam_flat = system_.GatherConstraintRows(i, lambda);
        Eigen::Map<const Eigen::MatrixXd> lam_map(lam_flat.data(), n, n);
        duals.psd_lambda.push_back(Eigen::MatrixXd(lam_map));
        auto slack_flat = system_.GatherConstraintRows(i, slack_rs);
        Eigen::Map<const Eigen::MatrixXd> slack_map(slack_flat.data(), n, n);
        duals.psd_slack.push_back(Eigen::MatrixXd(slack_map));

      } else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        const auto& dual_vars = system_.dual_variables(i);
        Eigen::VectorXd nu(dual_vars.size());
        for (int j = 0; j < (int)dual_vars.size(); ++j) {
          int dv = dual_vars[j];
          nu(j) = (dv < x_reduced.size()) ? -x_reduced(dv) : 0;
        }
        duals.nu.push_back(nu);
        // Equality residual: Cx - d.
        Eigen::VectorXd xv(data.primal_vars.size());
        for (int j = 0; j < (int)data.primal_vars.size(); ++j)
          xv(j) = x_reduced(data.primal_vars[j]);
        duals.eq_residual.push_back(
            Eigen::MatrixXd(data.C) * xv - data.d);

      } else if constexpr (std::is_same_v<T, Model::BarrierConstraintData>) {
        auto s_gathered = system_.GatherConstraintRows(i, slack_rs);
        auto l_gathered = system_.GatherConstraintRows(i, lambda);
        duals.slack.push_back(s_gathered);
        duals.lambda.push_back(l_gathered);
      }
    }, reduced_model_.constraint(i));
  }

  // Stationarity gradient: c + Qx - A'λ - C'ν.
  // Computed via KKT operations (handles decomposition correctly).
  auto grad = k->MakeSolverRHS();
  grad = cost_rhs;
  auto qx = k->MakeSolverRHS();
  qx.SetZero();
  k->AccumulateQx(x_rhs, qx);
  grad += qx;
  auto at_lambda = k->MakeSolverRHS();
  at_lambda.SetZero();
  k->AccumulateAtranspose(lambda, at_lambda);
  grad -= at_lambda;
  // Subtract C'ν from the stationarity gradient.
  // We already extracted ν above; now compute C'ν directly and subtract.
  {
    Eigen::VectorXd Ctnu = Eigen::VectorXd::Zero(k->number_of_variables());
    int eq_idx = 0;
    for (int i = 0; i < nc; ++i) {
      std::visit([&](const auto& data) {
        using T2 = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T2, Model::EqualityConstraintData>) {
          const auto& nu_i = duals.nu[eq_idx];
          // C' * nu in Model variable space.
          Eigen::VectorXd Ct_nu_i = Eigen::MatrixXd(data.C).transpose() * nu_i;
          for (int j = 0; j < (int)data.primal_vars.size(); ++j) {
            Ctnu(data.primal_vars[j]) += Ct_nu_i(j);
          }
          eq_idx++;
        }
      }, reduced_model_.constraint(i));
    }
    // Pack into SolverRHS and subtract.
    auto ctnu_rhs = k->MakeSolverRHS();
    ctnu_rhs = k->MakeBlockVariable(Ctnu);
    grad -= ctnu_rhs;
  }
  int n = k->number_of_variables();
  duals.stationarity_gradient.resize(n);
  grad.supernodes->GatherInto(duals.stationarity_gradient);
  // Expand to Model space.
  duals.stationarity_gradient = ExpandSolution(duals.stationarity_gradient);

  return duals;
}

}  // namespace conex
