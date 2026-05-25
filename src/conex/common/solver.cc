#include "conex/common/solver.h"

#include <numeric>

#include "conex/common/eja_ops.h"
#include "conex/common/structural_rank.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

namespace conex {

Solver::Solver() = default;
Solver::~Solver() = default;
Solver::Solver(Solver&&) noexcept = default;
Solver& Solver::operator=(Solver&&) noexcept = default;

Solver Solver::Build(const Model& model,
                     const SolverConfiguration& config) {
  // Penalty formulation: lift equalities into objective before presolve.
  const Model* input = &model;
  Model penalized;
  Solver s;
  if (config.penalty_alpha > 0) {
    // Stash original equality constraints for post-solve dual reconstruction.
    for (int i = 0; i < model.num_constraints(); ++i) {
      auto* eq = std::get_if<Model::EqualityConstraintData>(&model.constraint(i));
      if (eq) {
        s.penalty_eqs_.push_back(
            {config.penalty_alpha, eq->C, eq->d, eq->primal_vars});
      }
    }
    penalized = LiftEqualitiesToPenalty(model, config.penalty_alpha);
    input = &penalized;
  }

  s.original_model_ = model;  // stash for Model-space residuals

  auto [reduced, expansion] = RemoveStructuralRankDeficiency(*input);
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
  s.original_model_ = model;
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
  s.original_model_ = model;
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


SolverRHS Solver::MakeCostRHS() {
  auto* k = kkt();
  auto rhs = k->MakeSolverRHS();
  if (reduced_linear_cost_.size() > 0) {
    rhs.ScatterFrom(reduced_linear_cost_.data(), reduced_linear_cost_.size());
  } else {
    rhs.SetZero();
  }
  return rhs;
}

double Solver::ComputeObjective(const SolverRHS& cost_rhs,
                                const Eigen::VectorXd& x_reduced) {
  auto* k = kkt();
  auto x_rhs = k->MakeSolverRHS();
  x_rhs.ScatterFrom(x_reduced.data(), x_reduced.size());

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
  x_rhs.ScatterFrom(x_reduced.data(), x_reduced.size());

  // s = Ax + b.
  RowSpace s = k->MakeRowSpace();
  k->MultiplyA(x_rhs, s);
  s += k->GetAffineTerm();

  // Dual residual: A'λ + C'ν - Qx - c.
  // Compute in dense space to avoid separator/supernode bookkeeping issues.
  int n = k->number_of_variables();

  auto at_lam = k->MakeSolverRHS();
  at_lam.SetZero();
  k->AccumulateAtranspose(lambda, at_lam);
  Eigen::VectorXd at_lam_vec(n);
  k->GatherInto(at_lam, at_lam_vec);

  auto qx = k->MakeSolverRHS();
  qx.SetZero();
  k->AccumulateQx(x_rhs, qx);
  Eigen::VectorXd qx_vec(n);
  k->GatherInto(qx, qx_vec);

  Eigen::VectorXd cost_vec(n);
  cost_rhs.supernodes->GatherInto(cost_vec);

  Eigen::VectorXd dual_res = at_lam_vec - qx_vec - cost_vec;

  // The equality dual ν is embedded in x_reduced via the saddle-point
  // formulation [0,C';C,0].  The saddle-point ν has opposite sign to
  // the Lagrangian ν, so we subtract AccumulateCtranspose.
  if (auto* ts = tree_solver()) {
    auto c_trans = k->MakeSolverRHS();
    c_trans.SetZero();
    ts->AccumulateCtranspose(x_rhs, c_trans);
    Eigen::VectorXd ct_vec(n);
    k->GatherInto(c_trans, ct_vec);
    dual_res -= ct_vec;
  }

  // Stationarity norm: only over primal variables (exclude equality dual slots).
  int n_primal = reduced_linear_cost_.size() > 0
                     ? static_cast<int>(reduced_linear_cost_.size())
                     : n;

  // Use raw Eigen operations for complementarity and min eigenvalue.
  // The EJA dispatch (dot, minEigenvalue) requires SymmetricConeOperations
  // which is not available for barrier cones (exp, power).
  Eigen::Map<const Eigen::VectorXd> s_vec(s.col().data(), s.col().size());
  Eigen::Map<const Eigen::VectorXd> l_vec(lambda.col().data(), lambda.col().size());

  OptimalitySummary opt;
  opt.dual_residual = dual_res.head(n_primal).norm();
  opt.complementarity = s_vec.dot(l_vec);
  opt.min_slack = s_vec.minCoeff();
  opt.min_dual = l_vec.minCoeff();
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
  x_rhs.ScatterFrom(x_reduced.data(), x_reduced.size());
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
    ctnu_rhs.ScatterFrom(Ctnu.data(), Ctnu.size());
    grad -= ctnu_rhs;
  }
  int n = k->number_of_variables();
  duals.stationarity_gradient.resize(n);
  grad.supernodes->GatherInto(duals.stationarity_gradient);
  // Expand to Model space.
  duals.stationarity_gradient = ExpandSolution(duals.stationarity_gradient);

  return duals;
}

void Solver::ComputeModelSpaceResiduals(SolveResult& result) const {
  const auto& model = original_model_;
  const auto& x = result.x;
  const int n = model.num_variables();
  if (x.size() != n) return;

  // Recompute primal feasibility from original Model + x.
  // Does NOT overwrite result.duals (slack, eq_residual, lambda, nu)
  // which are populated by ExtractDuals / penalty.
  double min_slack_model = 1e30;
  double eq_res_model = 0;

  // Accumulate Qx and stationarity gradient.
  Eigen::VectorXd Qx = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd stat = model.has_linear_cost()
      ? Eigen::VectorXd(model.linear_cost())
      : Eigen::VectorXd::Zero(n);

  int lin_idx = 0, eq_idx = 0;
  for (int i = 0; i < model.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        // Slack: A * x[vars] + b.
        Eigen::VectorXd xv(data.vars.size());
        for (int j = 0; j < (int)data.vars.size(); ++j)
          xv(j) = x(data.vars[j]);
        Eigen::VectorXd slack = Eigen::MatrixXd(data.A) * xv + data.b;
        min_slack_model = std::min(min_slack_model, slack.minCoeff());

        // Lambda from the KKT solve (already in result from ExtractDuals).
        // We keep whatever was there — the ordering matches because we
        // iterate constraints in the same order as the original Model.
        // If ExtractDuals populated lambda in reduced-model order, it
        // won't match.  For now, recompute lambda = 0 (placeholder)
        // unless we can recover it.
        // TODO: recover lambda from the KKT solve with correct mapping.
        lin_idx++;

      } else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        Eigen::VectorXd xv(data.primal_vars.size());
        for (int j = 0; j < (int)data.primal_vars.size(); ++j)
          xv(j) = x(data.primal_vars[j]);
        Eigen::VectorXd res = Eigen::MatrixXd(data.C) * xv - data.d;
        eq_res_model = std::max(eq_res_model, res.norm());
        eq_idx++;

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        Eigen::VectorXd xv(data.vars.size());
        for (int j = 0; j < (int)data.vars.size(); ++j)
          xv(j) = x(data.vars[j]);
        Eigen::VectorXd Qxv = Eigen::MatrixXd(data.Q_sparse) * xv;
        for (int j = 0; j < (int)data.vars.size(); ++j)
          Qx(data.vars[j]) += Qxv(j);
      }
    }, model.constraint(i));
  }

  // Objective in original Model space.
  result.objective = stat.dot(x) + 0.5 * Qx.dot(x);

  // Update optimality with Model-space values.
  result.optimality.min_slack = min_slack_model;
}

}  // namespace conex
