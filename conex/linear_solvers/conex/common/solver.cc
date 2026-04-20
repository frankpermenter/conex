#include "conex/common/solver.h"

#include <numeric>

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
  s.reduced_linear_cost_ = reduced.linear_cost();
  s.reduced_model_ = std::move(reduced);
  s.system_ = KKTSystem::Build(s.reduced_model_, config);
  return s;
}

Solver Solver::Build(const Model& model,
                     const TreeSpec& tree,
                     const SolverConfiguration& config) {
  // TreeSpec references original constraint IDs — skip preprocessing.
  Solver s;
  s.expansion_.original_n = model.num_variables();
  s.expansion_.col_map.resize(model.num_variables());
  std::iota(s.expansion_.col_map.begin(), s.expansion_.col_map.end(), 0);
  s.reduced_linear_cost_ = model.linear_cost();
  s.reduced_model_ = model;
  s.system_ = KKTSystem::Build(s.reduced_model_, tree, config);
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

ConstraintDuals Solver::ExtractDuals(const Eigen::VectorXd& x_reduced,
                                     const RowSpace& lambda) {
  ConstraintDuals duals;

  // Compute slacks via KKT: s = Ax + b (uses symmetrized A for PSD).
  auto* k = kkt();
  auto x_rhs = k->MakeSolverRHS();
  x_rhs = k->MakeBlockVariable(x_reduced);
  RowSpace slack_rs = k->MakeRowSpace();
  k->MultiplyA(x_rhs, slack_rs);
  slack_rs += k->GetAffineTerm();
  const auto& slack_vec = slack_rs.col();

  // The RowSpace segments follow registration order in
  // KKTSystem::RegisterAssemblersWithTreeSolver:
  //   1. Linear constraints
  //   2. PSD constraints
  //   3. SOC constraints
  // We compute per-constraint offsets, then populate duals in Model order.
  const auto& lam_vec = lambda.col();
  const int nc = reduced_model_.num_constraints();

  // Compute RowSpace offset for each constraint, following registration order.
  std::vector<int> rs_offset(nc, -1);
  int offset = 0;
  // Pass 1: Linear.
  for (int i = 0; i < nc; ++i) {
    if (auto* lc = std::get_if<Model::LinearConstraintData>(
            &reduced_model_.constraint(i))) {
      rs_offset[i] = offset;
      offset += lc->A.rows();
    }
  }
  // Pass 2: PSD.
  for (int i = 0; i < nc; ++i) {
    if (auto* pc = std::get_if<Model::PSDConstraintData>(
            &reduced_model_.constraint(i))) {
      rs_offset[i] = offset;
      offset += pc->B.rows() * pc->B.rows();
    }
  }
  // Pass 3: SOC.
  for (int i = 0; i < nc; ++i) {
    if (auto* sc = std::get_if<Model::SOCConstraintData>(
            &reduced_model_.constraint(i))) {
      rs_offset[i] = offset;
      offset += sc->A.rows();
    }
  }

  // Now iterate in Model order and extract using computed offsets.
  for (int i = 0; i < nc; ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      int o = rs_offset[i];

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        int m = data.A.rows();
        duals.slack.push_back(slack_vec.segment(o, m));
        duals.lambda.push_back(lam_vec.segment(o, m));

      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        int m = data.A.rows();
        duals.slack.push_back(slack_vec.segment(o, m));
        // SOC quadratic representation P(W) = 2W² introduces a factor of 2.
        duals.lambda.push_back(2.0 * lam_vec.segment(o, m));

      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        int n = data.B.rows();
        int seg = n * n;
        Eigen::Map<const Eigen::MatrixXd> lam_map(
            lam_vec.data() + o, n, n);
        duals.psd_lambda.push_back(Eigen::MatrixXd(lam_map));
        Eigen::Map<const Eigen::MatrixXd> slack_map(
            slack_vec.data() + o, n, n);
        duals.psd_slack.push_back(Eigen::MatrixXd(slack_map));

      } else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        // The KKT system stores equality duals with opposite sign
        // from the standard convention (c = A'λ + C'ν), so we negate.
        const auto& dual_vars = system_.dual_variables(i);
        Eigen::VectorXd nu(dual_vars.size());
        for (int j = 0; j < (int)dual_vars.size(); ++j) {
          int dv = dual_vars[j];
          nu(j) = (dv < x_reduced.size()) ? -x_reduced(dv) : 0;
        }
        duals.nu.push_back(nu);

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        // Quadratic costs don't have duals.
      }
    }, reduced_model_.constraint(i));
  }

  return duals;
}

}  // namespace conex
