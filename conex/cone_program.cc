#include "conex/cone_program.h"
#include "conex/kkt_solver.h"

#include <vector>

#include "conex/divergence.h"
#include "conex/newton_step.h"

#include "conex/quadratic_cost.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

namespace {

inline void PrepareStep(ConstraintManager* kkt,
                        const StepOptions& newton_step_parameters, const Ref& y,
                        StepInfo* info) {
  StepInfo info_i;
  info_i.normsqrd = 0;
  info_i.norminfd = 0;
  info->normsqrd = 0;
  info->norminfd = -1;
  int i = 0;
  for (auto& ci : kkt->cone_inequalities()) {
    PrepareStep(ci->constraint(), newton_step_parameters, ci->Subvector(y),
                &info_i);
    if (info_i.norminfd > info->norminfd) {
      info->norminfd = info_i.norminfd;
    }
    info->normsqrd += info_i.normsqrd;
    i++;
  }
}

inline void TakeStep(std::vector<SupernodalAssembler*>* constraints,
                     const StepOptions& newton_step_parameters) {
  for (auto& c : *constraints) {
    TakeStep(c->constraint(), newton_step_parameters);
  }
}

void AssembleSchurComplementResiduals(ConstraintManager* kkt,
                                      SchurComplementSystem* s) {
  s->setZero();
  int i = 0;
  for (auto& ci : kkt->clique_assemblers()) {
    auto* rhs_i = ci->submatrix_data();
    s->inner_product_of_w_and_c += rhs_i->inner_product_of_w_and_c;
    s->inner_product_of_c_and_Qc += rhs_i->inner_product_of_c_and_Qc;
    int cnt = 0;

    for (const auto& k : ci->variables()) {
      s->AW(k) += rhs_i->AW(cnt);
      s->AQc(k) += rhs_i->AQc(cnt);
      cnt++;
    }
    i++;
  }

  int offset = kkt->GetNumberOfVariables();
  for (auto& eq : kkt->equality_constraints()) {
    int num_eq = eq.affine_term().rows();
    s->AQc.middleRows(offset, num_eq) = eq.affine_term();
    offset += num_eq;
  }
}

template <typename T>
void SetIdentity(std::vector<T*>* c) {
  for (auto& ci : *c) {
    SetIdentity(ci->constraint());
  }
}

void GetWeightedSlackEigenvalues(ConstraintManager* constraints, const Ref& y,
                                 double c_weight, WeightedSlackEigenvalues* p) {
  p->frobenius_norm_squared = 0;
  p->trace = 0;
  p->lambda_max = -30000;
  p->lambda_min = 30000;
  int i = 0;
  for (auto& ci : constraints->cone_inequalities()) {
    WeightedSlackEigenvalues temp;
    GetWeightedSlackEigenvalues(ci->constraint(), ci->Subvector(y), c_weight,
                                &temp);

    if (p->lambda_max < temp.lambda_max) {
      p->lambda_max = temp.lambda_max;
    }
    if (p->lambda_min > temp.lambda_min) {
      p->lambda_min = temp.lambda_min;
    }
    p->frobenius_norm_squared += temp.frobenius_norm_squared;
    p->trace += temp.trace;

    i++;
  }
}

template <typename T>
int Rank(const std::vector<T*>& c) {
  int rank = 0;
  for (const auto& ci : c) {
    rank += Rank(*ci->constraint());
  }
  return rank;
}

template <typename T>
void ConstructSchurComplementSystem(std::vector<T*>* c, bool initialize,
                                    SchurComplementSystem* sys) {
  bool init = initialize;
  for (auto& ci : *c) {
    ConstructSchurComplementSystem(ci, init, sys);
    init = false;
  }
}

//    y = newton_step_parameters.inv_sqrt_mu *
//            (b * b_scaling + prog.sys.AQc * c_scaling) -
//        2 * prog.sys.AW;

double ComputeMuFromLineSearch(ConstraintManager& constraints,
                               std::unique_ptr<SupernodalKKTSolver>& solver,
                               double dinf_upper_bound, const DenseMatrix& AQc,
                               double c_weight, const DenseMatrix& b,
                               const DenseMatrix& AW, Ref* y0) {
  *y0 = -2 * AW;
  solver->SolveInPlace(y0);

  VectorXd y1_data(b.rows());
  Ref y1(y1_data.data(), b.rows(), 1);
  y1 = AQc + b - 2 * AW;
  solver->SolveInPlace(&y1);
  LineSearchParameters params;
  params.c0_weight = c_weight * 0;
  params.c1_weight = c_weight * 1;
  params.dinf_upper_bound = dinf_upper_bound;
  LineSearchOutput output;

  int i = 0;
  for (auto& ci : constraints.cone_inequalities()) {
    LineSearchOutput output_i;
    Eigen::MatrixXd ysegment1 = ci->Subvector(*y0);
    Eigen::MatrixXd ysegment2 = ci->Subvector(y1);
    Ref z1(ysegment1.data(), ysegment1.rows(), 1);
    Ref z2(ysegment2.data(), ysegment2.rows(), 1);
    bool failure =
        PerformLineSearch(ci->constraint(), params, z1, z2, &output_i);
    if (failure) {
      return -1;
    }
    if (output_i.lower_bound > output.lower_bound) {
      output.lower_bound = output_i.lower_bound;
    }
    if (output_i.upper_bound < output.upper_bound) {
      output.upper_bound = output_i.upper_bound;
    }
    i++;
  }
  if (output.lower_bound <= output.upper_bound) {
    return output.upper_bound;
  } else {
    return -1;
  }
}

// Finds the k that maximizes the denominator of the divergence upperbound:
//
//   max( k lambda_min, 2 - k lambda_max),
//
double MinimizeNormInf(WeightedSlackEigenvalues& p) {
  double y = -1;
  if (p.lambda_min > 0) {
    y = 2.0 / (p.lambda_min + p.lambda_max);
  }
  return y;
}
double ComputeMuFromDivergence(ConstraintManager& constraints,
                               std::unique_ptr<KKTSolver>& solver,
                               const DenseMatrix& AQc, double c_weight,
                               const DenseMatrix& b,
                               const SolverConfiguration& config, int rankK,
                               Ref* workspace_y) {
  WeightedSlackEigenvalues mu_param;
  *workspace_y = AQc - b;
  solver->SolveInPlace(workspace_y);
  GetWeightedSlackEigenvalues(&constraints, *workspace_y, c_weight, &mu_param);
  mu_param.rank = rankK;

  double divergence_bound = config.divergence_upper_bound * rankK;

  double inv_sqrt_mu = 0;
  inv_sqrt_mu = DivergenceUpperBoundInverse(divergence_bound, mu_param);

  if (inv_sqrt_mu == -1) {
    inv_sqrt_mu = MinimizeNormInf(mu_param);
  }

  if (inv_sqrt_mu < 0 && mu_param.trace > 1e-12) {
    // If inverse evaluation has failed, choose mu that satisfies norm bound.
    double kstar = mu_param.trace / mu_param.frobenius_norm_squared;
    double norm_bound = 1.5 * (mu_param.frobenius_norm_squared * kstar * kstar -
                               2 * mu_param.trace * kstar + rankK);
    if (norm_bound > rankK * .7) {
      norm_bound = rankK * .7;
    }

    double a = mu_param.frobenius_norm_squared;
    double b = -2 * mu_param.trace;
    double c = rankK - norm_bound;
    if (b * b - 4 * a * c < 0) {
      inv_sqrt_mu = mu_param.trace / mu_param.frobenius_norm_squared;
    } else {
      inv_sqrt_mu = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
    }
  }

  return inv_sqrt_mu;
}

void ApplyLimits(double* x, double lb, double ub) {
  if (*x > ub) {
    *x = ub;
  }

  if (*x < lb) {
    *x = lb;
  }
}

}  // namespace

bool Initialize(Program& prog, const SolverConfiguration& config) {
  if (!prog.is_initialized ||
      config.initialization_mode == CONEX_INITIALIZATION_MODE_COLDSTART) {
    prog.stats = std::make_unique<WorkspaceStats>(config.max_iterations);
    auto& solver = prog.solver;

    prog.sys.m_ = prog.kkt_system_manager_.SizeOfKKTSystem();
    prog.sys.residual_only_ = true;

    prog.InitializeWorkspace();
    if (config.initialization_mode == CONEX_INITIALIZATION_MODE_COLDSTART) {
      prog.stats->b_scaling() = 1;
      prog.stats->c_scaling() = 1;
      SetIdentity(&prog.kkt_system_manager_.cone_inequalities());
    }

    START_TIMER(Sparsity Analysis);
    if (prog.kkt_system_manager_.equality_constraints().size() > 0) {
      solver = std::make_unique<SupernodalKKTSolver>(
          prog.kkt_system_manager_.variables(),
          prog.kkt_system_manager_.equality_constraint_multipliers());
    } else {
      solver = std::make_unique<SupernodalKKTSolver>(
          prog.kkt_system_manager_.variables());
    }
    solver->Bind(prog.kkt_system_manager_.clique_assemblers());
    END_TIMER
    solver->SetIterativeRefinementIterations(
        config.iterative_refinement_iterations);
    solver->SetSolverMode(config.kkt_solver);
  }

  return true;
}

bool Program::AddLinearCost(const VectorXd& b, const std::vector<int>& vars) {
  CONEX_DEMAND(static_cast<int>(vars.size()) == b.rows(),
               "Cost vector dimension does not equal number of variables");
  int cnt = 0;
  for (auto i : vars) {
    linear_cost_(i) += b(cnt++);
  }
  return false;
}
bool Program::AddLinearCost(const VectorXd& b) {
  CONEX_RETURN_ON_FAIL(
      GetNumberOfVariables() == b.rows(),
      "Cost vector dimension does not equal number of variables");
  linear_cost_ += b;
  return CONEX_SUCCESS;
}

void Program::ClearLinearCosts() { linear_cost_.setZero(); }

void SolveHelper(Program& prog, KKTSolver* kkt_solver,
                 const SolverConfiguration& config, double* primal_variable) {}

bool Solve(Program& prog, const SolverConfiguration& config,
           double* primal_variable) {
  CONEX_RETURN_ON_FAIL(
      prog.contains_quadratic_costs_ == false ||
          (config.enable_line_search && !config.enable_rescaling),
      "Must enable line search and disable rescaling for problems "
      "with quadratic costs.");

  VectorXd bin = -prog.linear_cost_;

  auto& constraints = prog.kkt_system_manager_.cone_inequalities();
  auto& solver = prog.solver;
  prog.status_.solved = 0;
  prog.status_.primal_infeasible = 0;
  prog.status_.dual_infeasible = 0;
  bool max_iters_reached = true;

#if CONEX_VERBOSE
  std::cout.precision(2);
  std::cout << std::scientific;
  std::cout << "Starting the Conex optimizer...\n";
#endif

#ifdef EIGEN_USE_MKL_ALL
  std::cout << "...MKL Enabled\n";
#endif
#ifdef EIGEN_USE_BLAS
  std::cout << "...BLAS Enabled\n";
#endif

  int m = bin.rows();
  // Empty program
  if (prog.NumberOfConstraints() == 0) {
    Eigen::Map<DenseMatrix> ynan(primal_variable, m, 1);
    prog.status_.solved = 0;
    ynan.array() = bin.array() * std::numeric_limits<double>::infinity();
    return prog.status_.solved;
  }

  Initialize(prog, config);
  std::cout << "\n";

  Eigen::MatrixXd ydata(prog.kkt_system_manager_.SizeOfKKTSystem(), 1);
  Eigen::Map<DenseMatrix> yout(primal_variable, m, 1);
  Ref y(ydata.data(), prog.kkt_system_manager_.SizeOfKKTSystem(), 1);

  double inv_sqrt_mu_max = config.inv_sqrt_mu_max;
  double cx = 1;
  double by = -1;
  double kkt_error = 0;

  StepOptions newton_step_parameters;
  newton_step_parameters.affine = 0;
  newton_step_parameters.inv_sqrt_mu = 0;
  newton_step_parameters.affine = false;

  int rankK = Rank(constraints);
  int centering_steps = 0;
  bool warmstart_aborted = false;
  Eigen::VectorXd b(prog.kkt_system_manager_.SizeOfKKTSystem());
  b.setZero();
  b.head(m) << bin;

  int initial_centering_steps = config.initial_centering_steps_coldstart;
  int initial_centering = 1;
  auto& c_scaling = prog.stats->c_scaling();
  auto& b_scaling = prog.stats->b_scaling();

  if (config.initialization_mode) {
    PRINTSTATUS("Warmstarting...");
    initial_centering_steps = config.initial_centering_steps_warmstart;
  }

  for (int i = 0; i < config.max_iterations; i++) {
    if (i >= initial_centering_steps) {
      initial_centering = 0;
    }

#if CONEX_VERBOSE
    if (i < 10) {
      std::cout << "i:  " << i << ", ";
    } else {
      std::cout << "i: " << i << ", ";
    }
#endif
    bool final_centering =
        (newton_step_parameters.inv_sqrt_mu >= inv_sqrt_mu_max) ||
        (kkt_error > config.kkt_error_tolerance) ||
        i >= (config.max_iterations - config.final_centering_steps);
    bool update_mu = (i == 0) || !(initial_centering || final_centering) ||
                     warmstart_aborted;
    warmstart_aborted = false;

    if (final_centering) {
      if (centering_steps >= config.final_centering_steps) {
        max_iters_reached = (i >= config.max_iterations - 1);
        break;
      }
    }

    START_TIMER(Assemble)
    solver->Assemble();
    AssembleSchurComplementResiduals(&prog.kkt_system_manager_, &prog.sys);
    END_TIMER

    if (i < 1 && config.enable_rescaling) {
      if (config.initialization_mode == CONEX_INITIALIZATION_MODE_COLDSTART) {
        b_scaling = 1.0 / (1 + b.norm());
        c_scaling = 1.0 / (1 + prog.sys.AQc.norm());
      }
      // The solver returns xhat = b_scaling * x and
      //                    shat = c_scaling * s
      // satisfying xhat * shat = mu * I
      //
      // This means x * s = mu/(b_scaling * c_scaling).
      // So, we rescale the target_mu by (b_scaling * c_scaling).
      double mu_target = 1.0 / (inv_sqrt_mu_max * inv_sqrt_mu_max);
      mu_target *= (b_scaling * c_scaling);
      inv_sqrt_mu_max = 1.0 / std::sqrt(mu_target);
    }

    START_TIMER(Factor)
    if (!solver->Factor()) {
      if (i == 0 &&
          config.initialization_mode == CONEX_INITIALIZATION_MODE_WARMSTART) {
        PRINTSTATUS("Aborting warmstart...");
        SetIdentity(&constraints);
        warmstart_aborted = true;
        continue;
      }
      prog.status_.solved = 0;
      PRINTSTATUS("Factorization failed.");
      return prog.status_.solved;
    }
    END_TIMER

    if (update_mu) {
      double temp = -1;
      if (config.enable_line_search) {
        temp = ComputeMuFromLineSearch(prog.kkt_system_manager_, solver,
                                       config.dinf_upper_bound,
                                       prog.sys.AQc * c_scaling, c_scaling,
                                       b * b_scaling, prog.sys.AW, &y);
        if (temp < 0) {
          temp = newton_step_parameters.inv_sqrt_mu;
        }
      }

      if (temp < 0) {
        CONEX_RETURN_ON_FAIL(
            !prog.contains_quadratic_costs_,
            "Solver terminating with error: line-search failed.");
        temp = ComputeMuFromDivergence(prog.kkt_system_manager_, solver,
                                       prog.sys.AQc * c_scaling, c_scaling,
                                       b * b_scaling, config, rankK, &y);
      }

      if (temp > 0) {
        newton_step_parameters.inv_sqrt_mu = temp;
      } else {
        newton_step_parameters.inv_sqrt_mu *= .5;
      }
    } else {
      if (initial_centering == 0) {
        centering_steps++;
      }
    }

    const double max = inv_sqrt_mu_max;
    const double min = std::sqrt(1.0 / (1e-15 + config.maximum_mu));
    ApplyLimits(&newton_step_parameters.inv_sqrt_mu, min, max);

    y = newton_step_parameters.inv_sqrt_mu *
            (b * b_scaling + prog.sys.AQc * c_scaling) -
        2 * prog.sys.AW;
    START_TIMER(Solve)
    solver->SolveInPlace(&y);
    END_TIMER

    newton_step_parameters.e_weight = 1;
    newton_step_parameters.c_weight =
        newton_step_parameters.inv_sqrt_mu * c_scaling;

    StepInfo info;
    START_TIMER(Update)
    PrepareStep(&prog.kkt_system_manager_, newton_step_parameters, y, &info);
    newton_step_parameters.step_size = 2.0 / (info.norminfd * info.norminfd);
    if (newton_step_parameters.step_size > 1) {
      newton_step_parameters.step_size = 1;
    }

    if (i == 0 &&
        (config.initialization_mode == CONEX_INITIALIZATION_MODE_WARMSTART) &&
        info.norminfd >= config.warmstart_abort_threshold) {
      PRINTSTATUS("Aborting warmstart...");
      SetIdentity(&constraints);
      warmstart_aborted = true;
    } else {
      TakeStep(&constraints, newton_step_parameters);
    }
    END_TIMER

    const double d_2 = std::sqrt(std::fabs(info.normsqrd));
    const double d_inf = std::fabs(info.norminfd);
    by = b.col(0).dot(y.col(0)) * 1.0 /
         (newton_step_parameters.inv_sqrt_mu * c_scaling);
    // inv_sqrt_mu * <c, x> = c' Q(w^{1/2}) (e + d)
    //                      = c' Q(w^{1/2}) (e +  e + Q(w^{1/2})(Ay - k c))
    //                      = c' Q(w^{1/2}) (2e + Q(w^{1/2})(Ay - k c))
    //                      = 2 c' w + c'Q(w)(Ay - k c' Q(w) c)
    cx = 2 * prog.sys.inner_product_of_w_and_c +
         prog.sys.AQc.col(0).dot(y.col(0)) -
         newton_step_parameters.inv_sqrt_mu *
             prog.sys.inner_product_of_c_and_Qc * c_scaling;

    cx /= (newton_step_parameters.inv_sqrt_mu * b_scaling);

    double mu = 1.0 / (newton_step_parameters.inv_sqrt_mu);
    mu *= mu;

    double s_dot_x = mu * (rankK - d_2 * d_2) / (b_scaling * c_scaling);

    mu = mu / (c_scaling * b_scaling);
    REPORT(mu);
    REPORT(d_2);
    REPORT(d_inf);
    if (!prog.contains_quadratic_costs_) {
      REPORT(by);
      REPORT(cx);
      kkt_error = std::fabs(cx - by - s_dot_x) / s_dot_x;
      REPORT(kkt_error);
    }

    prog.stats->num_iter = i + 1;
    prog.stats->sqrt_inv_mu[i] = newton_step_parameters.inv_sqrt_mu;
#if CONEX_VERBOSE
    std::cout << std::endl;
#endif

    if (final_centering ||
        newton_step_parameters.inv_sqrt_mu >= inv_sqrt_mu_max) {
      if (d_inf <= config.final_centering_tolerance) {
        max_iters_reached = false;
        break;
      }
    }
  }

  prog.status_.num_iterations = prog.stats->num_iter;
  yout = y.topRows(m);

  double mu = 1.0 / (newton_step_parameters.inv_sqrt_mu);
  mu *= mu;
  if (mu > config.infeasibility_threshold) {
    PRINTSTATUS("Infeasible Or Unbounded!!.");
    prog.status_.solved = 0;
    prog.status_.primal_infeasible =
        cx * newton_step_parameters.inv_sqrt_mu <= -.5;
    prog.status_.dual_infeasible =
        by * newton_step_parameters.inv_sqrt_mu >= .5;
  } else {
    prog.status_.solved = true;
  }
  if (config.prepare_dual_variables) {
    solver->Assemble();
    AssembleSchurComplementResiduals(&prog.kkt_system_manager_, &prog.sys);
    solver->Factor();
    DenseMatrix bres =
        newton_step_parameters.inv_sqrt_mu * b * b_scaling - 1 * prog.sys.AW;
    Ref y2map(bres.data(), bres.rows(), bres.cols());
    solver->SolveInPlace(&y2map);
    newton_step_parameters.affine = true;
    newton_step_parameters.e_weight = 0;
    newton_step_parameters.c_weight = 0;
    StepInfo info;
    PrepareStep(&prog.kkt_system_manager_, newton_step_parameters, y2map,
                &info);
  }

  if (prog.status_.solved) {
    yout /= (newton_step_parameters.inv_sqrt_mu);
    yout /= c_scaling;
  }

  if (prog.status_.solved) {
    if (max_iters_reached) {
      prog.status_.solved = false;
      PRINTSTATUS("Terminating at maximum iteration limit.");
    } else {
      PRINTSTATUS("Solved.");
    }
  }

  return prog.status_.solved;
}

DenseMatrix GetFeasibleObjective(Program* prg) {
  auto& prog = *prg;
  Initialize(prog, SolverConfiguration());
  prog.solver->Assemble();
  AssembleSchurComplementResiduals(&prog.kkt_system_manager_, &prog.sys);
  return .5 * prog.sys.AW;
}

bool Solve(const DenseMatrix& b, Program& prog,
           const SolverConfiguration& config, double* primal_variable) {
  prog.ClearLinearCosts();
  prog.AddLinearCost(-b);
  return Solve(prog, config, primal_variable);
}

CONEX_ID Program::AddQuadraticCost(const DenseMatrix& Q,
                                   const std::vector<int>& vars) {
  contains_quadratic_costs_ = true;
  return kkt_system_manager_.AddQuadraticCost(Q, vars);
}

int Program::NumberOfQuadraticCosts() const {
  return kkt_system_manager_.quadratic_costs().size();
}

int Program::UpdateQuadraticCost(int cost_id, double value, int row, int col) {
  int cnt = 0;
  for (auto c : kkt_system_manager_.quadratic_costs()) {
    if (cnt == cost_id) {
      return c.UpdateMatrix(value, row, col);
    }
    cnt++;
  }
  CONEX_RETURN_ON_FAIL(false, "Invalid Quadratic Cost ID.");
}

CONEX_ID Program::AddQuadraticCost(const Eigen::MatrixXd& Q) {
  CONEX_RETURN_ON_FAIL(Q.rows() == GetNumberOfVariables(),
                       "Order of matrix must equal number of variables.");
  std::vector<int> variables(GetNumberOfVariables());
  for (int i = 0; i < GetNumberOfVariables(); i++) {
    variables[i] = i;
  }
  return AddQuadraticCost(Q, variables);
}

int Program::UpdateLinearOperatorOfConstraint(int i, double value, int variable,
                                              int row, int col,
                                              int hyper_complex_dim) {
  CONEX_RETURN_ON_FAIL(
      i < static_cast<int>(kkt_system_manager_.cone_inequalities().size()),
      "Invalid Constraint");
  return UpdateLinearOperator(
      kkt_system_manager_.cone_inequalities().at(i)->constraint(), value,
      variable, row, col, hyper_complex_dim);
}

int Program::UpdateAffineTermOfConstraint(int i, double value, int row, int col,
                                          int hyper_complex_dim) {
  CONEX_RETURN_ON_FAIL(
      i < static_cast<int>(kkt_system_manager_.cone_inequalities().size()),
      "Invalid Constraint");
  return UpdateAffineTerm(
      kkt_system_manager_.cone_inequalities().at(i)->constraint(), value, row,
      col, hyper_complex_dim);
}

}  // namespace conex
