#include "conex/cone_program.h"

#include <vector>

#include "conex/divergence.h"
#include "conex/kkt_solver_factory.h"
#include "conex/newton_step.h"
#include "conex/serialize.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

namespace {

void IncrementSubvector(Eigen::Ref<MatrixXd> destination,
                        const std::vector<int>& indices,
                        const Eigen::Ref<const MatrixXd> source) {
  int i = 0;
  for (auto& r : indices) {
    destination.row(r) += source.row(i);
    i++;
  }
}

void MakeAffineTermOfEqualityConstraints(const ConstraintManager& kkt,
                                         Eigen::Ref<MatrixXd> f) {
  int i = 0;
  for (auto& eq : kkt.equality_constraints().data) {
    IncrementSubvector(f, kkt.equality_constraints().dual_variables.at(i),
                       eq.affine_term());
    i++;
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
    GetWeightedSlackEigenvalues(ci->constraint(), ci->PrimalSubvector(y),
                                c_weight, &temp);

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

LineSearchOutput ComputeMuFromLineSearch(ConstraintManager& constraints,
                                         std::unique_ptr<KKTSolverBase>& solver,
                                         double dinf_upper_bound,
                                         const DenseMatrix& AQc,
                                         double c_weight, const DenseMatrix& b,
                                         const DenseMatrix& AW, Ref* y0) {
  *y0 = -2 * AW;
  solver->SolveInPlace(*y0);

  VectorXd y1_data(b.rows());
  Ref y1(y1_data.data(), b.rows(), 1);
  y1 = AQc + b - 2 * AW;
  solver->SolveInPlace(y1);
  LineSearchParameters params;
  params.options_0.c_weight = c_weight * 0;
  params.options_1.c_weight = c_weight * 1;
  params.dinf_upper_bound = dinf_upper_bound;
  LineSearchOutput output;

  for (auto& ci : constraints.cone_inequalities()) {
    LineSearchOutput output_i;
    Eigen::MatrixXd ysegment1 = ci->PrimalSubvector(*y0);
    Eigen::MatrixXd ysegment2 = ci->PrimalSubvector(y1);
    Ref z1(ysegment1.data(), ysegment1.rows(), 1);
    Ref z2(ysegment2.data(), ysegment2.rows(), 1);
    bool failure =
        PerformLineSearch(ci->constraint(), params, z1, z2, &output_i);
    output.dt_squared_norm += output_i.dt_squared_norm;
    output.d0_squared_norm += output_i.d0_squared_norm;
    output.d0_dot_dt += output_i.d0_dot_dt;
    if (failure) {
      output.failed = true;
      return output;
    }
    if (output_i.lower_bound > output.lower_bound) {
      output.lower_bound = output_i.lower_bound;
    }
    if (output_i.upper_bound < output.upper_bound) {
      output.upper_bound = output_i.upper_bound;
    }
  }
  if (output.lower_bound > output.upper_bound) {
    output.failed = true;
  }
  return output;
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
                               std::unique_ptr<KKTSolverBase>& solver,
                               const DenseMatrix& AQc, double c_weight,
                               const DenseMatrix& b,
                               const SolverConfiguration& config, int rankK,
                               Ref* workspace_y) {
  WeightedSlackEigenvalues mu_param;
  *workspace_y = AQc - b;
  solver->SolveInPlace(*workspace_y);
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
    solver = KKTSolverFactory::create_unique(&prog.kkt_system_manager_, config);
    END_TIMER
  }
  return true;
}

bool Program::AddLinearCost(const VectorXd& b, const std::vector<int>& vars) {
  return kkt_system_manager_.AddLinearCost(b, vars);
}
bool Program::AddLinearCost(const VectorXd& b) {
  return kkt_system_manager_.AddLinearCost(b);
}

namespace {
std::string ToString(int solver_type) {
  switch (solver_type) {
    case CONEX_KKT_SOLVER_TREE:
      return "Cholesky Tree";
    case CONEX_KKT_SOLVER_SUPERNODAL:
      return "Supernodal Cholesky";
    case CONEX_KKT_SOLVER_CG:
      return "Conjugate Gradient";
    case CONEX_KKT_SOLVER_SUPERNODAL_QR:
      return "QR Factorization";
    case CONEX_KKT_SOLVER_SPARSE_QR:
      return "Sparse QR Factorization";
  }
  throw std::runtime_error("Unknown KKT system type");
  return "";
}
}  // namespace
void PrintSummary(const Program& prog, const SolverConfiguration& config) {
  std::cout << "  Variables:" << prog.GetNumberOfVariables() << std::endl;
  std::cout << "  Equality Constraints: "
            << prog.constraint_manager().equality_constraints().data.size()
            << std::endl;
  std::cout << "  Cone Inequalities: "
            << prog.constraint_manager().cone_inequalities().size()
            << std::endl;
  std::cout << "  Quadratic Costs: "
            << prog.constraint_manager().quadratic_costs().size() << std::endl;

  std::cout << "  KKT Solver: " << ToString(config.kkt_solver) << std::endl;
}

void Program::ClearLinearCosts() { kkt_system_manager_.ClearLinearCost(); }

bool Solve(Program& prog, const SolverConfiguration& config,
           double* primal_variable) {
  CONEX_RETURN_ON_FAIL(
      prog.contains_quadratic_costs() == false ||
          (config.enable_line_search && !config.enable_rescaling),
      "Must enable line search and disable rescaling for problems "
      "with quadratic costs.");

  VectorXd bin = -prog.kkt_system_manager_.GetLinearCostVector();

  if (config.enable_logging) {
    SaveConeProgram(prog.kkt_system_manager_, config.log_file);
  }

  auto& constraints = prog.kkt_system_manager_.cone_inequalities();
  auto& solver = prog.solver;
  prog.status_.solved = 0;
  prog.status_.primal_infeasible = 0;
  prog.status_.dual_infeasible = 0;
  bool max_iter_failure = false;

#if CONEX_VERBOSE
  if (config.verbose) {
    std::cout.precision(2);
    std::cout << std::scientific;
    std::cout << "Starting the Conex optimizer...\n";

#ifdef EIGEN_USE_MKL_ALL
    std::cout << "...MKL Enabled\n";
#endif
#ifdef EIGEN_USE_BLAS
    std::cout << "...BLAS Enabled\n";
#endif
  }
#endif

  int m = bin.rows();
  // Empty program
  if (prog.NumberOfConstraints() == 0) {
    if (prog.NumberOfQuadraticCosts() > 0) {
      {
        if (config.verbose) {
          std::cout << "Finding solution analytically...\n";
        }
        Initialize(prog, config);
        Eigen::VectorXd b(prog.kkt_system_manager_.SizeOfKKTSystem());
        b.setZero();
        b.head(m) << bin;
        MakeAffineTermOfEqualityConstraints(prog.kkt_system_manager_, b);
        solver->Assemble();
        CONEX_DEMAND(solver->Factor(), "Factorization failed.");
        solver->SolveInPlace(b);
        Eigen::Map<DenseMatrix> y_least_squares(primal_variable, m, 1);
        y_least_squares = b.head(m);
        double yQy = 0;
        for (const auto& cost : prog.constraint_manager().quadratic_costs()) {
          yQy += cost.EvaluateQuadraticCost(y_least_squares);
        }
        prog.status_.solved = true;
        prog.status_.primal_objective_value =
            .5 * yQy + prog.kkt_system_manager_.GetLinearCostVector().dot(
                           y_least_squares.col(0));
      }
    } else {
      Eigen::Map<DenseMatrix> ynan(primal_variable, m, 1);
      prog.status_.solved = 0;
      ynan.array() = bin.array() * std::numeric_limits<double>::infinity();
    }
    return prog.status_.solved;
  }

  Initialize(prog, config);
#if CONEX_VERBOSE
  if (config.verbose) {
    PrintSummary(prog, config);
    std::cout << "\n";
  }
#endif

  Eigen::MatrixXd ydata(prog.kkt_system_manager_.SizeOfKKTSystem(), 1);
  Eigen::Map<DenseMatrix> yout(primal_variable, m, 1);
  Ref y(ydata.data(), prog.kkt_system_manager_.SizeOfKKTSystem(), 1);

  double inv_sqrt_mu_max = config.inv_sqrt_mu_max;
  double cx = 1;
  double by = -1;
  double kkt_error = 0;

  StepOptions newton_step_parameters;
  newton_step_parameters.affine = 0;
  double newton_step_parameters_inv_sqrt_mu = 0;
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
    if (config.verbose) {
      if (i < 10) {
        std::cout << "i:  " << i << ", ";
      } else {
        std::cout << "i: " << i << ", ";
      }
    }
#endif
    bool final_centering =
        (newton_step_parameters_inv_sqrt_mu >= inv_sqrt_mu_max) ||
        (kkt_error > config.kkt_error_tolerance) ||
        i >= (config.max_iterations - config.final_centering_steps);
    bool update_mu = (i == 0) || !(initial_centering || final_centering) ||
                     warmstart_aborted;
    warmstart_aborted = false;

    START_TIMER(Assemble)
    solver->Assemble();
    AssembleSchurComplementResiduals(prog.kkt_system_manager_, &prog.sys);
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
        LineSearchOutput output = ComputeMuFromLineSearch(
            prog.kkt_system_manager_, solver, config.dinf_upper_bound,
            prog.sys.AQc * c_scaling, c_scaling, b * b_scaling, prog.sys.AW,
            &y);
        if (output.failed) {
          temp = -output.d0_dot_dt / output.dt_squared_norm;
          // For debugging, print the predicted value of the d2 norm.
          // This should agree with the norm that is reported.
          // double norm = output.d0_squared_norm +
          //               (temp * temp) * output.dt_squared_norm +
          //               2 * temp * output.d0_dot_dt;
          // DUMP(std::sqrt(norm));
          if (temp < 0) {
            temp = newton_step_parameters_inv_sqrt_mu;
          }
        } else {
          temp = output.upper_bound;
        }
      }

      if (temp < 0) {
        CONEX_RETURN_ON_FAIL(
            !prog.contains_quadratic_costs(),
            "Solver terminating with error: line-search failed.");
        temp = ComputeMuFromDivergence(prog.kkt_system_manager_, solver,
                                       prog.sys.AQc * c_scaling, c_scaling,
                                       b * b_scaling, config, rankK, &y);
      }

      if (temp > 0) {
        newton_step_parameters_inv_sqrt_mu = temp;
      } else {
        newton_step_parameters_inv_sqrt_mu *= .5;
      }
    } else {
      if (initial_centering == 0) {
        centering_steps++;
      }
    }

    const double max = inv_sqrt_mu_max;
    const double min = std::sqrt(1.0 / (1e-15 + config.maximum_mu));
    ApplyLimits(&newton_step_parameters_inv_sqrt_mu, min, max);

    y = newton_step_parameters_inv_sqrt_mu *
            (b * b_scaling + prog.sys.AQc * c_scaling) -
        2 * prog.sys.AW;
    START_TIMER(Solve)
    solver->SolveInPlace(y);
    END_TIMER

    newton_step_parameters.e_weight = 1;
    newton_step_parameters.c_weight =
        newton_step_parameters_inv_sqrt_mu * c_scaling;

    StepInfo info;
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
    }

    const double d_2 = std::sqrt(std::fabs(info.normsqrd));
    const double d_inf = std::fabs(info.norminfd);
    by = b.col(0).dot(y.col(0)) * 1.0 /
         (newton_step_parameters_inv_sqrt_mu * c_scaling);
    // inv_sqrt_mu * <c, x> = c' Q(w^{1/2}) (e + d)
    //                      = c' Q(w^{1/2}) (e +  e + Q(w^{1/2})(Ay - k c))
    //                      = c' Q(w^{1/2}) (2e + Q(w^{1/2})(Ay - k c))
    //                      = 2 c' w + c'Q(w)(Ay - k c' Q(w) c)
    cx = 2 * prog.sys.inner_product_of_w_and_c +
         prog.sys.AQc.col(0).dot(y.col(0)) -
         newton_step_parameters_inv_sqrt_mu *
             prog.sys.inner_product_of_c_and_Qc * c_scaling;
    cx /= (newton_step_parameters_inv_sqrt_mu * b_scaling);

    double mu = 1.0 / (newton_step_parameters_inv_sqrt_mu);
    mu *= mu;

    double s_dot_x = mu * (rankK - d_2 * d_2) / (b_scaling * c_scaling);

    mu = mu / (c_scaling * b_scaling);
#if CONEX_VERBOSE
    if (config.verbose) {
      REPORT(mu);
      REPORT(d_2);
      REPORT(d_inf);
      double yQy = 0;
      if (prog.contains_quadratic_costs()) {
        double scale = newton_step_parameters_inv_sqrt_mu * c_scaling;
        scale *= scale;
        for (const auto& cost : prog.constraint_manager().quadratic_costs()) {
          yQy += cost.EvaluateQuadraticCost(y) * 1.0 / scale;
        }
      }
      double pobj = -(by - 0.5 * yQy);
      double dobj = -(cx + 0.5 * yQy);
      prog.status_.dual_objective_value = dobj;
      prog.status_.primal_objective_value = pobj;
      REPORT(pobj);
      REPORT(dobj);
      kkt_error =
          std::fabs(dobj - pobj + s_dot_x) / (1e-12 + std::fabs(s_dot_x));
      REPORT(kkt_error);
      std::cout << std::endl;
    }
#endif

    prog.stats->num_iter = i + 1;
    prog.stats->sqrt_inv_mu[i] = newton_step_parameters_inv_sqrt_mu;

    bool terminate =
        final_centering && centering_steps >= config.final_centering_steps ||
        i == config.max_iterations - 1;
    bool converged = newton_step_parameters_inv_sqrt_mu >= inv_sqrt_mu_max &&
                     d_inf <= config.final_centering_tolerance;

    if (terminate || converged) {
      max_iter_failure = !converged;
      if (config.prepare_dual_variables) {
        newton_step_parameters.affine = true;
        DenseMatrix bres(b.rows(), 1);
        Ref y2map(bres.data(), bres.rows(), bres.cols());
        StepInfo info;
        bres = newton_step_parameters_inv_sqrt_mu * b * b_scaling -
               1 * prog.sys.AW;
        newton_step_parameters.e_weight = 0;
        newton_step_parameters.c_weight = 0;
        solver->SolveInPlace(y2map);
        PrepareStep(&prog.kkt_system_manager_, newton_step_parameters, y2map,
                    &info);
        TakeStep(&constraints, newton_step_parameters);
      } else {
        TakeStep(&constraints, newton_step_parameters);
      }
      break;
    } else {
      TakeStep(&constraints, newton_step_parameters);
      continue;
    }
    throw std::runtime_error("Unreachable: termination logic has a bug.");
  }

  prog.status_.num_iterations = prog.stats->num_iter;
  yout = y.topRows(m);

  double mu = 1.0 / (newton_step_parameters_inv_sqrt_mu);
  mu *= mu;
  if (mu > config.infeasibility_threshold) {
    PRINTSTATUS("Infeasible Or Unbounded!!.");
    prog.status_.solved = 0;
    prog.status_.primal_infeasible =
        cx * newton_step_parameters_inv_sqrt_mu <= -.5;
    prog.status_.dual_infeasible =
        by * newton_step_parameters_inv_sqrt_mu >= .5;
  } else {
    prog.status_.solved = true;
  }

  if (prog.status_.solved) {
    yout /= (newton_step_parameters_inv_sqrt_mu);
    yout /= c_scaling;
  }

  if (prog.status_.solved) {
    if (max_iter_failure) {
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
  AssembleSchurComplementResiduals(prog.kkt_system_manager_, &prog.sys);
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

void Program::InitializeWorkspace() {
  CONEX_CHECK(workspace_data_);
  workspaces = kkt_system_manager_.workspace();

  workspaces.emplace_back(stats.get());
  workspaces.emplace_back(&sys);
  auto size = SizeOf(workspaces);
  if (size > workspace_data_->size()) {
    workspace_data_->resize(size);
  }
  Initialize(&workspaces, workspace_data_->data());

  is_initialized = true;
}

Eigen::VectorXd Solve(Program& prog, const SolverConfiguration& config) {
  Eigen::VectorXd y(prog.GetNumberOfVariables());
  Solve(prog, config, y.data());
  return y;
}

Eigen::MatrixXd Program::GetDualVariable(int i) {
  auto ci = kkt_system_manager_.cone_inequalities().at(i);
  Eigen::VectorXd xi(ci->constraint()->dual_variable_size());
  ci->constraint()->get_dual_variable(xi.data());
  if (!status_.primal_infeasible) {
    xi.array() /=
        (stats->sqrt_inv_mu[stats->num_iter - 1] * stats->b_scaling());
  }
  return xi;
}

void PrepareStep(ConstraintManager* kkt,
                 const StepOptions& newton_step_parameters, const Ref& y,
                 StepInfo* info) {
  StepInfo info_i;
  info_i.normsqrd = 0;
  info_i.norminfd = 0;
  info->normsqrd = 0;
  info->norminfd = -1;
  int i = 0;
  for (auto& ci : kkt->cone_inequalities()) {
    PrepareStep(ci->constraint(), newton_step_parameters,
                ci->PrimalSubvector(y), &info_i);
    if (info_i.norminfd > info->norminfd) {
      info->norminfd = info_i.norminfd;
    }
    info->normsqrd += info_i.normsqrd;
    i++;
  }
}

void AssembleSchurComplementResiduals(const ConstraintManager& kkt,
                                      SchurComplementSystem* s) {
  s->setZero();
  int i = 0;
  for (auto& ci : kkt.cone_inequalities()) {
    auto* rhs_i = ci->submatrix_data();
    s->inner_product_of_w_and_c += rhs_i->inner_product_of_w_and_c;
    s->inner_product_of_c_and_Qc += rhs_i->inner_product_of_c_and_Qc;
    s->inner_product_of_c_and_Qe += rhs_i->inner_product_of_c_and_Qe;
    s->inner_product_of_c_and_e += rhs_i->inner_product_of_c_and_e;
    int cnt = 0;

    for (const auto& k : ci->variables()) {
      s->AW(k) += rhs_i->AW(cnt);
      s->AQc(k) += rhs_i->AQc(cnt);
      s->AQe(k) += rhs_i->AQe(cnt);
      s->Ae(k) += rhs_i->Ae(cnt);
      cnt++;
    }
    i++;
  }

  MakeAffineTermOfEqualityConstraints(kkt, s->AQc);
}

void TakeStep(std::vector<SupernodalAssemblerConstraint*>* constraints,
              const StepOptions& newton_step_parameters) {
  for (auto& c : *constraints) {
    TakeStep(c->constraint(), newton_step_parameters);
  }
}

}  // namespace conex
