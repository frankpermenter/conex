#include "conex/cone_program.h"
#include "conex/kkt_solver.h"

#include <vector>

#include "conex/divergence.h"
#include "conex/newton_step.h"

namespace conex {

double CalcMinMu(double lambda_max, double, WeightedSlackEigenvalues* p) {
  const double kMaxNormInfD = p->limit;
  double inv_sqrt_mu = (1.0 + kMaxNormInfD) / (lambda_max + 1e-12);
  if (inv_sqrt_mu < 1e-3) {
    inv_sqrt_mu = 1e-3;
  }
  return inv_sqrt_mu;
}

template <typename T>
void SetIdentity(std::vector<T*>* c) {
  for (auto& ci : *c) {
    SetIdentity(ci);
  }
}

void GetWeightedSlackEigenvalues(ConstraintManager<Container>* constraints,
                                 const Ref& y, double c_weight,
                                 WeightedSlackEigenvalues* p) {
  p->frobenius_norm_squared = 0;
  p->trace = 0;
  p->lambda_max = -30000;
  p->lambda_min = 30000;
  int i = 0;
  for (auto& ci : constraints->eqs) {
    auto ysegment = ExtractVars(y, constraints->cliques.at(i));
    Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> z(ysegment.data(),
                                                  ysegment.size(), 1);
    WeightedSlackEigenvalues temp;
    GetWeightedSlackEigenvalues(&ci.constraint, z, c_weight, &temp);

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

void PrepareParametrizedSlack(ConstraintManager<Container>* kkt,
                              const StepOptions& newton_step_parameters,
                              const Ref& y1, const Ref& y2, StepInfo* info) {
  int i = 0;
  SlackWeights p1;
  p1.e_weight = 0;
  p1.c_weight = 0;
  SlackWeights p2;
  p2.e_weight = 0;
  p2.c_weight = 1;
  for (auto& ci : kkt->eqs) {
    // TODO(FrankPermenter): Remove creation of these maps.
    auto y1segment = ExtractVars(y1, kkt->cliques.at(i));
    auto y2segment = ExtractVars(y2, kkt->cliques.at(i));
    Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> z1(y1segment.data(),
                                                   y1segment.size(), 1);

    Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> z2(y2segment.data(),
                                                   y2segment.size(), 1);

    PrepareParametrizedSlack(&ci.constraint, p1, z1, p2, z2);
    i++;
  }

  auto params = newton_step_parameters;
  for (int j = 0; j < 5; j++) {
    int i = 0;
    StepInfo info_i;
    *info = info_i;
    bool valid = true;
    // bool primal_feasible = false;
    // bool dual_feasible = false;
    for (auto& ci : kkt->eqs) {
      DoPrimalDualLineSearch(&ci.constraint, params, &info_i);
      if (info_i.inv_sqrt_mu_primal_lower_bound >
          info->inv_sqrt_mu_primal_lower_bound) {
        info->inv_sqrt_mu_primal_lower_bound =
            info_i.inv_sqrt_mu_primal_lower_bound;
      }
      if (info_i.inv_sqrt_mu_dual_lower_bound >
          info->inv_sqrt_mu_dual_lower_bound) {
        info->inv_sqrt_mu_dual_lower_bound =
            info_i.inv_sqrt_mu_dual_lower_bound;
      }
      if (info_i.inv_sqrt_mu_primal_upper_bound <
          info->inv_sqrt_mu_primal_upper_bound) {
        info->inv_sqrt_mu_primal_upper_bound =
            info_i.inv_sqrt_mu_primal_upper_bound;
      }
      if (info_i.inv_sqrt_mu_dual_upper_bound <
          info->inv_sqrt_mu_dual_upper_bound) {
        info->inv_sqrt_mu_dual_upper_bound =
            info_i.inv_sqrt_mu_dual_upper_bound;
      }

      if (info_i.inv_sqrt_mu_dual_lower_bound <
          info_i.inv_sqrt_mu_dual_upper_bound) {
        //        dual_feasible =
        //          params.dinf_limit <= 1 &&
        //          info_i.inv_sqrt_mu_dual_upper_bound > 0;
      }

      if (info_i.inv_sqrt_mu_primal_lower_bound <
          info_i.inv_sqrt_mu_primal_upper_bound) {
        //    primal_feasible =
        //        params.dinf_limit <= 1 &&
        //        info_i.inv_sqrt_mu_primal_upper_bound > 0;
      }

      double lower_bound = info_i.inv_sqrt_mu_primal_lower_bound;
      if (lower_bound < info_i.inv_sqrt_mu_dual_lower_bound) {
        lower_bound = info_i.inv_sqrt_mu_dual_lower_bound;
      }
      double upper_bound = info_i.inv_sqrt_mu_primal_upper_bound;
      if (upper_bound > info_i.inv_sqrt_mu_dual_upper_bound) {
        upper_bound = info_i.inv_sqrt_mu_dual_upper_bound;
      }
      if (upper_bound > lower_bound) {
        params.dinf_limit = upper_bound;
        valid = true;
        break;
      } else {
        valid = false;
      }

      i++;
    }
    if (valid) {
      return;
    } else {
      params.dinf_limit += 0.1;
    }
  }
  throw std::runtime_error("Failed to find mu");
}

template <typename T>
int Rank(const std::vector<T*>& c) {
  int rank = 0;
  for (const auto& ci : c) {
    rank += Rank(*ci);
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

bool Initialize(Program& prog, const SolverConfiguration& config) {
  if (!prog.is_initialized || config.initialization_mode == 0) {
    prog.stats = std::make_unique<WorkspaceStats>(config.max_iterations);
    auto& solver = prog.solver;
    auto& kkt = prog.kkt;

    prog.sys.m_ = prog.kkt_system_manager_.SizeOfKKTSystem();
    prog.sys.residual_only_ = true;

    START_TIMER(Sparsity Analysis);
    solver = std::make_unique<Solver>(prog.kkt_system_manager_.cliques,
                                      prog.kkt_system_manager_.dual_vars);

    kkt.clear();
    for (auto& c : prog.kkt_system_manager_.eqs) {
      c.kkt_assembler.Reset();
    }

    int i = 0;
    for (auto& c : prog.kkt_system_manager_.eqs) {
      c.kkt_assembler.workspace_ = &c.constraint;
      c.kkt_assembler.SetNumberOfVariables(
          prog.kkt_system_manager_.cliques.at(i).size());
      kkt.push_back(&c.kkt_assembler);
      i++;
    }

    prog.InitializeWorkspace();
    solver->Bind(&kkt);

    if (config.initialization_mode == 0) {
      SetIdentity(&prog.constraints);
    }

    END_TIMER
  }
  return true;
}

double MinimizeNormInf(WeightedSlackEigenvalues& p) {
  double y = -1;
  if (p.lambda_min + p.lambda_max > 0) {
    y = 2.0 / (p.lambda_min + p.lambda_max);
  }
  return y;
}

double ComputeMuFromDivergence(ConstraintManager<Container>& constraints,
                               std::unique_ptr<Solver>& solver,
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

bool Solve(const DenseMatrix& bin, Program& prog,
           const SolverConfiguration& config, double* primal_variable) {
#ifdef EIGEN_USE_MKL_ALL
  std::cout << "CONEX: MKL Enabled";
#endif

  auto& constraints = prog.constraints;
  auto& solver = prog.solver;
  prog.status_.solved = 0;
  prog.status_.primal_infeasible = 0;
  prog.status_.dual_infeasible = 0;

#if CONEX_VERBOSE
  std::cout.precision(2);
  std::cout << std::scientific;
  std::cout << "Starting the Conex optimizer...\n";
#endif

  CONEX_DEMAND(prog.GetNumberOfVariables() == bin.rows(),
               "Cost vector dimension does not equal number of variables");

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
  double cw = 1;
  double by = -1;

  StepOptions newton_step_parameters;
  newton_step_parameters.affine = 0;
  IterationStats stats;
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
  double c_scaling = 1;
  double b_scaling = 1;

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
        i >= (config.max_iterations - config.final_centering_steps);
    bool update_mu = (i == 0) || !(initial_centering || final_centering) ||
                     warmstart_aborted;
    warmstart_aborted = false;

    if (final_centering) {
      if (centering_steps >= config.final_centering_steps) {
        break;
      }
    }

    START_TIMER(Assemble)
    solver->Assemble();
    AssembleSchurComplement(&prog.kkt_system_manager_, &prog.sys);
    END_TIMER

    if (i < 1) {
      b_scaling = 1.0 / (1e-9 + b.norm());
      c_scaling = 1.0 / (1e-9 + prog.sys.AQc.norm());
    }
    REPORT(b_scaling);
    REPORT(c_scaling);

    START_TIMER(Factor)
    if (!solver->Factor()) {
      solver->Assemble();
      solver->Factor();
      if (i == 0 && config.initialization_mode) {
        PRINTSTATUS("Aborting warmstart (factorization failed)...");
        SetIdentity(&prog.constraints);
        warmstart_aborted = true;
        continue;
      }
      prog.status_.solved = 0;
      PRINTSTATUS("Factorization failed.");
      return prog.status_.solved;
    }
    END_TIMER

    // Do not do line search if we have equality constraints.
    // TODO(FrankPermenter): Add support for line search with equalities.
    bool do_line_search =
        prog.kkt_system_manager_.GetNumberOfDualVariables() == 0;
    do_line_search = false;

    StepInfo info_slack;
    if (do_line_search) {
      Eigen::MatrixXd y1data(prog.kkt_system_manager_.SizeOfKKTSystem(), 1);
      Ref y1(y1data.data(), prog.kkt_system_manager_.SizeOfKKTSystem(), 1);
      y1 = -2 * prog.sys.AW;
      solver->SolveInPlace(&y1);
      Eigen::MatrixXd y2data(prog.kkt_system_manager_.SizeOfKKTSystem(), 1);
      Ref y2(y2data.data(), prog.kkt_system_manager_.SizeOfKKTSystem(), 1);
      y2 = b + prog.sys.AQc;
      solver->SolveInPlace(&y2);
      PrepareParametrizedSlack(&prog.kkt_system_manager_,
                               newton_step_parameters, y1, y2, &info_slack);
    }

    if (update_mu) {
      if (do_line_search) {
        newton_step_parameters.inv_sqrt_mu =
            info_slack.inv_sqrt_mu_primal_upper_bound;
        if (newton_step_parameters.inv_sqrt_mu >
            info_slack.inv_sqrt_mu_dual_upper_bound) {
          newton_step_parameters.inv_sqrt_mu =
              info_slack.inv_sqrt_mu_dual_upper_bound;
        }
      } else {
        double temp = ComputeMuFromDivergence(
            prog.kkt_system_manager_, solver, prog.sys.AQc * c_scaling,
            c_scaling, b * b_scaling, config, rankK, &y);
        if (temp > 0) {
          newton_step_parameters.inv_sqrt_mu = temp;
        } else {
          newton_step_parameters.inv_sqrt_mu *= .5;
        }
      }
    } else {
      if (initial_centering == 0) {
        centering_steps++;
      }
    }

    const double max = config.inv_sqrt_mu_max;
    const double min = std::sqrt(1.0 / (1e-15 + config.maximum_mu));
    ApplyLimits(&newton_step_parameters.inv_sqrt_mu, min, max);

    double mu = 1.0 / (newton_step_parameters.inv_sqrt_mu);
    mu *= mu;

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

    if (i == 0 && config.initialization_mode &&
        info.norminfd >= config.warmstart_abort_threshold) {
      PRINTSTATUS("Aborting warmstart... (Newton step too large)");
      SetIdentity(&prog.constraints);
      warmstart_aborted = true;
    } else {
      TakeStep(&prog.kkt_system_manager_, newton_step_parameters);
    }
    END_TIMER

    const double d_2 = std::sqrt(std::fabs(info.normsqrd));
    const double d_inf = std::fabs(info.norminfd);

    REPORT(mu);
    REPORT(d_2);
    REPORT(d_inf);
    by = b.col(0).dot(y.col(0)) * 1.0 /
         (newton_step_parameters.inv_sqrt_mu * c_scaling);
    cw = prog.sys.inner_product_of_c_and_w * 1.0 /
         (newton_step_parameters.inv_sqrt_mu * b_scaling);

    REPORT(by);
    REPORT(cw);

    prog.stats->num_iter = i + 1;
    prog.stats->sqrt_inv_mu[i] = newton_step_parameters.inv_sqrt_mu;
#if CONEX_VERBOSE
    std::cout << std::endl;
#endif

    if (final_centering ||
        newton_step_parameters.inv_sqrt_mu >= inv_sqrt_mu_max) {
      if (d_inf < config.final_centering_tolerance) {
        break;
      }
    }
  }

  yout = y.topRows(m);

  double mu = 1.0 / (newton_step_parameters.inv_sqrt_mu);
  mu *= mu;
  if (mu > config.infeasibility_threshold) {
    PRINTSTATUS("Infeasible Or Unbounded!!.");
    prog.status_.solved = 0;
    prog.status_.primal_infeasible =
        cw * newton_step_parameters.inv_sqrt_mu <= -.5;
    prog.status_.dual_infeasible =
        by * newton_step_parameters.inv_sqrt_mu >= .5;
  } else {
    PRINTSTATUS("Solved.");
    prog.status_.solved = 1;
  }

  if (config.prepare_dual_variables) {
    DenseMatrix y2;
    double cost_w;
    solver->Assemble();
    AssembleSchurComplement(&prog.kkt_system_manager_, &prog.sys);
    solver->Factor();
    DenseMatrix bres = newton_step_parameters.inv_sqrt_mu * b - 1 * prog.sys.AW;
    y2 = solver->Solve(bres);

    newton_step_parameters.affine = true;
    newton_step_parameters.e_weight = 0;
    newton_step_parameters.c_weight = 0;
    Ref y2map(y2.data(), y2.rows(), y2.cols());
    StepInfo info;
    PrepareStep(&prog.kkt_system_manager_, newton_step_parameters, y2map,
                &info);
  }

  if (prog.status_.solved) {
    yout /= (newton_step_parameters.inv_sqrt_mu);
    yout /= c_scaling;
  }
  return prog.status_.solved;
}

DenseMatrix GetFeasibleObjective(Program* prg) {
  auto& prog = *prg;
  Initialize(prog, SolverConfiguration());

  Eigen::VectorXd AW(prog.kkt_system_manager_.SizeOfKKTSystem());
  Eigen::VectorXd AQc(prog.kkt_system_manager_.SizeOfKKTSystem());
  double inner_product_of_c_and_w;
  prog.solver->Assemble(&AW, &AQc, &inner_product_of_c_and_w);

  return .5 * AW;
}

bool Program::Initialize(const SolverConfiguration& config) {
  return ::conex::Initialize(*this, config);
}

}  // namespace conex
