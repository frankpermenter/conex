#include "conex/self_dual_embedding.h"

#include "conex/cone_program.h"
#include "conex/debug_macros.h"
namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace {

template <typename T>
int Rank(const std::vector<T*>& c) {
  int rank = 0;
  for (const auto& ci : c) {
    rank += ci->constraint()->Rank();
  }
  return rank;
}

VectorXd BuildRHS(const SelfDualEmbeddingSystem& s, const VectorXd& b,
                  const double& wt, const double& sqrtmu) {
  int m = b.rows();
  VectorXd f(m + 1);
  f.setZero();
  f.head(m) = wt * (b + s.AQc) + sqrtmu * (s.AQe - s.AQc + s.Ae - b) - 2 * s.AW;
  f(m) = 1.0 / wt + 2 * s.inner_product_of_w_and_c -
         wt * (s.inner_product_of_c_and_Qc) -
         sqrtmu * (s.inner_product_of_c_and_Qe - s.inner_product_of_c_and_Qc) -
         sqrtmu * (s.inner_product_of_c_and_e + 1);

  return f;
}

SelfDualEmbeddingSolution SolveEmbeddingHelper(const SelfDualEmbeddingSystem& s,
                                               KKTSolverBase* kkt_solver,
                                               const VectorXd& b,
                                               const double& wt,
                                               const double& sqrtmu) {
  static double schur_complement_system_last = -1;
  SelfDualEmbeddingSolution sol;
  int m = b.rows();
  auto f = BuildRHS(s, b, wt, sqrtmu);

  // const MatrixXd& S11 = s.G;
  const MatrixXd& S21 = b.transpose() - s.AQc.transpose();
  const MatrixXd& S12 = -wt * (s.AQc + b);
  MatrixXd S22(1, 1);
  S22(0, 0) = wt * s.inner_product_of_c_and_Qc + 1.0 / wt;

  double schur_complement_system = S22(0, 0);
  schur_complement_system -= (S21 * kkt_solver->Solve(S12))(0, 0);

  if (schur_complement_system == 0) {
    schur_complement_system = schur_complement_system_last;
  }
  schur_complement_system_last = schur_complement_system;

  sol.sol1 = 1.0 / schur_complement_system *
             (f.tail(1) - S21 * kkt_solver->Solve(f.head(m)));

  VectorXd ref = f.head(m) - S12 * sol.sol1;
  sol.sol2 = kkt_solver->Solve(ref);
  return sol;
}

}  // namespace

// SelfDualEmbeddingSolution SolveEmbedding(const SelfDualEmbeddingSystem& s,
//                                         const VectorXd& b, const double& wt,
//                                         const double& sqrtmu) {
//  KKTSolver solver(s.G);
//  return SolveEmbeddingHelper(s, &solver, b, wt, sqrtmu);
//}

SelfDualEmbeddingSolution SolveEmbedding(const SelfDualEmbeddingSystem& s,
                                         KKTSolverBase* kkt_solver,
                                         const VectorXd& b, const double& wt,
                                         const double& sqrtmu) {
  return SolveEmbeddingHelper(s, kkt_solver, b, wt, sqrtmu);
}

struct NewtonDirectionInfo {
  StepOptions options;
  StepInfo info;
  double dinf;
  double dinf_t;
  double dinf_w;
  Eigen::VectorXd y;
  double d_tau;
};

NewtonDirectionInfo SetNewtonDirection(ConstraintManager* constraints,
                                       const SchurComplementSystem& sys,
                                       KKTSolverBase* solver, const VectorXd& b,
                                       double wt, double sqrtmu,
                                       NewtonDirectionInfo* dir_ptr) {
  auto& dir = *dir_ptr;
  auto sol = SolveEmbedding(sys, solver, b, wt, sqrtmu);
  dir.d_tau = sol.sol1(0);
  dir.y = sol.sol2;

  double c_weight = wt * (1 + dir.d_tau) - sqrtmu;
  double e_weight = sqrtmu;

  StepInfo info;
  dir.options.step_type = CONEX_STEP_TYPE_GEODESIC;
  dir.options.c_weight = c_weight;
  dir.options.e_weight = 1;
  dir.options.w_weight = e_weight;
  Ref ym(sol.sol2.data(), dir.y.rows(), 1);
  PrepareStep(constraints, dir.options, ym, &info);
  dir.dinf = info.norminfd;
  dir.info = info;
  dir.dinf_w = dir.dinf;
  dir.dinf_t = std::abs(dir.d_tau);

  if (dir.dinf < std::abs(dir.d_tau)) {
    dir.dinf = std::abs(dir.d_tau);
  }
  return dir;
}

#if 0
double DoLineSearch(ConstraintManager* constraints,
                    const SchurComplementSystem& sys, KKTSolverBase* solver,
                    const VectorXd& b, double wt, double sqrtmu,
                    NewtonDirectionInfo* dir_ptr) {
  dir_ptr->dinf = 1e30;
  double scale = 1;
  double sqrtmu_previous = sqrtmu;
  int j = 0;
  int max_iter = 10;
  while (dir_ptr->dinf >= 1 && j < max_iter) {
    scale = 1 - std::pow(.8, j + 1);
    if (j == max_iter - 1) {
      scale = .99;
    }
    sqrtmu = sqrtmu_previous * scale;
    SetNewtonDirection(constraints, sys, solver, b, wt, sqrtmu, dir_ptr);
    j++;
  }
  return sqrtmu;
}
#else

double DoLineSearch(double dinf_upper_bound, ConstraintManager* constraints,
                    const SchurComplementSystem& sys, KKTSolverBase* solver,
                    const VectorXd& b, double wt, double sqrtmu,
                    NewtonDirectionInfo* dir_ptr) {
  NewtonDirectionInfo dir0;
  NewtonDirectionInfo dir1;
  SetNewtonDirection(constraints, sys, solver, b, wt, 0, &dir0);
  SetNewtonDirection(constraints, sys, solver, b, wt, 1, &dir1);

  LineSearchOutput output;

  LineSearchParameters params;
  params.dinf_upper_bound = dinf_upper_bound;
  params.options_0 = dir0.options;
  params.options_1 = dir1.options;

  for (auto& ci : constraints->cone_inequalities()) {
    LineSearchOutput output_i;
    Eigen::MatrixXd y1 = ci->PrimalSubvector(dir0.y);
    Eigen::MatrixXd y2 = ci->PrimalSubvector(dir1.y);
    bool failure =
        ci->constraint()->PerformLineSearch(params, y1, y2, &output_i);
    if (failure) {
      throw std::runtime_error("Line search failed.");
      output.failed = true;
      break;
    }
    if (output_i.lower_bound > output.lower_bound) {
      output.lower_bound = output_i.lower_bound;
    }
    if (output_i.upper_bound < output.upper_bound) {
      output.upper_bound = output_i.upper_bound;
    }
  }

  double dinfmax = params.dinf_upper_bound;
  double delta = (dir1.d_tau - dir0.d_tau);
  double upper_bound_i = (dinfmax - dir0.d_tau) / delta;
  double lower_bound_i = (-dinfmax - dir0.d_tau) / delta;
  if (lower_bound_i > upper_bound_i) {
    double temp = upper_bound_i;
    upper_bound_i = lower_bound_i;
    lower_bound_i = temp;
  }
  if (lower_bound_i > output.lower_bound) {
    output.lower_bound = lower_bound_i;
  }
  if (upper_bound_i < output.upper_bound) {
    output.upper_bound = upper_bound_i;
  }

  return output.lower_bound;
}
#endif

// Solves embedding equations for decreasing
// sequence of mu:
//
//    x's + k * tau = mu * (rank K + 1)
//    b'y - c'x = kappa
//    A'x = tau * b + mu * (Ae - b)
//    tau c - Ay = s + mu * (c-e)
//

void SolveHSD(Program& prog, const Eigen::VectorXd&,
              const SolverConfiguration& config, VectorXd* yout, double* tau,
              double* kappa) {
  Initialize(prog, config);
  int m = prog.constraint_manager().SizeOfKKTSystem();
  int num_vars = prog.constraint_manager().GetNumberOfVariables();
  VectorXd ydata(m);
  WorkspaceHSDEmbedding workspace(ydata.data(), m);
  SolveHSD(prog.constraint_manager(), config, prog.kkt_system_residual(),
           prog.kkt_solver(), prog.Status(), workspace, &prog.statistics());
  *yout = ydata.head(num_vars);
}

void SolveHSD(ConstraintManager& kkt_system_manager_,
              const SolverConfiguration& config, SchurComplementSystem& sys,
              KKTSolverBase* solver, ConexStatus& status_,
              WorkspaceHSDEmbedding& workspace, WorkspaceStats* stats) {
  std::cout << "\n\nStarting Conex optimizer (Self-dual-embedding mode)\n";
  auto* yout = &workspace.y;

  int m = kkt_system_manager_.GetNumberOfVariables();
  Eigen::VectorXd b(kkt_system_manager_.SizeOfKKTSystem());
  b.setZero();
  b.head(m) << -kkt_system_manager_.GetLinearCostVector();

  double sqrtmu = 1;
  double wt = sqrtmu;
  NewtonDirectionInfo dir;
  double& dinf = dir.dinf;
  double& dt = dir.d_tau;
  VectorXd& y = dir.y;
  int rank = Rank(kkt_system_manager_.cone_inequalities());
  for (int i = 0; i < config.max_iterations; i++) {
    solver->Assemble();
    CONEX_CHECK(solver->Factor());

    AssembleSchurComplementResiduals(kkt_system_manager_, &sys);

    sqrtmu = DoLineSearch(config.dinf_upper_bound, &kkt_system_manager_, sys,
                          solver, b, wt, sqrtmu, &dir);
    SetNewtonDirection(&kkt_system_manager_, sys, solver, b, wt, sqrtmu, &dir);

    dir.options.step_size = 2.0 / (dinf * dinf);
    if (dir.options.step_size > 1) {
      dir.options.step_size = 1;
    }

    double tau;
    double kappa;
    tau = sqrtmu * (wt * (1 + dt));
    kappa = sqrtmu * (1.0 / wt * (1 - dt));

    double primal_obj = b.head(m).dot(y.head(m));
    double dual_obj = sys.inner_product_of_w_and_c;

    // clang-format off
    // inv_sqrt_mu * <c, x> = c' Q(w^{1/2}) (e + d)
    //                      = c' Q(w^{1/2}) (e +  e + Q(w^{1/2})(Ay - k c - k_1 e)) 
    //                      = c' Q(w^{1/2}) (2e + Q(w^{1/2})(Ay - k c - k1e)) 
    //                      = 2 c' w + c'Q(w)(Ay - k c' Q(w) c - k1 c Q(w) e)
    // clang-format on
    dual_obj = 2 * sys.inner_product_of_w_and_c + sys.AQc.col(0).dot(y.col(0)) -
               dir.options.c_weight * sys.inner_product_of_c_and_Qc -
               dir.options.w_weight * sys.inner_product_of_c_and_Qe;

    dual_obj *= sqrtmu / tau;
    primal_obj *= sqrtmu / tau;

    double gap_error =
        (primal_obj - dual_obj) * tau -
        (kappa - sqrtmu * sqrtmu * (1.0 + sys.inner_product_of_c_and_e));

    std::cout << stats->num_iter << " mu: " << 1.0 / std::pow(wt * (1 + dt), 2)
              << " kappa: " << kappa << "  dinf: " << dinf
              << "  dinf_w: " << dir.dinf_w << "  d_t: " << dt
              << "  dsqr: " << dir.info.normsqrd << "  rank: " << rank
              << "  theta: " << sqrtmu * sqrtmu << "  b'y: " << primal_obj
              << " c'x: " << dual_obj << " gap_error " << gap_error
              << std::endl;
    stats->sqrt_inv_mu[stats->num_iter] = wt * (1 + dt);
    stats->num_iter++;
    double min_mu = 1.0 / config.inv_sqrt_mu_max;
    min_mu *= min_mu;
    if (dinf <= config.final_centering_tolerance) {
      if (sqrtmu * sqrtmu <= min_mu ||
          stats->sqrt_inv_mu[stats->num_iter - 1] > config.inv_sqrt_mu_max) {
        (*yout) = y * (wt * (1 + dt));
        status_.solved = (wt * (1 + dt)) > 1e-2;
        if (!status_.solved) {
          dir.options.step_type = CONEX_STEP_TYPE_DUAL_BARRIER;
          TakeStep(&kkt_system_manager_.cone_inequalities(), dir.options);
        }
        return;
      }
    }

    TakeStep(&kkt_system_manager_.cone_inequalities(), dir.options);
    wt = wt * std::exp(dir.options.step_size * dt);
  }
}

Eigen::VectorXd SolveHSD(Program& prog, const SolverConfiguration& config) {
  Eigen::VectorXd y(prog.GetNumberOfVariables());
  VectorXd b = prog.constraint_manager().GetLinearCostVector();
  double tau;
  double kappa;
  SolveHSD(prog, -b, config, &y, &tau, &kappa);
  return y;
}
}  // namespace conex
