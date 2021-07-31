#include "conex/self_dual_embedding.h"
#include "conex/cone_program.h"
#include "conex/debug_macros.h"
namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace {
VectorXd BuildRHS(SelfDualEmbeddingSystem& s, const VectorXd& b,
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

class KKTSolver {
 public:
  KKTSolver(const MatrixXd& S) : LLT_(S) {}
  void SolveInPlace(VectorXd* d) { LLT_.solveInPlace(*d); }
  VectorXd Solve(const VectorXd& d) const {
    return LLT_.matrixL().transpose().solve(LLT_.matrixL().solve(d));
  }

  Eigen::LLT<MatrixXd> LLT_;
};

template <typename T>
SelfDualEmbeddingSolution SolveEmbeddingHelper(SelfDualEmbeddingSystem& s,
                                               T& kkt_solver, const VectorXd& b,
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
  schur_complement_system -= (S21 * kkt_solver.Solve(S12))(0, 0);

  if (schur_complement_system == 0) {
    schur_complement_system = schur_complement_system_last;
  }
  schur_complement_system_last = schur_complement_system;

  sol.sol1 = 1.0 / schur_complement_system *
             (f.tail(1) - S21 * kkt_solver.Solve(f.head(m)));

  VectorXd ref = f.head(m) - S12 * sol.sol1;
  sol.sol2 = kkt_solver.Solve(ref);
  return sol;
}

}  // namespace

SelfDualEmbeddingSolution SolveEmbedding(SelfDualEmbeddingSystem& s,
                                         const VectorXd& b, const double& wt,
                                         const double& sqrtmu) {
  KKTSolver solver(s.G);
  return SolveEmbeddingHelper(s, solver, b, wt, sqrtmu);
}

SelfDualEmbeddingSolution SolveEmbedding(SelfDualEmbeddingSystem& s,
                                         Solver& kkt_solver, const VectorXd& b,
                                         const double& wt,
                                         const double& sqrtmu) {
  return SolveEmbeddingHelper(s, kkt_solver, b, wt, sqrtmu);
}

void SolveHSD(Program& prog, const Eigen::VectorXd& bin,
              const SolverConfiguration& config, VectorXd* yout, double* tau,
              double* kappa) {
  std::cout << "\n\nStarting Conex optimizer (Self-dual-embedding mode)\n";

  if (!prog.is_initialized) {
    std::runtime_error("Program is not initialized.");
  }
  int m = bin.size();
  Eigen::VectorXd b(prog.kkt_system_manager_.SizeOfKKTSystem());
  int num_eqs = b.rows() - m;
  b.setZero();
  b.head(m) << bin / (1 + bin.norm() * 0);

  auto sys = prog.sys;

  double sqrtmu = .1;
  double eps = 1e-6;
  double wt = sqrtmu;
  for (int i = 0; i < config.max_iterations; i++) {
    prog.solver->Assemble();
    MatrixXd M = prog.solver->KKTMatrix();
    prog.solver->Factor();
    AssembleSchurComplementResiduals(&prog.kkt_system_manager_, &sys);
    auto sol = SolveEmbedding(sys, *prog.solver, b, wt, sqrtmu);
    VectorXd y = sol.sol2;
    double dt = sol.sol1(0);

    MatrixXd Error(num_eqs, 2);
    MatrixXd My = M.bottomRows(num_eqs) * y;
    VectorXd feq = sys.AQc.bottomRows(num_eqs);
    Error << My * feq.norm() / My.norm(), feq;
    VectorXd lambda;

    double c_weight = wt * (1 + dt) - sqrtmu;
    double e_weight = sqrtmu;

    StepOptions options;
    StepInfo info;
    options.affine = 0;
    options.c_weight = c_weight;
    options.e_weight = 1;
    options.w_weight = e_weight;
    Ref ym(sol.sol2.data(), y.rows(), 1);
    PrepareStep(&prog.kkt_system_manager_, options, ym, &info);

    double dinf = info.norminfd;
    if (dinf < std::abs(dt)) {
      dinf = std::abs(dt);
    }
    options.step_size = 2.0 / (dinf);
    if (options.step_size > 1) {
      options.step_size = 1;
    }

    double tau;
    double kappa;
    if (dinf < 1) {
      tau = sqrtmu * (wt * (1 + dt));
      kappa = sqrtmu * (1.0 / wt * (1 - dt));
    } else {
      tau = sqrtmu * wt;
      kappa = sqrtmu * 1.0 / wt;
    }

    double primal_obj = b.head(m).dot(y.head(m));
    double dual_obj = sys.inner_product_of_w_and_c;
    int num_eqs = b.size() - m;
    if (num_eqs > 0) {
      dual_obj += feq.dot(y.tail(num_eqs));
    }

    dual_obj *= sqrtmu / tau;
    primal_obj *= sqrtmu / tau;

    std::cout << i << " tau: " << tau << " kappa: " << kappa << "  d: " << dinf
              << "  sqrtmu: " << sqrtmu << "  b'y: " << primal_obj
              << " c'w: " << dual_obj << std::endl;
    prog.stats->sqrt_inv_mu[i] = tau / sqrtmu;
    prog.stats->num_iter = i + 1;
    double min_mu = 0.0001;
    if (dinf < 0.01) {
      if (sqrtmu < min_mu) {
        (*yout) = y.head(m) * sqrtmu / tau;
        options.affine = 1;
        TakeStep(&prog.kkt_system_manager_, options);
        return;
      }
    }
    if (sqrtmu > min_mu) {
      sqrtmu *= 0.5;
    }
    TakeStep(&prog.kkt_system_manager_, options);
    wt = wt * std::exp(options.step_size * dt);
  }
  DUMP("FAILED!");
}

}  // namespace conex
