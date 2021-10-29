#include <Eigen/Dense>
namespace conex {
namespace quadratic_programs {

struct ProblemData {
  Eigen::MatrixXd W;
  Eigen::VectorXd c;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::MatrixXd B;
  Eigen::VectorXd d;
  ProblemData() = default;
  ProblemData(int m, int n) : A(m, n), c(n), b(m), W(n, n) {}
};

struct SolverOptions {
  double dinf_limit = 1.0;
  int enable_dynamic_regularization = 1.0;
  int enable_dual_correction = 0;
  int enable_rescaling = 0;
  double target_duality_gap = 1e-9;
  int maximum_iterations = 10;
  double theta_weight = 1;
  double sqrt_mu_weight = 1;
  double minimum_mu = 1;
  double theta_truncation_threshold; 
  double endgame_rescaling_threshold; 
  double endgame_rescaling_factor; 
  int verbosity; 
};

struct Variable {
  Eigen::VectorXd expv;
  Eigen::VectorXd x;
  Eigen::VectorXd lambda;
  Eigen::VectorXd slack;
  double sqrtmu = 0;
};

enum : int {
   CONEX_LOGSPACE_IPM_SOLVED = 0,
   CONEX_LOGSPACE_IPM_SOLVED_INACCURATE = 1,
   CONEX_LOGSPACE_IPM_INFEASIBLE = 2,
   CONEX_LOGSPACE_IPM_UNKNOWN = 3,
};

struct Solution {
  Variable x;
  int status;
  int iterations;
};

Solution LogspaceIPM(const ProblemData& data,
                     const SolverOptions& options = SolverOptions(),
                     const Variable& v0 = Variable());

Solution LogspaceIPM(const ProblemData& data, const SolverOptions& options,
                     const Eigen::VectorXd& v0);

}  // namespace quadratic_programs
}  // namespace conex
