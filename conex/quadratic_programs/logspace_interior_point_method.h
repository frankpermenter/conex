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
  double inv_sqrt_mu_weight = 1;
};

struct Variable {
  Eigen::VectorXd expv;
  Eigen::VectorXd x;
  Eigen::VectorXd lambda;
  Eigen::VectorXd slack;
  double sqrtmu = 0;
};

Variable LogspaceIPM(const ProblemData& data,
                     const SolverOptions& options = SolverOptions(),
                     const Variable& v0 = Variable());

Variable LogspaceIPM(const ProblemData& data, const SolverOptions& options,
                     const Eigen::VectorXd& v0);

}  // namespace quadratic_programs
}  // namespace conex
