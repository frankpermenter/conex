#include <Eigen/Dense>

namespace conex {
namespace quadratic_programs {
struct Limits {
  double inv_sqrt_mu_lb = 1e-9;
  double inv_sqrt_mu_ub = 1e9;
  double theta_times_inv_sqrt_mu_lb = 0;
  double theta_times_inv_sqrt_mu_ub = 1;
  double inv_sqrt_mu_weight = -0.01;
  double theta_weight = 1;
};

Eigen::MatrixXd InfinityNorm(const Eigen::MatrixXd& d0,
                             const Eigen::MatrixXd& d1,
                             const Eigen::MatrixXd& d2, double bound,
                             const Limits& limit = Limits());


struct MuThetaValues {
  double sqrtmu;
  double theta;
  bool success;
};
MuThetaValues MuThetaSelect(const Eigen::MatrixXd& d0,
                             const Eigen::MatrixXd& d1,
                             const Eigen::MatrixXd& d2, double bound,
                             const Limits& limit = Limits());


}  // namespace quadratic_programs
}  // namespace conex
