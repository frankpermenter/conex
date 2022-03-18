#include "conex/debug_macros.h"
#include "utils.h"
#include <Eigen/Dense>
namespace conex {
namespace quadratic_programs {
using Eigen::MatrixXd;
using Eigen::VectorXd;
class Contraction {
 public:
  Contraction(const MatrixXd& B, const MatrixXd& W)
      : B_(B), llt_(B.transpose() * B + W) {}
  MatrixXd Eval(const MatrixXd& x) {
    return B_ * llt_.solve(B_.transpose() * x);
  }

  MatrixXd OneMinusEval(const MatrixXd& x) { return x - Eval(x); }

  MatrixXd EvalSubMatrix(const MatrixXd& x, const std::vector<int>& indices);

 private:
  Eigen::LLT<MatrixXd> llt_;
  MatrixXd B_;
};

Direction NewtonDirectionFromProjection(const ProblemData& data,
                                        const VectorXd& exp_v_in,
                                        const double sqrtmuinv_in,
                                        double scale);

}  // namespace quadratic_programs
}  // namespace conex
