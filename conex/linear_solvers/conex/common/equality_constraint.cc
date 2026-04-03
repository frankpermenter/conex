#include "conex/common/equality_constraint.h"

namespace conex {

SupernodalAssemblerEqualities::SupernodalAssemblerEqualities(
    const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
    const std::vector<int>& primal_variables,
    const std::vector<int>& dual_variables)
    : SupernodalAssemblerBase(primal_variables, dual_variables), A_(A), b_(b) {}

}  // namespace conex
