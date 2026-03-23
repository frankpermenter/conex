#include "conex/equality_constraint.h"

namespace conex {

using T = EqualityConstraints;
using Eigen::MatrixXd;
using std::vector;

T::EqualityConstraints(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b)
    : A_(A), b_(b) {}

SupernodalAssemblerEqualities::SupernodalAssemblerEqualities(
    const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
    const std::vector<int>& primal_variables,
    const std::vector<int>& dual_variables)
    : SupernodalAssemblerBase(primal_variables, dual_variables), A_(A), b_(b) {}

}  // namespace conex
