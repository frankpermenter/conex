#include "supernode_inverse.h"
#include "gtest/gtest.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

GTEST_TEST(SupernodeSubmatrix, Constructor) {
  SupernodeSubmatrix::Parameters p;
  SupernodeSubmatrix solver(p);
}

} // namespace conex
