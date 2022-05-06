#include "supernode_inverse.h"
#include "gtest/gtest.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

GTEST_TEST(SupernodeSubmatrix, Constructor) {
  SupernodeSubmatrix::Parameters p;
  p.num_edges = 3;
  p.spatial_dimension = 2;
  SupernodeSubmatrix solver(p);
}

} // namespace conex
