#include "supernode_inverse.h"
#include "gtest/gtest.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

GTEST_TEST(SupernodeSubmatrix, Constructor) {
  SupernodeSubmatrix::Parameters p;
  p.num_edges = 3;
  p.spatial_dimension = 2;
  EXPECT_NO_THROW({SupernodeSubmatrix solver(p);});

  p.spatial_dimension = 0;
  p.num_edges = 0;
  EXPECT_THROW({SupernodeSubmatrix solver(p);}, std::runtime_error);
}

GTEST_TEST(SupernodeSubmatrix, MakeKKTMatrix) {
  Eigen::MatrixXd ref(13, 13);
  ref <<
        1.01,    0.01,     0,       0,   0.01,   0.01,         0.01,            0,            0,            0,    0,    0,  0,
        0.01,    1.01,     0,       0,   0.01,   0.01,         0.01,            0,            0,            0,    0,    0,  0,
           0,       0,  1.01,    0.01,      0,      0,            0,         0.01,         0.01,         0.01,    0,    0,  0,
           0,       0,  0.01,    1.01,      0,      0,            0,         0.01,         0.01,         0.01,    0,    0,  0,
        0.01,    0.01,     0,       0,      1,      1,         0.01,            0,            0,            0,    0,    0,  0,
        0.01,    0.01,     0,       0,      2,      2,         0.01,            0,            0,            0,    0,    0,  0,
        0.01,    0.01,     0,       0,      3,      3,          101,            0,            0,            0,    0,    0,  0,
           0,       0,  0.01,    0.01,      4,      4, -1.53834e-05,      5.04984,         0.01,         0.01,    0,    0,  0,
           0,       0,  0.01,    0.01,      5,      5, -1.53834e-05,    0.0498431,      5.04984,         0.01,    0,    0,  0,
           0,       0,  0.01,    0.01,      6,      6,    0.0999989,    0.0119906,    0.0119906,       101.01,    0,    0,  0,
           1,       0,     1,       0,      0,      0,            0,            0,            0,            0,    0,    0,  0,
           0,       1,     0,       1,      0,      0,            0,            0,            0,            0,    0,    0,  0,
           0,       0,     0,       0,      0,      0,           10,            0,            0,           11,    0,    0,  0;

  SupernodeSubmatrix::Parameters p;
  p.num_edges = 2;
  p.spatial_dimension = 2;
  SupernodeSubmatrix solver(p);
  solver.SetData(ref);
  MatrixXd x_calc = solver.MakeKKTMatrix();
  MatrixXd error = (x_calc - ref).selfadjointView<Eigen::Lower>();
  EXPECT_NEAR((error).norm(), 0, 1e-14);
}


} // namespace conex
