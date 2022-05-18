#pragma once
#include <memory>

#include "conex/RLDLT.h"
#include "conex/cholesky_solvers.h"
#include "conex/kkt_subsystem.h"
#include "conex/kkt_tree_solver.h"
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {

/*             z1                        z2                      y1 p1 y2 p2 ls
   lf 1.01         0.01            0            0         0.01         0.01 0.01
   0            0            0            0            0            0 0.01 1.01
   0            0         0.01         0.01         0.01            0 0 0 0 0 0
           0            0         1.01         0.01            0            0 0
   0.01         0.01         0.01            0            0            0 0 0
   0.01         1.01            0            0            0         0.01 0.01
   0.01            0            0            0 0.01         0.01            0 0
   1.81797         0.01         0.01            0            0            0 0 0
   0 0.01         0.01            0            0    0.0179686      1.81797 0.01
   0            0            0            0            0            0 0.01 0.01
   0            0    0.0103923    0.0103923          101            0 0 0 0 0 0
           0            0         0.01         0.01   -0.0159988   -0.0159988
   -1.53834e-05      5.04984         0.01         0.01            0            0
   0 0            0         0.01         0.01   -0.0159988   -0.0159988
   -1.53834e-05    0.0498431      5.04984         0.01            0            0
   0 0            0         0.01         0.01 -0.000760711 -0.000760711
   0.0999989    0.0119906    0.0119906       101.01            0            0 0
           1            0            1            0            0            0 0
   0            0            0            0            0            0 0 1 0 1 0
   0            0            0            0            0            0 0 0 0 0 0
   0            0            0           10            0            0 11 0 0 0

*/

class DenseBlock;
class IncomingSpatialVariableBlock;

/* Builds a tree-solver for the supernode submatrix. For three n-coming
edges, the tree has form:

                          W
               (z1, W)  (z2, W)   (z3, W)

where W = (y1, p1, y2, p2, y3, p3, lam spatial, lam flow).


A concrete example matrix with 2 incoming edges is:


                z1                        z2                      y1 p1 y2 p2 ls
lf

        1.01         0.01            0            0         0.01         0.01
0.01            0            0            0            0            0 0 0.01
1.01            0            0         0.01         0.01         0.01 0 0 0 0 0
0 0            0         1.01         0.01            0            0 0 0.01 0.01
0.01            0            0            0 0            0 0.01         1.01 0
0            0         0.01         0.01         0.01            0            0
0 0.01         0.01            0            0      1.81797         0.01 0.01 0
0            0            0            0            0 0.01         0.01 0 0
0.0179686      1.81797         0.01            0            0            0 0 0 0
        0.01         0.01            0            0    0.0103923    0.0103923
101            0            0            0            0            0 0 0 0 0.01
0.01   -0.0159988   -0.0159988 -1.53834e-05      5.04984         0.01 0.01 0 0 0
           0            0         0.01         0.01   -0.0159988   -0.0159988
-1.53834e-05    0.0498431      5.04984         0.01            0            0 0
           0            0         0.01         0.01 -0.000760711 -0.000760711
0.0999989    0.0119906    0.0119906       101.01            0            0 0 1
0            1            0            0            0            0            0
0            0            0            0            0 0            1 0 1 0 0 0
0            0            0            0            0            0 0 0 0 0 0 0
10            0            0           11            0            0            0
*/

class SupernodeSubmatrix {
 public:
  struct Parameters {
    int num_edges;
    int spatial_dimension;
  };

  SupernodeSubmatrix(const Parameters& params,
                     Eigen::Ref<MatrixXd> full_matrix);
  ~SupernodeSubmatrix();

  bool AssembleAndFactor() { return tree_solver_->AssembleAndFactor(); }
  bool Factor() { return tree_solver_->Factor(); }
  void SolveInPlace(Eigen::Ref<Eigen::MatrixXd> x) const {
    tree_solver_->SolveInPlace(x, /*do not permute*/ false);
  }
  void SetData(Eigen::Ref<Eigen::MatrixXd> full_matrix);
  Eigen::MatrixXd MakeKKTMatrix() const {
    return tree_solver_->KKTMatrix(true /*no permutation*/);
  }

 private:
  std::vector<std::unique_ptr<IncomingSpatialVariableBlock>> incoming_blocks_;
  std::unique_ptr<DenseBlock> dense_block_;
  std::unique_ptr<SymmetricLinearSystemTreeSolver> tree_solver_;
  Parameters params_;
};

}  // namespace conex
