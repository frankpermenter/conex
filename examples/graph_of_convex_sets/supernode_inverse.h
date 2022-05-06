#include <Eigen/Dense>
#include <memory>
#include "conex/kkt_tree_solver.h"

#include "kkt_subsystem.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {

/*             z1                        z2                      y1                   p1                  y2                 p2                  ls                   lf
        1.01         0.01            0            0         0.01         0.01         0.01            0            0            0            0            0            0
        0.01         1.01            0            0         0.01         0.01         0.01            0            0            0            0            0            0
           0            0         1.01         0.01            0            0            0         0.01         0.01         0.01            0            0            0
           0            0         0.01         1.01            0            0            0         0.01         0.01         0.01            0            0            0
        0.01         0.01            0            0      1.81797         0.01         0.01            0            0            0            0            0            0
        0.01         0.01            0            0    0.0179686      1.81797         0.01            0            0            0            0            0            0
        0.01         0.01            0            0    0.0103923    0.0103923          101            0            0            0            0            0            0
           0            0         0.01         0.01   -0.0159988   -0.0159988 -1.53834e-05      5.04984         0.01         0.01            0            0            0
           0            0         0.01         0.01   -0.0159988   -0.0159988 -1.53834e-05    0.0498431      5.04984         0.01            0            0            0
           0            0         0.01         0.01 -0.000760711 -0.000760711    0.0999989    0.0119906    0.0119906       101.01            0            0            0
           1            0            1            0            0            0            0            0            0            0            0            0            0
           0            1            0            1            0            0            0            0            0            0            0            0            0
           0            0            0            0            0            0           10            0            0           11            0            0            0

         
      y1   y2

*/

using IncomingSpatialVariableBlockBase = 
KKTCholeskySystem<CholeskySolver<Eigen::LLT<MatrixXd>, true>>;


class DenseBlock;
class IncomingSpatialVariableBlock;

class SupernodeSubmatrix  {
 public:

  struct Parameters {
    int num_edges;
    int spatial_dimension;
  };

  SupernodeSubmatrix(const Parameters& params);
  ~SupernodeSubmatrix();

  bool Factor() { return tree_solver_->Factor(); }
  void SolveInPlace(Eigen::Ref<Eigen::MatrixXd> x) { return tree_solver_->SolveInPlace(x); }
  void SetData(Eigen::Ref<Eigen::MatrixXd> full_matrix);
 private:
  std::vector<std::unique_ptr<IncomingSpatialVariableBlock>> incoming_blocks_;
  std::unique_ptr<DenseBlock> dense_block_;
  std::unique_ptr<SymmetricLinearSystemTreeSolver> tree_solver_;
  Parameters params_;
};

} // namespace conex
