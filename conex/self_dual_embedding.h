#include <Eigen/Dense>
#include "conex/kkt_solver.h"

namespace conex {
struct SelfDualEmbeddingSolution {
  Eigen::VectorXd sol1;
  Eigen::VectorXd sol2;
};

/*
struct SelfDualEmbeddingSystem {
  SelfDualEmbeddingSystem(int m) : 
      AWA(m, m), AW(m, 1), AQc(m, 1), AQe(m, 1), Ae(m, 1) {}
  Eigen::MatrixXd AWA;
  Eigen::VectorXd AW;
  Eigen::VectorXd AQc;
  Eigen::VectorXd AQe;
  Eigen::VectorXd Ae;
  double inner_product_of_c_and_e;
  double inner_product_of_c_and_w;
  double inner_product_of_c_and_Qc;
  double inner_product_of_c_and_Qc_scale;
  double inner_product_of_c_and_Qe;
};*/

SelfDualEmbeddingSolution SolveEmbedding(SelfDualEmbeddingSystem& s,
                                         const Eigen::VectorXd& b,
                                         const double& wt,
                                         const double& sqrtmu);

SelfDualEmbeddingSolution SolveEmbedding(SelfDualEmbeddingSystem& s,
                                         Solver& kkt_solver,
                                         const Eigen::VectorXd& b,
                                         const double& wt,
                                         const double& sqrtmu);



}  // namespace conex
