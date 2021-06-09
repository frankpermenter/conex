#include <Eigen/Dense>

namespace conex {
struct SelfDualEmbeddingSolution {
  Eigen::VectorXd sol1;
  Eigen::VectorXd sol2;
};

struct SelfDualEmbeddingSystem {
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
};

SelfDualEmbeddingSolution SolveEmbedding(SelfDualEmbeddingSystem& s,
                  const Eigen::VectorXd& b,
                  const double& wt,
                  const double& sqrtmu);

} // namespace conex
