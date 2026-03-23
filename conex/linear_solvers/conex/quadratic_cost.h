#include "conex/error_checking_macros.h"
#include "conex/supernodal_assembler_base.h"
#include "conex/workspace.h"
#include <Eigen/Dense>

namespace conex {
class SupernodalAssemblerStatic : public SupernodalAssemblerBase {
 public:
  SupernodalAssemblerStatic(const Eigen::MatrixXd& A,
                            const std::vector<int>& variables)
      : SupernodalAssemblerBase(variables), A_(A) {
    if (A_.rows() != A.cols()) {
      throw std::runtime_error("Static assembler requires square matrix");
    }
  }

  int UpdateMatrix(double value, int row, int col) {
    CONEX_RETURN_ON_FAIL(row < A_.rows() && col < A_.cols(),
                         "Indices are out of bounds.");

    A_(row, col) = value;
    return CONEX_SUCCESS;
  }

  virtual void SetDenseData() override {
    if (!submatrix_data_.initialized) {
#if CONEX_DEBUG_MESSAGES
      std::cerr
          << "Performing self initialization of SupernodalAssemblerStatic. Did "
             "you forget to initialize workspace?";
#endif
      Workspace workspace = Workspace(&submatrix_data_);
      memory_.resize(SizeOf(workspace));
      Initialize(&workspace, memory_.data());
    }
    submatrix_data_.setZero();
    submatrix_data_.G = A_;
  }
  Eigen::MatrixXd A_;
  Eigen::VectorXd memory_;
};

class SupernodalAssemblerQuadratic : public SupernodalAssemblerStatic {
 public:
  SupernodalAssemblerQuadratic(const Eigen::MatrixXd& A,
                               const std::vector<int>& variables)
      : SupernodalAssemblerStatic(A, variables) {}
  Eigen::MatrixXd CostMatrix() const { return A_; }
};
}  // namespace conex
