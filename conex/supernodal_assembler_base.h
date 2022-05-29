#pragma once
#include <vector>

#include "conex/newton_step.h"
#include "conex/supernodal_cholesky_data.h"
#include <Eigen/Dense>
namespace conex {

class SupernodalAssemblerBase {
 public:
  SupernodalAssemblerBase(const std::vector<int>& shared_variables,
                          int num_private) {
    SetVariables(shared_variables, num_private);
  }
  SupernodalAssemblerBase(){};

  // Entries of diagonal block to update
  void BindDiagonalBlock(const DiagonalBlock* data);
  // Entries of off diagonal block to update
  void BindOffDiagonalBlock(const OffDiagonalBlock* data);
  void Reset() {
    diag.clear();
    off_diag.clear();
    scatter_block.clear();
    direct_update = false;
  }

  virtual bool is_dynamic() const { return false; }
  virtual bool is_positive_definite() const { return true; }
  virtual int number_of_auxiliary_variables() const { return 0; }
  virtual std::vector<int> variables() const { return variables_; }

  void UpdateBlocks();
  virtual void SetDenseData() = 0;

  Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> Subvector(
      const Eigen::MatrixXd& x) const {
    ysegment.resize(variables_.size(), 1);
    Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> z(ysegment.data(),
                                                  ysegment.size(), 1);
    int cnt = 0;
    for (auto i : variables_) {
      z(cnt++) = x(i);
    }
    return z;
  }

  int NumberOfVariables() { return num_variables_; };

  // TODO(frankpermenter): deprecate this method. It
  // is currently used by Drake.
  void SetNumberOfVariables(int num_variables) {
    num_variables_ = num_variables;
    submatrix_data_.m_ = num_variables;
  };

  void SetVariables(const std::vector<int>& variables, int num_private) {
    variables_ = variables;
    SetNumberOfVariables(variables.size() + num_private);
  };

  SchurComplementSystem* submatrix_data() { return &submatrix_data_; }

 protected:
  WorkspaceSchurComplement submatrix_data_;
  mutable Eigen::VectorXd ysegment;
  double GetCoeff(int i, int j);

  void Increment(const int* r, int sizer, const int* c, int sizec,
                 Eigen::Map<Eigen::MatrixXd>* data);
  void Set(const int* r, int sizer, const int* c, int sizec,
           Eigen::Map<Eigen::MatrixXd>* data);

  void IncrementLowerTri(const int* r, int sizer, const int* c, int sizec,
                         Eigen::Map<Eigen::MatrixXd>* data);
  void SetLowerTri(const int* r, int sizer, const int* c, int sizec,
                   Eigen::Map<Eigen::MatrixXd>* data);

  void SetDiagonalBlock(const std::vector<int>& r,
                        Eigen::Map<Eigen::MatrixXd>* data);

  void Scatter(const int* r, int sizer, const int* c, int sizec, double** data);
  int num_variables_;
  std::vector<int> variables_;

  bool direct_update = false;
  std::vector<DiagonalBlock> diag;
  std::vector<OffDiagonalBlock> off_diag;
  std::vector<OffDiagonalBlock> scatter_block;
  virtual ~SupernodalAssemblerBase(){};
};

}  // namespace conex
