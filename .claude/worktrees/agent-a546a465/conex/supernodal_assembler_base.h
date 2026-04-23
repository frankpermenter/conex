#pragma once
#include <vector>

#include "conex/constraint_interface.h"
#include "conex/newton_step.h"
#include "conex/supernodal_cholesky_data.h"
#include <Eigen/Dense>
namespace conex {

// Manages the transfer of clique submatrix to supernodal data structure.
// The SetDenseData triggers an update the submatrix which
// is store in an Eigen::Map.
class SupernodalAssemblerBase : public ConstraintBase {
 public:
  SupernodalAssemblerBase(const std::vector<int>& shared_variables) {
    SetPrimalVariables(shared_variables);
  }
  SupernodalAssemblerBase(const std::vector<int>& primal_variables,
                          const std::vector<int>& dual_variables) {
    std::vector<int> variables = primal_variables;
    variables.insert(variables.end(), dual_variables.begin(),
                     dual_variables.end());
    SetPrimalVariables(primal_variables);
    SetDualVariables(dual_variables);
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

  void accept(Visitor*) const override {
    throw std::runtime_error("Not implemented.");
  }

  int number_of_variables() const override {
    return primal_variables().size() + dual_variables().size();
  }

  virtual bool is_dynamic() const { return false; }
  virtual bool is_positive_definite() const { return true; }
  virtual std::vector<int> variables() const {
    std::vector<int> variables = primal_variables_;
    variables.insert(variables.end(), dual_variables_.begin(),
                     dual_variables_.end());
    return variables;
  }
  virtual const std::vector<int>& primal_variables() const {
    return primal_variables_;
  }
  virtual const std::vector<int>& dual_variables() const {
    return dual_variables_;
  }

  void UpdateBlocks();
  virtual void SetDenseData() = 0;

  Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> PrimalSubvector(
      const Eigen::MatrixXd& x) const {
    ysegment.resize(primal_variables_.size(), 1);
    Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> z(ysegment.data(),
                                                  ysegment.size(), 1);
    int cnt = 0;
    for (auto i : primal_variables_) {
      z(cnt++) = x(i);
    }
    return z;
  }

  void SetPrimalVariables(const std::vector<int>& variables) {
    primal_variables_ = variables;
    num_variables_ = primal_variables_.size() + dual_variables_.size();
    submatrix_data_.m_ = number_of_variables();
  };
  void SetDualVariables(const std::vector<int>& variables) {
    dual_variables_ = variables;
    num_variables_ = primal_variables_.size() + dual_variables_.size();
    submatrix_data_.m_ = number_of_variables();
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
  std::vector<int> primal_variables_;
  std::vector<int> dual_variables_;

  bool direct_update = false;
  std::vector<DiagonalBlock> diag;
  std::vector<OffDiagonalBlock> off_diag;
  std::vector<OffDiagonalBlock> scatter_block;
  virtual ~SupernodalAssemblerBase(){};
};

}  // namespace conex
