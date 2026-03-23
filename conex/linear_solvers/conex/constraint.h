#pragma once
#include <vector>

#include "conex/error_checking_macros.h"
#include "conex/newton_step.h"
#include "conex/supernodal_assembler_base.h"
#include "conex/workspace.h"
#include <Eigen/Dense>

namespace conex {

class Constraint : public SupernodalAssemblerBase {
 public:
  virtual ~Constraint() = default;

  void BuildSchurComplement(bool initialize, SchurComplementSystem* sys) {
    do_schur_complement(initialize, sys);
  }

  Workspace workspace() { return do_get_workspace(); }

  int number_of_variables() const override { return do_number_of_variables(); }

  virtual LazySymmetricMatrix* GetLazyEvaluator() { return nullptr; }
  virtual void set_precompute_gram(bool) {}

  bool is_dynamic() const override { return true; }
  bool is_positive_definite() const override { return true; }

  void SetDenseData() override {
    if (!submatrix_data_.initialized) {
      Workspace ws = Workspace(&submatrix_data_);
      dense_data_memory_.resize(SizeOf(ws));
      Initialize(&ws, dense_data_memory_.data());
    }
    BuildSchurComplement(true, &submatrix_data_);
  }

 private:
  virtual void do_schur_complement(bool initialize,
                                   SchurComplementSystem* sys) = 0;
  virtual Workspace do_get_workspace() = 0;
  virtual int do_number_of_variables() const = 0;

  Eigen::VectorXd dense_data_memory_;
};

}  // namespace conex
