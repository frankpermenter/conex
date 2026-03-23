#pragma once
#include <cstring>
#include <memory>
#include <vector>

#include "conex/constraint_interface.h"
#include "conex/error_checking_macros.h"
#include "conex/newton_step.h"
#include "conex/supernodal_assembler_base.h"
#include "conex/workspace.h"
#include <Eigen/Dense>

namespace conex {

class Constraint : public IVariableShape {
 public:
  virtual ~Constraint() = default;

  void BuildSchurComplement(bool initialize, SchurComplementSystem* sys) {
    do_schur_complement(initialize, sys);
  }

  void SetIdentity() { do_set_identity(); }

  Workspace workspace() { return do_get_workspace(); }

  int number_of_variables() const override { return do_number_of_variables(); }

  virtual LazySymmetricMatrix* GetLazyEvaluator() { return nullptr; }
  virtual void set_precompute_gram(bool) {}

 private:
  virtual void do_schur_complement(bool initialize,
                                   SchurComplementSystem* sys) = 0;
  virtual void do_set_identity() = 0;
  virtual Workspace do_get_workspace() = 0;
  virtual int do_number_of_variables() const = 0;
};

class SupernodalAssemblerConstraint : public SupernodalAssemblerBase {
 public:
  SupernodalAssemblerConstraint(const std::vector<int>& variables,
                                Constraint* W)
      : SupernodalAssemblerBase(variables) {
    workspace_ = W;
    CONEX_CHECK(W);
  }

  virtual bool is_dynamic() const override { return true; }
  virtual bool is_positive_definite() const override { return true; }
  Constraint* constraint() { return workspace_; }

  virtual void SetDenseData() {
    if (!submatrix_data_.initialized) {
#if CONEX_DEBUG_MESSAGES
      std::cerr << "Performing self initialization of SupernodalAssembler. Did "
                   "you forget to initialize workspace?";
#endif
      Workspace workspace = Workspace(&submatrix_data_);
      memory_.resize(SizeOf(workspace));
      Initialize(&workspace, memory_.data());
    }

    if (workspace_) {
      workspace_->BuildSchurComplement(true, &submatrix_data_);
    } else {
      throw std::runtime_error("Supernodal assembler data source is not set.");
    }
  }

  LazySymmetricMatrix* GetLazyEvaluator() override {
    return workspace_->GetLazyEvaluator();
  }
  void set_precompute_gram(bool v) override {
    workspace_->set_precompute_gram(v);
  }

  SupernodalAssemblerConstraint(){};
  Constraint* workspace_ = NULL;
  Eigen::VectorXd memory_;
};

}  // namespace conex
