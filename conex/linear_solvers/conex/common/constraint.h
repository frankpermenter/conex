#pragma once
#include <vector>

#include "conex/common/supernodal_assembler_base.h"
#include "conex/common/workspace.h"
#include <Eigen/Dense>

namespace conex {

class Constraint : public SupernodalAssemblerBase {
 public:
  virtual ~Constraint() = default;

  Workspace workspace() { return do_get_workspace(); }

  int number_of_variables() const override { return do_number_of_variables(); }

  virtual LazySymmetricMatrix* GetLazyEvaluator() { return nullptr; }
  virtual void set_precompute_gram(bool) {}

  bool is_dynamic() const override { return true; }
  bool is_positive_definite() const override { return true; }

 private:
  virtual Workspace do_get_workspace() = 0;
  virtual int do_number_of_variables() const = 0;
};

}  // namespace conex
