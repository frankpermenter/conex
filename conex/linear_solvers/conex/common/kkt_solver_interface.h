#pragma once

#include "conex/common/error_checking_macros.h"
#include <Eigen/Dense>

namespace conex {

class KKTSolverBase {
 public:
  void Assemble() {
    DoAssemble();
    assembled_ = true;
    factored_ = false;
  }

  bool AssembleAndFactor() {
    assembled_ = false;
    factored_ = false;
    bool ok = DoAssembleAndFactor();
    if (ok) {
      assembled_ = true;
      factored_ = true;
    }
    return assembled_ && factored_;
  }

  bool Factor() {
    CONEX_DEMAND(assembled_, "System has not been assembled.");
    assembled_ = false;
    if (DoFactor()) {
      factored_ = true;
    } else {
      factored_ = false;
    }
    return factored_;
  }

  Eigen::MatrixXd Solve(Eigen::Ref<const Eigen::MatrixXd> b,
                        bool permute_to_elimination_order = true) const {
    CONEX_DEMAND(factored_, "System has not been factored.");
    Eigen::MatrixXd x = b;
    DoSolveInPlace(x, permute_to_elimination_order);
    return x;
  }

  Eigen::MatrixXd KKTMatrix(bool permute_to_elimination_order = false) const {
    CONEX_DEMAND(assembled_,
                 "System has not been assembled or is factored in place.");
    return DoKKTMatrix(permute_to_elimination_order);
  }

  virtual ~KKTSolverBase() = default;

 private:
  virtual void DoAssemble() = 0;
  virtual bool DoFactor() = 0;
  virtual bool DoAssembleAndFactor() {
    DoAssemble();
    return DoFactor();
  }
  virtual void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                              bool permute_to_elimination_order) const = 0;
  virtual Eigen::MatrixXd DoKKTMatrix(
      bool permute_to_elimination_order) const = 0;
  bool factored_ = false;
  bool assembled_ = false;
};

}  // namespace conex
