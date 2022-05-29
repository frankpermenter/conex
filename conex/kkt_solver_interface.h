#pragma once
#include "conex/error_checking_macros.h"
#include <Eigen/Dense>

namespace conex {

class KKTSolverBase {
 public:
  void Assemble() {
    DoAssemble();
    assembled_ = true;
  }

  bool Factor() {
    assembled_ = false;
    if (DoFactor()) {
      factored_ = true;
    } else {
      factored_ = false;
    }
    return factored_;
  }

  void SolveInPlace(Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>* b,
                    bool permute_to_elimination_order = true) const {
    CONEX_DEMAND(factored_, "System has not been factored.");
    DoSolveInPlace(b, permute_to_elimination_order);
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
  virtual void DoSolveInPlace(Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>* b,
                              bool permute_to_elimination_order) const = 0;
  virtual Eigen::MatrixXd DoKKTMatrix(
      bool permute_to_elimination_order) const = 0;
  bool factored_ = false;
  bool assembled_ = false;
};

/*
class KKTSystemTypeEraser {
  template <typename T>
  KKTSystemTypeEraser(T obj) : self_(new Dispatcher<T>(std::move(obj))) {}

  bool Factor() { return self_->Factor(); }
  Eigen::VectorXd Solve(const Eigen::VectorXd& b) const;
  void SolveInPlace(Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>* b) const;
  Eigen::MatrixXd KKTMatrix() const;

 private:
  template <typename T>
  class Dispatcher final : KKTSolverBase {
   public:
    Dispatcher(T t) : data(std::move(t)) {}
    bool DoFactor() override { return data.Factor(); }
    Eigen::VectorXd DoSolve(const Eigen::VectorXd& b) const override {
      return data.Solve(b);
    }
    void DoSolveInPlace(
        Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>* b) const override {
      return data.SolveInPlace(b);
    }
    Eigen::MatrixXd DoKKTMatrix() const;
    T data;
  };
  std::unique_ptr<KKTSolverBase> self_;
};
*/
}  // namespace conex
