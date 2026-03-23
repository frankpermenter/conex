#pragma once
#include <chrono>

#include "conex/error_checking_macros.h"
#include <Eigen/Dense>

namespace conex {

struct KKTSolverTimings {
  double assemble_and_factor_us = 0;
  double solve_us = 0;
  void Reset() { assemble_and_factor_us = 0; solve_us = 0; }
};

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
    auto t0 = Clock::now();
    bool ok = DoAssembleAndFactor();
    if (record_timings_) {
      timings_.assemble_and_factor_us +=
          std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
    }
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

  void SolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                    bool permute_to_elimination_order = true) const {
    CONEX_DEMAND(factored_, "System has not been factored.");
    auto t0 = Clock::now();
    DoSolveInPlace(b, permute_to_elimination_order);
    if (record_timings_) {
      timings_.solve_us +=
          std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
    }
  }

  Eigen::MatrixXd KKTMatrix(bool permute_to_elimination_order = false) const {
    CONEX_DEMAND(assembled_,
                 "System has not been assembled or is factored in place.");
    return DoKKTMatrix(permute_to_elimination_order);
  }

  void SetRecordTimings(bool v) { record_timings_ = v; }
  const KKTSolverTimings& timings() const { return timings_; }
  void ResetTimings() { timings_.Reset(); }

  virtual ~KKTSolverBase() = default;

 private:
  using Clock = std::chrono::high_resolution_clock;
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
  bool record_timings_ = false;
  mutable KKTSolverTimings timings_;
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
