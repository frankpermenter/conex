#pragma once
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include "conex/visitor.h"
#include "conex/constraint_interface.h"
namespace conex {

// Serializable = DoGetData(Constraint)
// Serializable( accept(  ) )
// 
// Serialize can implement a virtual.

class Data;
class DataTwo;
class ConstraintTwo;
class ConstraintOne;




struct Data : ConstraintBase {
  int order;
  std::vector<int> variables;
  Eigen::MatrixXd matrix;
  Eigen::VectorXd affine_term;
  std::vector<Eigen::MatrixXd> matrices;

  void accept(Visitor* v) override { v->visit(*this); }
};

class ConstraintOne : ConstraintBase {
 public:
  const Data& GetParameters() const { return data_; }
  virtual void accept(Visitor*) = 0;
 private:
  Data data_;
};

struct DataTwo : ConstraintBase {
  int order;
  std::vector<int> variables;
  Eigen::MatrixXd matrix;
  void accept(Visitor* v) override { v->visit(*this); }
};

class ConstraintTwo : ConstraintBase {
 public:
  const DataTwo& GetParameters() const { return data_; }
 private:
  DataTwo data_;
  virtual void accept(Visitor*) = 0;
};



}  // namespace conex
