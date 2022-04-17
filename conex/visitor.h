#pragma once
namespace conex {

class Data;
class DataTwo;
class ConstraintTwo;
class ConstraintOne;
class LinearConstraint;
class SOCConstraint;

enum class IDs: int {
 DataOne = 0,
 DataTwo = 1,
 ConstraintTwo = 2,
 ConstraintOne = 3,
 LinearConstraint = 4,
 SOCConstraint = 5,
};



// Visit existing objects and serialize them.
class Visitor {
 public:
  virtual void visit(const Data&) = 0;
  virtual void visit(const DataTwo&) = 0;
  virtual void visit(const ConstraintOne&) = 0;
  virtual void visit(const ConstraintTwo&) = 0;

  virtual void visit(const LinearConstraint&) = 0;
  virtual void visit(const SOCConstraint&) = 0;
  virtual ~Visitor() = default;
};


} // namespace conex
