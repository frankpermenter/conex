#pragma once
namespace conex {

class Data;
class DataTwo;
class ConstraintTwo;
class ConstraintOne;
class LinearConstraint;
class SOCConstraint;

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
