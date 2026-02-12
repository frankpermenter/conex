#pragma once

namespace conex {

class LinearConstraint;
class SOCConstraint;
class EqualityConstraints;
class QuadraticConstraintBase;
class DenseLMIConstraint;

template <typename T>
class HermitianPsdConstraint;

template <int dimension>
class MatrixAlgebra;

// Visit existing objects and serialize them.
class Visitor {
 public:
  virtual void visit(const LinearConstraint&) = 0;
  virtual void visit(const SOCConstraint&) = 0;
  virtual void visit(const EqualityConstraints&) = 0;
  virtual void visit(const DenseLMIConstraint&) = 0;
  virtual void visit(const HermitianPsdConstraint<MatrixAlgebra<1>>&) {}
  virtual void visit(const HermitianPsdConstraint<MatrixAlgebra<2>>&){};
  virtual void visit(const HermitianPsdConstraint<MatrixAlgebra<4>>&){};
  virtual void visit(const HermitianPsdConstraint<MatrixAlgebra<8>>&){};
  virtual void visit(const QuadraticConstraintBase&) {}
  virtual ~Visitor() = default;
};

class IVisitable {
 public:
  virtual void accept(Visitor*) const = 0;
  virtual ~IVisitable() = default;
};

class IVariableShape {
 public:
  virtual int number_of_variables() const = 0;
  virtual ~IVariableShape() = default;
};
}  // namespace conex
