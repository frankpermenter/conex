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

using RealHermitianPsdConstraint = HermitianPsdConstraint<MatrixAlgebra<1>>;
using ComplexHermitianPsdConstraint = HermitianPsdConstraint<MatrixAlgebra<2>>;
using QuaternicHermitianPsdConstraint =
    HermitianPsdConstraint<MatrixAlgebra<4>>;
using OctonicHermitianPsdConstraint = HermitianPsdConstraint<MatrixAlgebra<8>>;

// Visit existing objects and serialize them.
class Visitor {
 public:
  virtual void visit(const LinearConstraint&) = 0;
  virtual void visit(const SOCConstraint&) = 0;
  virtual void visit(const EqualityConstraints&) = 0;
  virtual void visit(const DenseLMIConstraint&) = 0;
  virtual ~Visitor() = default;
};

class ConstraintBase {
 public:
  virtual void accept(Visitor*) = 0;
  virtual ~ConstraintBase() = default;
};
}  // namespace conex
