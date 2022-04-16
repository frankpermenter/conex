#pragma once
#include <vector>
#include <Eigen/Dense>
namespace conex {

template <typename Class, typename T>
struct Property {
  constexpr Property(T Class::*aMember, const char* aName)
      : member{aMember}, name{aName} {}

  using Type = T;

  T Class::*member;
  const char* name;
};

template <typename Class, typename T>
constexpr auto MakeProperty(T Class::*member, const char* name) {
  return Property<Class, T>{member, name};
}

class Data;
class DataTwo;
class Visitor {
 public:
  virtual void visit(const Data&) = 0;
  virtual void visit(const DataTwo&) = 0;
  virtual ~Visitor() = default;
};

struct DataBase {
  virtual ~DataBase() = default;
  virtual void accept(Visitor*) = 0;
};


enum : int {
  DataOneID = 0,
  DataTwoID = 1,
};

struct Data : DataBase {
  int order;
  std::vector<int> variables;
  Eigen::MatrixXd matrix;
  Eigen::VectorXd affine_term;
  std::vector<Eigen::MatrixXd> matrices;

  constexpr static auto properties =
      std::make_tuple(MakeProperty(&Data::order, "order"),
                      MakeProperty(&Data::matrix, "matrix"),
                      MakeProperty(&Data::matrices, "matrices"),
                      MakeProperty(&Data::affine_term, "affine_term"),
                      MakeProperty(&Data::variables, "variables"));

  void accept(Visitor * v) override { return v->visit(*this); }
};


struct DataTwo : DataBase {
  int order;
  std::vector<int> variables;
  Eigen::MatrixXd matrix;

  constexpr static auto properties =
      std::make_tuple(MakeProperty(&DataTwo::order, "order"),
                      MakeProperty(&DataTwo::matrix, "matrix"),
                      MakeProperty(&DataTwo::variables, "variables"));

  void accept(Visitor * v) override { return v->visit(*this); }
};

} //namespace conex
