#pragma once
#include <vector>
#include <Eigen/Dense>
namespace conex {


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


  void accept(Visitor* v) override { return v->visit(*this); }
};

struct DataTwo : DataBase {
  int order;
  std::vector<int> variables;
  Eigen::MatrixXd matrix;

  void accept(Visitor* v) override { return v->visit(*this); }
};

}  // namespace conex
