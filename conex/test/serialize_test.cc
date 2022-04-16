#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

#include "serialize.h"

namespace conex {
using Eigen::MatrixXd;
bool IsEqual(const std::vector<MatrixXd>& m1, const std::vector<MatrixXd>& m2) {
  if (m1.size() != m2.size()) {
    return false;
  }
  for (size_t i = 0; i < m1.size(); ++i) {
    if ((m2.at(i) - m1.at(i)).norm() != 0) {
      return false;
    }
  }
  return true;
}

struct ConstraintBase {
  Value serialize() {
    Value value;
    value["data"] = generate_json();
    value["id"].value() = to_string(type_id());
    return value;
  }

  virtual ~ConstraintBase() = default;

 private:
  virtual Value generate_json() = 0;
  virtual int type_id() = 0;
};

enum : int {
  ConstraintOneID = 0,
  ConstraintTwoID = 1,
};

struct Constraint : ConstraintBase {
  int order;
  std::vector<int> variables;
  Eigen::MatrixXd matrix;
  vector<Eigen::MatrixXd> matrices;
  Eigen::VectorXd affine_term;

  constexpr static auto properties =
      std::make_tuple(property(&Constraint::order, "order"),
                      property(&Constraint::matrix, "matrix"),
                      property(&Constraint::matrices, "matrices"),
                      property(&Constraint::affine_term, "affine_term"),
                      property(&Constraint::variables, "variables"));

 private:
  Value generate_json() override { return toJson(*this); }
  int type_id() override { return ConstraintOneID; }
};

struct ConstraintTwo : ConstraintBase {
  int order;
  std::vector<int> variables;
  Eigen::MatrixXd matrix;

  constexpr static auto properties =
      std::make_tuple(property(&ConstraintTwo::order, "order"),
                      property(&ConstraintTwo::matrix, "matrix"),
                      property(&ConstraintTwo::variables, "variables"));

 private:
  Value generate_json() override { return toJson(*this); }
  int type_id() override { return ConstraintTwoID; }
};

std::unique_ptr<ConstraintBase> create_from_json(const Value& value,
                                                 int constraint_type) {
  // switch (string_to_id.at(value.children().at("type").value())) {
  switch (constraint_type) {
    case ConstraintOneID: {
      Constraint constraint = fromJson<conex::Constraint>(value);
      return std::make_unique<Constraint>(std::move(constraint));
    }
    case ConstraintTwoID: {
      ConstraintTwo constraint = fromJson<conex::ConstraintTwo>(value);
      return std::make_unique<ConstraintTwo>(std::move(constraint));
    }
  }
  throw;
  return std::make_unique<ConstraintTwo>();
}

ConstraintTwo MakeConstraintTwo() {
  conex::ConstraintTwo constraint;
  constraint.variables = vector<int>{1, 2, 3};
  constraint.matrix = Eigen::MatrixXd::Identity(3, 3);
  constraint.order = 89;
  return constraint;
}

Constraint MakeConstraint() {
  conex::Constraint constraint;
  constraint.variables = vector<int>{1, 2, 3};
  constraint.matrix = Eigen::MatrixXd::Identity(3, 3);
  constraint.affine_term.setLinSpaced(3, -1, 2);
  constraint.order = 89;
  constraint.matrices.push_back(Eigen::MatrixXd::Identity(3, 3));
  constraint.matrices.push_back(4 * Eigen::MatrixXd::Identity(3, 3));
  return constraint;
}


void CompareConstraints2(const ConstraintTwo& x, const ConstraintTwo& y) {
  EXPECT_EQ((x.matrix - y.matrix).norm(), 0);
  EXPECT_EQ(x.variables, y.variables);
  EXPECT_EQ(x.order, y.order);
}

void CompareConstraints1(const Constraint& x, const Constraint& y) {
  EXPECT_EQ((x.matrix - y.matrix).norm(), 0);
  EXPECT_EQ((x.affine_term - y.affine_term).norm(), 0);
  EXPECT_EQ(x.variables, y.variables);
  EXPECT_EQ(x.order, y.order);
  EXPECT_TRUE( IsEqual(y.matrices, x.matrices));
}

GTEST_TEST(Serialize, TestVirtualInterfaces) {
  std::vector<std::unique_ptr<ConstraintBase>> constraints;
  constraints.emplace_back(new Constraint(std::move(MakeConstraint())));
  constraints.emplace_back(new ConstraintTwo(std::move(MakeConstraintTwo())));

  Value program;
  for (size_t i = 0; i < constraints.size(); ++i) {
    program["constraints"][to_string(i)] =
        constraints.at(i)->serialize();
  }

  std::vector<std::unique_ptr<ConstraintBase>> constraints_deserialize(2);
  for (size_t i = 0; i < constraints.size(); ++i) {
    const auto& all_constraints = program["constraints"];
    const auto& constraint_i = all_constraints[to_string(i)];
    const auto& id = constraint_i["id"].value();
    const auto& data = constraint_i["data"];
    constraints_deserialize.at(i) = create_from_json(data, stoi(id));
  }
  
  CompareConstraints1(*dynamic_cast<Constraint*>(constraints_deserialize.at(0).get()), 
                      MakeConstraint());
  CompareConstraints2(*dynamic_cast<ConstraintTwo*>(constraints_deserialize.at(1).get()), 
                      MakeConstraintTwo());
}

GTEST_TEST(Serialize, ConvertConstraint) {
  Constraint constraint = MakeConstraint();
  Value jsonConstraint = toJson(constraint);
  Constraint constraint_from_json = fromJson<conex::Constraint>(jsonConstraint);

  CompareConstraints1(constraint, constraint_from_json);

  std::string jsonString = ConvertToJsonString(jsonConstraint);
  Value jsonConstraintFromString = ParseJsonString(jsonString);

  Constraint constraint_from_json_string =
      fromJson<conex::Constraint>(jsonConstraintFromString);
  CompareConstraints1(constraint, constraint_from_json_string);

}

}  // namespace conex
