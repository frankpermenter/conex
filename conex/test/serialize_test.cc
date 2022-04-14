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
  Json::Value serialize() {
    Json::Value value;
    value.data.children["data"] = generate_json();
    value.data.children["id"].data.string = to_string(type_id());
    return value;
  }

  virtual ~ConstraintBase() = default;

 private:
  virtual Json::Value generate_json() = 0;
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
  Json::Value generate_json() override { return toJson(*this); }
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
  Json::Value generate_json() override { return toJson(*this); }
  int type_id() override { return ConstraintTwoID; }
};

std::unique_ptr<ConstraintBase> create_from_json(const Json::Value& value,
                                                 int constraint_type) {
  // switch (string_to_id.at(value.data.children.at("type").data.string)) {
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

GTEST_TEST(Serialize, ConvertProgram) {
  std::vector<std::unique_ptr<ConstraintBase>> constraints;
  constraints.emplace_back(new Constraint(std::move(MakeConstraint())));
  constraints.emplace_back(new ConstraintTwo(std::move(MakeConstraintTwo())));

  Json::Value program;
  for (size_t i = 0; i < constraints.size(); ++i) {
    program.data.children["constraints"].data.children[to_string(i)] =
        constraints.at(i)->serialize();
  }

  // Json::Value jsonConstraintFromString = MakeValue(MakeJsonString(program));

  std::vector<std::unique_ptr<ConstraintBase>> constraints_deserialize(2);
  for (size_t i = 0; i < constraints.size(); ++i) {
    const auto& all_constraints = program.data.children.at("constraints");
    const auto& constraint_i = all_constraints.data.children.at(to_string(i));
    const auto& id = constraint_i.data.children.at("id").data.string;
    const auto& data = constraint_i.data.children.at("data");
    constraints_deserialize.at(i) = create_from_json(data, stoi(id));
  }
}

GTEST_TEST(Serialize, ConvertConstraint) {
  Constraint constraint = MakeConstraint();
  Json::Value jsonConstraint = toJson(constraint);
  Constraint constraint_from_json = fromJson<conex::Constraint>(jsonConstraint);

  EXPECT_TRUE(IsEqual(constraint_from_json.matrices, constraint.matrices));

  EXPECT_EQ((constraint.matrix - constraint_from_json.matrix).norm(), 0);
  EXPECT_EQ((constraint.affine_term - constraint_from_json.affine_term).norm(),
            0);
  EXPECT_EQ(constraint.variables, constraint_from_json.variables);
  EXPECT_EQ(constraint.order, constraint_from_json.order);

  std::string jsonString = MakeJsonString(jsonConstraint);
  Json::Value jsonConstraintFromString = MakeValue(jsonString);

  Constraint constraint_from_json_string =
      fromJson<conex::Constraint>(jsonConstraintFromString);

  EXPECT_EQ((constraint.matrix - constraint_from_json_string.matrix).norm(), 0);
  EXPECT_EQ(
      (constraint.affine_term - constraint_from_json_string.affine_term).norm(),
      0);
  EXPECT_EQ(constraint.variables, constraint_from_json_string.variables);
  EXPECT_EQ(constraint.order, constraint_from_json_string.order);

  EXPECT_TRUE(
      IsEqual(constraint_from_json_string.matrices, constraint.matrices));
}

}  // namespace conex
