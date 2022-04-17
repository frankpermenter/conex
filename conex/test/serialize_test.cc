#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

#include "json_parser.h"
#include "serialize.h"
#include "test_constraint.h"
#include "conex/linear_constraint.h"

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

std::unique_ptr<ConstraintBase> create_from_json(const JsonObject& value,
                                           std::string constraint) {
  IDs constraint_type =  static_cast<IDs>(stoi(constraint));
  switch (constraint_type) {
    case IDs::DataOne: {
      Data constraint = fromJson<conex::Data>(value);
      return std::make_unique<Data>(std::move(constraint));
    }
    case IDs::DataTwo: {
      DataTwo constraint = fromJson<conex::DataTwo>(value);
      return std::make_unique<DataTwo>(std::move(constraint));
    }
    case IDs::LinearConstraint: {
      LinearConstraint constraint = fromJson<conex::LinearConstraint>(value);
      return std::make_unique<LinearConstraint>(std::move(constraint));
    }
  }
  throw;
  return std::make_unique<DataTwo>();
}

DataTwo MakeDataTwo() {
  conex::DataTwo constraint;
  constraint.variables = vector<int>{1, 2, 3};
  constraint.matrix = Eigen::MatrixXd::Identity(3, 3);
  constraint.order = 89;
  return constraint;
}

Data MakeData() {
  conex::Data constraint;
  constraint.variables = vector<int>{1, 2, 3};
  constraint.matrix = Eigen::MatrixXd::Identity(3, 3);
  constraint.affine_term.setLinSpaced(3, -1, 2);
  constraint.order = 89;
  constraint.matrices.push_back(Eigen::MatrixXd::Identity(3, 3));
  constraint.matrices.push_back(4 * Eigen::MatrixXd::Identity(3, 3));
  return constraint;
}

LinearConstraint MakeLinearConstraint() {
  Eigen::MatrixXd A(3, 5);
  Eigen::VectorXd C(3);
  for (int i = 0; i < 3; i++) {
    A.row(i).setLinSpaced(A.cols(), -1, 1);
  }
  C.setLinSpaced(A.rows(), -1, 1);
  DUMP(A);
  DUMP(C);
  return LinearConstraint(A, C);
}


void CompareData2(const DataTwo& x, const DataTwo& y) {
  EXPECT_EQ((x.matrix - y.matrix).norm(), 0);
  EXPECT_EQ(x.variables, y.variables);
  EXPECT_EQ(x.order, y.order);
}

void CompareData1(const Data& x, const Data& y) {
  EXPECT_EQ((x.matrix - y.matrix).norm(), 0);
  EXPECT_EQ((x.affine_term - y.affine_term).norm(), 0);
  EXPECT_EQ(x.variables, y.variables);
  EXPECT_EQ(x.order, y.order);
  EXPECT_TRUE(IsEqual(y.matrices, x.matrices));
}

void CompareLinearConstraint(const ConstraintBase* x_ptr, 
                             const ConstraintBase* y_ptr) {
  const auto& x = *dynamic_cast<const LinearConstraint*>(x_ptr);
  const auto& y = *dynamic_cast<const LinearConstraint*>(y_ptr);
  DUMP(x.constraint_matrix());
  DUMP(y.constraint_matrix());
  EXPECT_EQ((x.constraint_matrix() - y.constraint_matrix()).norm(), 0);
  EXPECT_EQ((x.affine_term() - y.affine_term()).norm(), 0);
}


GTEST_TEST(Serialize, TestVirtualInterfaces) {
  std::vector<std::unique_ptr<ConstraintBase>> constraints;
  constraints.emplace_back(new Data(std::move(MakeData())));
  constraints.emplace_back(new DataTwo(std::move(MakeDataTwo())));
  constraints.emplace_back(new LinearConstraint(std::move(MakeLinearConstraint())));

  JsonObject program;
  Serializer serialize;
  program["constraints"] = serialize.GenerateJsonObject(constraints);

  std::vector<std::unique_ptr<ConstraintBase>> constraints_deserialize;
  for (size_t i = 0; i < constraints.size(); ++i) {
    const auto& all_constraints = program["constraints"];
    const auto& constraint_i = all_constraints[to_string(i)];
    const auto& id = constraint_i["id"].value();
    const auto& data = constraint_i["data"];
    constraints_deserialize.push_back(create_from_json(data, id));
  }

  CompareData1(*dynamic_cast<Data*>(constraints_deserialize.at(0).get()),
               MakeData());
  CompareData2(*dynamic_cast<DataTwo*>(constraints_deserialize.at(1).get()),
               MakeDataTwo());

  CompareLinearConstraint(constraints_deserialize.at(2).get(), constraints.at(2).get());
}

GTEST_TEST(Serialize, ConvertData) {
  Data constraint = MakeData();
  JsonObject jsonData = toJson(constraint);
  Data constraint_from_json = fromJson<conex::Data>(jsonData);

  CompareData1(constraint, constraint_from_json);

  std::string jsonString = ConvertToJsonString(jsonData);
  JsonObject jsonDataFromString = ParseJsonString(jsonString);

  Data constraint_from_json_string = fromJson<conex::Data>(jsonDataFromString);
  CompareData1(constraint, constraint_from_json_string);
}

}  // namespace conex
