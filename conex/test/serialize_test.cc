#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

#include "json_parser.h"
#include "serialize.h"
#include "test_constraint.h"

namespace conex {

// Data Factory
//
// Transformation:
//
//   1)  json -> Data               (serializer)
//   2)  Data -> InterfacePointer   (factory?)
//   3)  InterfacePointer -> json.
//
// Implementations:
//
//   1) switch json[type_id]:
//        case type_id:
//           data  = Make<Data>(json[data])
//
//   2) InterfacePointer* Factory(Data) { return Object(Data) }  // overload on
//   DataStructType.
//
//   3a) class Object : InterfacePointer
//       generate_json() { to_json(Data) }   )
//
//       So, class must know about data and serializer.
//
//   3b) class VisitorI
//        visit(Data A);
//        visit(Data B);
//        visit(Data C);
//        visit(Data D);

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

std::unique_ptr<DataBase> create_from_json(const JsonObject& value,
                                           int constraint_type) {
  switch (constraint_type) {
    case DataOneID: {
      Data constraint = fromJson<conex::Data>(value);
      return std::make_unique<Data>(std::move(constraint));
    }
    case DataTwoID: {
      DataTwo constraint = fromJson<conex::DataTwo>(value);
      return std::make_unique<DataTwo>(std::move(constraint));
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

GTEST_TEST(Serialize, TestVirtualInterfaces) {
  std::vector<std::unique_ptr<DataBase>> constraints;
  constraints.emplace_back(new Data(std::move(MakeData())));
  constraints.emplace_back(new DataTwo(std::move(MakeDataTwo())));

  JsonObject program;
  Serializer serialize;
  program["constraints"] = serialize.GenerateJsonObject(constraints);

  std::vector<std::unique_ptr<DataBase>> constraints_deserialize(2);
  for (size_t i = 0; i < constraints.size(); ++i) {
    const auto& all_constraints = program["constraints"];
    const auto& constraint_i = all_constraints[to_string(i)];
    const auto& id = constraint_i["id"].value();
    const auto& data = constraint_i["data"];
    constraints_deserialize.at(i) = create_from_json(data, stoi(id));
  }

  CompareData1(*dynamic_cast<Data*>(constraints_deserialize.at(0).get()),
               MakeData());
  CompareData2(*dynamic_cast<DataTwo*>(constraints_deserialize.at(1).get()),
               MakeDataTwo());
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
