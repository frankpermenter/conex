#include "conex/serialize.h"

#include <iostream>
#include <map>
#include <numeric>
#include <tuple>

#include "conex/cone_program.h"
#include "conex/debug_macros.h"
#include "conex/dense_lmi_constraint.h"
#include "conex/equality_constraint.h"
#include "conex/json_parser.h"
#include "conex/linear_constraint.h"
#include "conex/soc_constraint.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

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

template <typename T>
T MakeEqualityConstraint(int value) {
  Eigen::MatrixXd A(1, 4);
  Eigen::VectorXd b(1);
  A.row(0).setConstant(1);
  b.setConstant(1);
  return T(A, b);
}

template <typename T>
T MakeMatrixConstraint(int value) {
  Eigen::MatrixXd A(2, 3);
  Eigen::VectorXd C(2);
  for (int i = 0; i < A.rows(); i++) {
    A.row(i).setLinSpaced(A.cols(), -value, value);
  }
  C.setLinSpaced(A.rows(), 200, 400);
  return T(A, C);
}

template <typename T>
T MakeSocConstraint(int value) {
  Eigen::MatrixXd A(3, 3);
  Eigen::VectorXd C(3);
  for (int i = 0; i < A.rows(); i++) {
    A.row(i).setLinSpaced(A.cols(), -value, value);
  }
  C.setZero();
  C(0) = 100;
  return T(A, C);
}

DenseLMIConstraint MakeLMIConstraint(int value) {
  int order = 3;
  Eigen::MatrixXd A0(order, order);
  // clang-format off
  A0 << 0, 1, 5,
        1, 2, 8,
        5, 8, 9;
  Eigen::MatrixXd A1(order, order);
  A1 << 1, 1, 2,
        1, 0, 8,
        2, 8, 9;
  Eigen::MatrixXd A2(order, order);
  A2 << 1, 0, 0,
        0, 0, 8,
        0, 8, 9;
  Eigen::MatrixXd C(order, order);
  C << 1000, 1, 0,
       1, 1000, 1,
       0, 1, 2000;
  // clang-format on
  std::vector<MatrixXd> matrices;
  matrices.push_back(A0);
  matrices.push_back(A1);
  matrices.push_back(A2);
  return DenseLMIConstraint(order, matrices, C);
}

template <typename T>
void CompareMatrixConstraint(const ConstraintBase* x_ptr,
                             const ConstraintBase* y_ptr) {
  const auto& x = *dynamic_cast<const T*>(x_ptr);
  const auto& y = *dynamic_cast<const T*>(y_ptr);
  EXPECT_EQ((x.constraint_matrix() - y.constraint_matrix()).norm(), 0);
  EXPECT_EQ((x.affine_term() - y.affine_term()).norm(), 0);
}

void CompareLMIConstraint(const ConstraintBase* x_ptr,
                          const ConstraintBase* y_ptr) {
  using T = DenseLMIConstraint;
  const auto& x = *dynamic_cast<const T*>(x_ptr);
  const auto& y = *dynamic_cast<const T*>(y_ptr);
  EXPECT_EQ((x.affine_term() - y.affine_term()).norm(), 0);
  EXPECT_TRUE(IsEqual(x.constraint_matrices(), y.constraint_matrices()));
}

std::vector<std::unique_ptr<ConstraintBase>> MakeConstraints() {
  std::vector<std::unique_ptr<ConstraintBase>> constraints;
  constraints.emplace_back(new LinearConstraint(
      std::move(MakeMatrixConstraint<LinearConstraint>(1))));
  constraints.emplace_back(
      new SOCConstraint(std::move(MakeSocConstraint<SOCConstraint>(2))));

  constraints.emplace_back(new EqualityConstraints(
      std::move(MakeEqualityConstraint<EqualityConstraints>(3))));

  constraints.emplace_back(
      new DenseLMIConstraint(std::move(MakeLMIConstraint(3))));
  return constraints;
}

GTEST_TEST(Serialize, TestSerializeDeserialize) {
  std::vector<std::unique_ptr<ConstraintBase>> constraints = MakeConstraints();
  JsonObject program;
  Serializer serialize;
  program["constraints"] = serialize.GenerateJsonObject(constraints);
  program["num_constraints"] = ConvertToJson(3);

  ConstraintManager c;
  std::vector<std::unique_ptr<ConstraintBase>> constraints_deserialize;
  for (size_t i = 0; i < constraints.size(); ++i) {
    const auto& all_constraints = program["constraints"];
    const auto& constraint_i = all_constraints[to_string(i)];
    constraints_deserialize.push_back(MakeConstraintFromJSON(constraint_i));
  }

  int i = 0;
  CompareMatrixConstraint<LinearConstraint>(constraints_deserialize.at(i).get(),
                                            constraints.at(i).get());
  i++;
  CompareMatrixConstraint<SOCConstraint>(constraints_deserialize.at(i).get(),
                                         constraints.at(i).get());

  i++;
  CompareMatrixConstraint<EqualityConstraints>(
      constraints_deserialize.at(i).get(), constraints.at(i).get());

  i++;
  CompareLMIConstraint(constraints_deserialize.at(i).get(),
                       constraints.at(i).get());
}

std::vector<int> MakeVariableList(ConstraintBase* c) {
  std::vector<int> vars(c->number_of_variables());
  std::iota(vars.begin(), vars.end(), 0);
  return vars;
}

GTEST_TEST(DeserializeConeProgram, TestSerializeDeserialize) {
  std::vector<std::unique_ptr<ConstraintBase>> constraints = MakeConstraints();
  JsonObject program;
  Serializer serialize;
  program["quadratic_costs"]["0"]["cost_matrix"] =
      ConvertToJson(Eigen::MatrixXd::Identity(4, 4));
  program["quadratic_costs"]["0"]["variables"] =
      ConvertToJson(std::vector<int>{0, 1, 2, 3});
  program["num_quadratic_costs"] = ConvertToJson(static_cast<int>(1));

  Eigen::VectorXd linear_cost = Eigen::VectorXd::LinSpaced(4, -1, 1);
  program["linear_cost"] = ConvertToJson(linear_cost);

  program["constraints"] = serialize.GenerateJsonObject(constraints);
  for (size_t i = 0; i < constraints.size(); i++) {
    program["constraints"][to_string(i)]["variables"] =
        ConvertToJson(MakeVariableList(constraints.at(i).get()));
  }
  program["num_constraints"] =
      ConvertToJson(static_cast<int>(constraints.size()));

  program["num_variables"] = ConvertToJson(4);
  ConstraintManager c(4);
  DeserializeConeProgram(program, &c);
  EXPECT_EQ(c.cone_inequalities().size(),
            constraints.size() - 1 /* minus one equality constraint*/);
  EXPECT_EQ(c.equality_constraints().size(), 1U /* one equality constraint*/);

  JsonObject program_serialized = SerializeConeProgram(c);
  ConstraintManager c_deserialized(4);
  DeserializeConeProgram(program_serialized, &c_deserialized);

  ConstraintManager c_deserialized_from_string(4);
  std::string prog_stringify = ConvertToJsonString(program_serialized);
  DeserializeConeProgram(ParseJsonString(prog_stringify),
                         &c_deserialized_from_string);
  EXPECT_NEAR(
      (c_deserialized_from_string.GetLinearCostVector() - linear_cost).norm(),
      0, 1e-9);
  Program prog(std::move(c_deserialized_from_string));
  Eigen::VectorXd y(4);
  Solve(prog, SolverConfiguration(), y.data());
  Program prog_ref(std::move(c));
  Eigen::VectorXd y_ref(4);
  Solve(prog, SolverConfiguration(), y_ref.data());
  EXPECT_NEAR((y - y_ref).norm(), 0, 1e-15);
}

}  // namespace conex
