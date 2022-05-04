#include "conex/serialize.h"

#include <iostream>
#include <map>
#include <tuple>

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
T MakeMatrixConstraint(int value) {
  Eigen::MatrixXd A(3, 5);
  Eigen::VectorXd C(3);
  for (int i = 0; i < 3; i++) {
    A.row(i).setLinSpaced(A.cols(), -value, value);
  }
  C.setLinSpaced(A.rows(), -value, value);
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
  C << 1, 1, 0,
       1, 1, 1,
       0, 1, 2;
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

GTEST_TEST(Serialize, TestSerializeDeserialize) {
  std::vector<std::unique_ptr<ConstraintBase>> constraints;
  constraints.emplace_back(new LinearConstraint(
      std::move(MakeMatrixConstraint<LinearConstraint>(1))));
  constraints.emplace_back(
      new SOCConstraint(std::move(MakeMatrixConstraint<SOCConstraint>(2))));

  constraints.emplace_back(new EqualityConstraints(
      std::move(MakeMatrixConstraint<EqualityConstraints>(3))));

  constraints.emplace_back(
      new DenseLMIConstraint(std::move(MakeLMIConstraint(3))));

  JsonObject program;
  Serializer serialize;
  program["constraints"] = serialize.GenerateJsonObject(constraints);

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

}  // namespace conex
