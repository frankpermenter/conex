#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

#include "json_parser.h"
#include "serialize.h"
#include "conex/linear_constraint.h"
#include "conex/soc_constraint.h"

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
    case IDs::LinearConstraint: {
      LinearConstraint constraint = fromJson<conex::LinearConstraint>(value);
      return std::make_unique<LinearConstraint>(std::move(constraint));
    }
    case IDs::SOCConstraint: {
      SOCConstraint constraint = fromJson<conex::SOCConstraint>(value);
      return std::make_unique<SOCConstraint>(std::move(constraint));
    }
  }

}


LinearConstraint MakeLinearConstraint() {
  Eigen::MatrixXd A(3, 5);
  Eigen::VectorXd C(3);
  for (int i = 0; i < 3; i++) {
    A.row(i).setLinSpaced(A.cols(), -1, 1);
  }
  C.setLinSpaced(A.rows(), -1, 1);
  return LinearConstraint(A, C);
}

SOCConstraint MakeSOCConstraint() {
  Eigen::MatrixXd A(3, 5);
  Eigen::VectorXd C(3);
  for (int i = 0; i < 3; i++) {
    A.row(i).setLinSpaced(A.cols(), -1, 1);
  }
  C.setLinSpaced(A.rows(), -1, 1);
  return SOCConstraint(A, C);
}


void CompareLinearConstraint(const ConstraintBase* x_ptr, 
                             const ConstraintBase* y_ptr) {
  const auto& x = *dynamic_cast<const LinearConstraint*>(x_ptr);
  const auto& y = *dynamic_cast<const LinearConstraint*>(y_ptr);
  EXPECT_EQ((x.constraint_matrix() - y.constraint_matrix()).norm(), 0);
  EXPECT_EQ((x.affine_term() - y.affine_term()).norm(), 0);
}

void CompareSOCConstraint(const ConstraintBase* x_ptr, 
                             const ConstraintBase* y_ptr) {
  const auto& x = *dynamic_cast<const SOCConstraint*>(x_ptr);
  const auto& y = *dynamic_cast<const SOCConstraint*>(y_ptr);
  EXPECT_EQ((x.constraint_matrix() - y.constraint_matrix()).norm(), 0);
  EXPECT_EQ((x.affine_term() - y.affine_term()).norm(), 0);
}


GTEST_TEST(Serialize, TestVirtualInterfaces) {
  std::vector<std::unique_ptr<ConstraintBase>> constraints;
  constraints.emplace_back(new LinearConstraint(std::move(MakeLinearConstraint())));
  constraints.emplace_back(new SOCConstraint(std::move(MakeSOCConstraint())));

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

  int i = 0;
  CompareLinearConstraint(constraints_deserialize.at(i).get(), constraints.at(i).get());
  i++;
  CompareSOCConstraint(constraints_deserialize.at(i).get(), constraints.at(i).get());
}

}  // namespace conex
