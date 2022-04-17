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


template<typename T>
T MakeMatrixConstraint() {
  Eigen::MatrixXd A(3, 5);
  Eigen::VectorXd C(3);
  for (int i = 0; i < 3; i++) {
    A.row(i).setLinSpaced(A.cols(), -1, 1);
  }
  C.setLinSpaced(A.rows(), -1, 1);
  return T(A, C);
}

template<typename T>
void CompareMatrixConstraint(const ConstraintBase* x_ptr, 
                             const ConstraintBase* y_ptr) {
  const auto& x = *dynamic_cast<const T*>(x_ptr);
  const auto& y = *dynamic_cast<const T*>(y_ptr);
  EXPECT_EQ((x.constraint_matrix() - y.constraint_matrix()).norm(), 0);
  EXPECT_EQ((x.affine_term() - y.affine_term()).norm(), 0);
}

GTEST_TEST(Serialize, TestVirtualInterfaces) {
  std::vector<std::unique_ptr<ConstraintBase>> constraints;
  constraints.emplace_back(new LinearConstraint(std::move(MakeMatrixConstraint<LinearConstraint>())));
  constraints.emplace_back(new SOCConstraint(std::move(MakeMatrixConstraint<SOCConstraint>())));

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
  CompareMatrixConstraint<LinearConstraint>(constraints_deserialize.at(i).get(), constraints.at(i).get());
  i++;
  CompareMatrixConstraint<SOCConstraint>(constraints_deserialize.at(i).get(), constraints.at(i).get());
}

}  // namespace conex
