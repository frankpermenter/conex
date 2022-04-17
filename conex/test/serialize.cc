#include "conex/test/serialize.h"
#include "conex/test/json_parser.h"
#include "conex/linear_constraint.h"
#include "conex/soc_constraint.h"
#include "conex/equality_constraint.h"
#include "conex/psd_constraint.h"
#include "conex/equality_constraint.h"
#include "conex/quadratic_cone_constraint.h"

using Eigen::MatrixXd;

namespace conex {
namespace {
std::string enum_to_string(IDs e) {
  return to_string(static_cast<int>(e));
}
}

// Linear Constraint //////////////////////////////////////////////
void Serializer::visit(const LinearConstraint& data) {
  JsonObject value;
  value["data"]["constraint_matrix"] = ConvertToJson(data.constraint_matrix());
  value["data"]["upper_bound"] = ConvertToJson(data.affine_term());
  value["id"].value() = enum_to_string(IDs::LinearConstraint);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}

template<> LinearConstraint fromJson<LinearConstraint>(const JsonObject& data) {
  auto matrix = ConstructObjectFromJson<MatrixXd>(data["constraint_matrix"]);
  auto affine = ConstructObjectFromJson<MatrixXd>(data["upper_bound"]);
  return LinearConstraint(matrix, affine);
}

// SOC Constraint //////////////////////////////////////////////
void Serializer::visit(const SOCConstraint& data) {
  JsonObject value;
  value["data"]["constraint_matrix"] = ConvertToJson(data.constraint_matrix());
  value["data"]["upper_bound"] = ConvertToJson(data.constraint_matrix());
  value["id"].value() = enum_to_string(IDs::SOCConstraint);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}

template<> SOCConstraint fromJson<SOCConstraint>(const JsonObject& data) {
  auto matrix = ConstructObjectFromJson<MatrixXd>(data["constraint_matrix"]);
  auto affine = ConstructObjectFromJson<MatrixXd>(data["upper_bound"]);
  return SOCConstraint(matrix, affine);
}

// Equality Constraint //////////////////////////////////////////////
void Serializer::visit(const EqualityConstraints& data) {
  JsonObject value;
  value["data"]["constraint_matrix"] = ConvertToJson(data.constraint_matrix());
  value["data"]["affine_term"] = ConvertToJson(data.constraint_matrix());
  value["id"].value() = enum_to_string(IDs::SOCConstraint);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}

template<> EqualityConstraints fromJson<EqualityConstraints>(const JsonObject& data) {
  auto matrix = ConstructObjectFromJson<MatrixXd>(data["constraint_matrix"]);
  auto affine = ConstructObjectFromJson<MatrixXd>(data["upper_bound"]);
  return EqualityConstraints(matrix, affine);
}




}  // namespace conex
