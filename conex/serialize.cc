#include "conex/serialize.h"
#include "conex/dense_lmi_constraint.h"
#include "conex/equality_constraint.h"
#include "conex/json_parser.h"
#include "conex/linear_constraint.h"
#include "conex/quadratic_cone_constraint.h"
#include "conex/soc_constraint.h"

using Eigen::MatrixXd;

namespace conex {

enum class IDs : int {
  LinearConstraint = 0,
  SOCConstraint = 1,
  EqualityConstraints = 2,
  DenseLMIConstraint = 3,
};
namespace {
std::string enum_to_string(IDs e) { return to_string(static_cast<int>(e)); }
}  // namespace

// Linear Constraint //////////////////////////////////////////////
void Serializer::visit(const LinearConstraint& data) {
  JsonObject value;
  value["data"]["constraint_matrix"] = ConvertToJson(data.constraint_matrix());
  value["data"]["upper_bound"] = ConvertToJson(data.affine_term());
  value["id"].value() = enum_to_string(IDs::LinearConstraint);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}

template <>
LinearConstraint fromJson<LinearConstraint>(const JsonObject& data) {
  auto matrix = ConstructObjectFromJson<MatrixXd>(data["constraint_matrix"]);
  auto affine = ConstructObjectFromJson<MatrixXd>(data["upper_bound"]);
  return LinearConstraint(matrix, affine);
}

// SOC Constraint //////////////////////////////////////////////
void Serializer::visit(const SOCConstraint& data) {
  JsonObject value;
  value["data"]["constraint_matrix"] = ConvertToJson(data.constraint_matrix());
  value["data"]["upper_bound"] = ConvertToJson(data.affine_term());
  value["id"].value() = enum_to_string(IDs::SOCConstraint);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}

template <>
SOCConstraint fromJson<SOCConstraint>(const JsonObject& data) {
  auto matrix = ConstructObjectFromJson<MatrixXd>(data["constraint_matrix"]);
  auto affine = ConstructObjectFromJson<MatrixXd>(data["upper_bound"]);
  return SOCConstraint(matrix, affine);
}

// Equality Constraint //////////////////////////////////////////////
void Serializer::visit(const EqualityConstraints& data) {
  JsonObject value;
  value["data"]["constraint_matrix"] = ConvertToJson(data.constraint_matrix());
  value["data"]["affine_term"] = ConvertToJson(data.affine_term());
  value["id"].value() = enum_to_string(IDs::EqualityConstraints);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}

template <>
EqualityConstraints fromJson<EqualityConstraints>(const JsonObject& data) {
  auto matrix = ConstructObjectFromJson<MatrixXd>(data["constraint_matrix"]);
  auto affine = ConstructObjectFromJson<MatrixXd>(data["affine_term"]);
  return EqualityConstraints(matrix, affine);
}

// LMIT //////////////////////////////////////////////
void Serializer::visit(const DenseLMIConstraint& data) {
  JsonObject value;
  value["data"]["constraint_matrix"] =
      ConvertToJson(data.constraint_matrices());
  value["data"]["affine_term"] = ConvertToJson(data.affine_term());
  value["data"]["order"] = ConvertToJson(Rank(data));
  value["id"].value() = enum_to_string(IDs::DenseLMIConstraint);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}
DenseLMIConstraint fromJsonDenseLMIConstraint(const JsonObject& data) {
  auto matrix =
      ConstructObjectFromJson<vector<MatrixXd>>(data["constraint_matrix"]);
  auto affine = ConstructObjectFromJson<MatrixXd>(data["affine_term"]);
  int order = ConstructObjectFromJson<int>(data["order"]);
  return DenseLMIConstraint(order, matrix, affine);
}

template <>
DenseLMIConstraint fromJson<DenseLMIConstraint>(const JsonObject& data) {
  auto matrix =
      ConstructObjectFromJson<vector<MatrixXd>>(data["constraint_matrix"]);
  auto affine = ConstructObjectFromJson<MatrixXd>(data["affine_term"]);
  int order = ConstructObjectFromJson<int>(data["order"]);
  return DenseLMIConstraint(order, matrix, affine);
}

template <typename T>
std::unique_ptr<ConstraintBase> MakeConstraint(const JsonObject& value) {
  T constraint = fromJson<T>(value["data"]);
  return std::make_unique<T>(std::move(constraint));
}

std::unique_ptr<ConstraintBase> MakeConstraintFromJSON(
    const JsonObject& value) {
  IDs constraint_type = static_cast<IDs>(stoi(value["id"].value()));
  switch (constraint_type) {
    case IDs::LinearConstraint: {
      return MakeConstraint<LinearConstraint>(value);
    }
    case IDs::SOCConstraint: {
      return MakeConstraint<SOCConstraint>(value);
    }
    case IDs::EqualityConstraints: {
      return MakeConstraint<EqualityConstraints>(value);
    }
    case IDs::DenseLMIConstraint: {
      return MakeConstraint<DenseLMIConstraint>(value);
    }
  }
  throw std::runtime_error("Failed to parse JSON");
}
}  // namespace conex
