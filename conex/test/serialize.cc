#include "conex/test/serialize.h"
#include "conex/equality_constraint.h"
#include "conex/linear_constraint.h"
#include "conex/psd_constraint.h"
#include "conex/quadratic_cone_constraint.h"
#include "conex/soc_constraint.h"
#include "conex/test/json_parser.h"

using Eigen::MatrixXd;

namespace conex {
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
  value["data"]["upper_bound"] = ConvertToJson(data.constraint_matrix());
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
  value["data"]["affine_term"] = ConvertToJson(data.constraint_matrix());
  value["id"].value() = enum_to_string(IDs::SOCConstraint);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}

template <>
EqualityConstraints fromJson<EqualityConstraints>(const JsonObject& data) {
  auto matrix = ConstructObjectFromJson<MatrixXd>(data["constraint_matrix"]);
  auto affine = ConstructObjectFromJson<MatrixXd>(data["upper_bound"]);
  return EqualityConstraints(matrix, affine);
}

std::unique_ptr<ConstraintBase> MakeConstraintFromJSON(
    const JsonObject& value) {
  IDs constraint_type = static_cast<IDs>(stoi(value["id"].value()));
  switch (constraint_type) {
    case IDs::LinearConstraint: {
      LinearConstraint constraint =
          fromJson<conex::LinearConstraint>(value["data"]);
      return std::make_unique<LinearConstraint>(std::move(constraint));
    }
    case IDs::SOCConstraint: {
      SOCConstraint constraint = fromJson<conex::SOCConstraint>(value["data"]);
      return std::make_unique<SOCConstraint>(std::move(constraint));
    }
  }
  throw std::runtime_error("Failed to parse JSON");
}
}  // namespace conex
