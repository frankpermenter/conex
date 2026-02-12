#include "conex/serialize.h"

#include <fstream>

#include "conex/debug_macros.h"
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
  value["data"]["order"] = ConvertToJson(data.GetRank());
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

JsonObject SerializeConeProgram(const ConstraintManager& constraint_manager) {
  Serializer serialize;
  JsonObject program;
  program["num_variables"] = ConvertToJson(
      static_cast<int>(constraint_manager.GetNumberOfVariables()));
  std::vector<const ConstraintBase*> constraint_serializer;
  vector<std::vector<int>> constraint_variables;

  for (auto& v : constraint_manager.cone_inequalities()) {
    constraint_serializer.push_back(v);
    constraint_variables.push_back(v->variables());
  }
  int i = 0;
  for (auto& v : constraint_manager.equality_constraints().data) {
    constraint_serializer.push_back(&v);
    constraint_variables.push_back(
        constraint_manager.equality_constraints().variables.at(i));
    i++;
  }

  program["num_constraints"] =
      ConvertToJson(static_cast<int>(constraint_serializer.size()));
  program["constraints"] = serialize.GenerateJsonObject(constraint_serializer);

  i = 0;
  for (auto& v : constraint_variables) {
    program["constraints"][to_string(i)]["variables"] = ConvertToJson(v);
    i++;
  }

  i = 0;
  program["num_quadratic_costs"] = ConvertToJson(
      static_cast<int>(constraint_manager.quadratic_costs().size()));

  for (auto& v : constraint_manager.quadratic_costs()) {
    program["quadratic_costs"][to_string(i)]["variables"] =
        ConvertToJson(v.variables());
    program["quadratic_costs"][to_string(i)]["cost_matrix"] =
        ConvertToJson(v.CostMatrix());
    i++;
  }
  program["linear_cost"] =
      ConvertToJson(constraint_manager.GetLinearCostVector());
  return program;
}

namespace {

struct ConstraintCounter {
  int linear_constraints = 0;
  int soc_constraints = 0;
  int equality_constraints = 0;
  int dense_lmi_constraints = 0;
};

void AddConstraintFromJSON(const JsonObject& value, ConstraintManager* c,
                           ConstraintCounter* stats) {
  IDs constraint_type = static_cast<IDs>(stoi(value["id"].value()));
  std::vector<int> variables =
      ConstructObjectFromJson<std::vector<int>>(value["variables"]);
  switch (constraint_type) {
    case IDs::LinearConstraint: {
      c->AddConstraint(fromJson<LinearConstraint>(value["data"]), variables);
      stats->linear_constraints++;
      break;
    }
    case IDs::SOCConstraint: {
      c->AddConstraint(fromJson<SOCConstraint>(value["data"]), variables);
      stats->soc_constraints++;
      break;
    }
    case IDs::EqualityConstraints: {
      c->AddEqualityConstraint(fromJson<EqualityConstraints>(value["data"]),
                               variables);
      stats->equality_constraints++;
      break;
    }
    case IDs::DenseLMIConstraint: {
      c->AddConstraint(fromJson<DenseLMIConstraint>(value["data"]), variables);
      stats->dense_lmi_constraints++;
      break;
    }
    default:
      throw std::runtime_error("Failed to parse JSON");
  }
}
void PrintSummary(const JsonObject& json, const ConstraintCounter& count) {
  std::cout << "Importing cone program from JSON: " << std::endl;
  ;
  size_t num_constraints = stoi(json["num_constraints"].value());
  size_t num_variables = stoi(json["num_variables"].value());
  std::cout << " Num Variables: " << num_variables << std::endl;
  ;
  std::cout << " Num Constraints: " << num_constraints << std::endl;
  std::cout << "  Linear Constraints: " << count.linear_constraints
            << std::endl;
  std::cout << "  SOC Constraints: " << count.soc_constraints << std::endl;
  std::cout << "  LMI Constraints: " << count.dense_lmi_constraints
            << std::endl;
  std::cout << "  Equality Constraints: " << count.equality_constraints
            << std::endl;
}
}  // namespace

void DeserializeConeProgram(const JsonObject& json, ConstraintManager* c) {
  Serializer serialize;
  const auto& all_constraints = json["constraints"];
  size_t num_constraints = stoi(json["num_constraints"].value());
  size_t num_variables = stoi(json["num_variables"].value());
  c->SetNumberOfVariables(num_variables);
  ConstraintCounter constraint_count;
  for (size_t i = 0; i < num_constraints; ++i) {
    AddConstraintFromJSON(all_constraints[to_string(i)], c, &constraint_count);
  }

  size_t num_quadratic_costs = stoi(json["num_quadratic_costs"].value());
  for (size_t i = 0; i < num_quadratic_costs; ++i) {
    const auto& data = json["quadratic_costs"][to_string(i)];
    std::vector<int> variables =
        ConstructObjectFromJson<std::vector<int>>(data["variables"]);
    c->AddQuadraticCost(ConstructObjectFromJson<MatrixXd>(data["cost_matrix"]),
                        variables);
  }
  c->AddLinearCost(ConstructObjectFromJson<MatrixXd>(json["linear_cost"]));
  PrintSummary(json, constraint_count);
}

void SaveConeProgram(const ConstraintManager& c, const std::string& filename) {
  std::ofstream myfile;
  myfile.open(filename);
  myfile << ConvertToJsonString(SerializeConeProgram(c));
  myfile.close();
}

}  // namespace conex
