#pragma once
#include <memory>

#include "conex/constraint_interface.h"
#include "conex/constraint_manager.h"
#include "conex/json_parser.h"

namespace conex {

class Serializer : Visitor {
 public:
  JsonObject GenerateJsonObject(
      const std::vector<std::unique_ptr<ConstraintBase>>& constraints) {
    json_ = JsonObject();
    for (auto& c : constraints) {
      // If constraint c accepts, then it will
      // call Visitor::visit().
      c->accept(this);
    }
    return json_;
  }

  template <typename T>
  JsonObject GenerateJsonObject(const std::vector<T*>& constraints) {
    json_ = JsonObject();
    for (auto& c : constraints) {
      // If constraint c accepts, then it will
      // call Visitor::visit().
      c->accept(this);
    }
    return json_;
  }

  void visit(const LinearConstraint&) override;
  void visit(const SOCConstraint&) override;
  void visit(const EqualityConstraints&) override;
  void visit(const DenseLMIConstraint&) override;

 private:
  JsonObject json_;
};

template <typename T>
T fromJson(const JsonObject& data);

template <typename T>
JsonObject toJson(const T& object);

std::unique_ptr<ConstraintBase> MakeConstraintFromJSON(const JsonObject& value);

void DeserializeConeProgram(const JsonObject& json, ConstraintManager* c);

JsonObject SerializeConeProgram(const ConstraintManager& constraint_manager);

}  // namespace conex
