#pragma once
#include "conex/test/json_parser.h"
#include "conex/visitor.h"
#include "conex/constraint_interface.h"

namespace conex {

class Serializer : Visitor {
 public:
  JsonObject GenerateJsonObject(
      const std::vector<std::unique_ptr<ConstraintBase>>& constraints) {
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

 private:
  JsonObject json_;
};

template<typename T>
T fromJson(const JsonObject& data);

template <typename T>
JsonObject toJson(const T& object);


std::unique_ptr<ConstraintBase> MakeConstraintFromJSON(const JsonObject& value);
}  // namespace conex
