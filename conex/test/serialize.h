#pragma once
#include "conex/test/json_parser.h"
#include "conex/test/test_constraint.h"
#include "conex/test/visitor.h"

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

  void visit(const Data&) override;
  void visit(const DataTwo&) override;
  void visit(const Constraint&) override;
  void visit(const ConstraintTwo&) override;

 private:
  JsonObject json_;
};

template<typename T>
T fromJson(const JsonObject& data);

template <typename T>
JsonObject toJson(const T& object);

}  // namespace conex
