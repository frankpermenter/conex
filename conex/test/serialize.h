#pragma once
#include "conex/test/json_parser.h"
#include "conex/test/test_constraint.h"

namespace conex {
class Visitor;


class Serializer : Visitor {
 public:
  JsonObject GenerateJsonObject(const std::vector<std::unique_ptr<DataBase>>& constraints) {
    for (auto& c : constraints) {
      c->accept(this);
    }
    return json_;
  }

  void visit(const Data&) override;  
  void visit(const DataTwo&) override;
 private:
   JsonObject json_;
};


} // namespace
