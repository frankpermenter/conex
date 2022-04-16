#include "conex/test/serialize.h"
#include "conex/test/test_constraint.h"
#include "conex/test/json_parser.h"

namespace conex {
void Serializer::visit(const Data& data) {
  JsonObject value;
  value["data"] = toJson(data);
  value["id"].value() = to_string(DataOneID);
  int i = json_.as_map().size();
  json_[ to_string(i)] = value;
}

void Serializer::visit(const DataTwo& data) {
  JsonObject value;
  value["data"] = toJson(data);
  value["id"].value() = to_string(DataTwoID);
  int i = json_.as_map().size();
  json_[ to_string(i)] = value;
}
}
