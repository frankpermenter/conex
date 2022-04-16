#pragma once

#include <iostream>
#include <map>
#include <tuple>

#include <Eigen/Dense>

#include "conex/debug_macros.h"
#include "conex/error_checking_macros.h"
#include "gtest/gtest.h"

using std::stod; 
using std::stoi;
using std::string;
using std::to_string;
using std::vector;

namespace conex {

// Utility class for parsing/emiting JSON strings.  Used to 
// store list of key-value pairs.  Since values can be either strings
// or more key-value pairs, we allow this object to behave like
// either depending on context.
class JsonObject {
 public:
  bool is_string() const { return map_.size() == 0; }
  bool is_map() const { return value_.length() == 0; }
  bool is_empty() const { return map_.size() == 0 && value_.length() == 0; }

  JsonObject& operator[](std::string name) { return map_[std::move(name)]; }

  const JsonObject& operator[](std::string name) const {
    auto it = map_.find(std::move(name));
    if (it != map_.end()) {
      return it->second;
    }
    throw;
  }
  
  std::map<std::string, JsonObject>& as_map() { 
    CONEX_ASSERT(is_map(), "Object is string.");
    return map_; 
  }

  const std::map<std::string, JsonObject>& as_map() const {
    CONEX_ASSERT(is_map(), "Object is string.");
    return map_; 
  }

  std::string& value() { 
    CONEX_ASSERT(is_string(), "Object is map.");
    return value_; 
  }
  const std::string& value() const { 
    CONEX_ASSERT(is_string(), "Object is map.");
    return value_; 
  }
 private:
   std::map<std::string, JsonObject> map_;
   std::string value_ = "";
};

JsonObject ConvertToJson(const std::string& value);
JsonObject ConvertToJson(int value);
JsonObject ConvertToJson(double value);
JsonObject ConvertToJson(const std::vector<int>& v);
JsonObject ConvertToJson(const Eigen::MatrixXd& value);
JsonObject ConvertToJson(const vector<Eigen::MatrixXd>& value);


// sequence for
template <typename T, T... S, typename F>
constexpr void for_sequence(std::integer_sequence<T, S...>, F&& f) {
    (static_cast<void>(f(std::integral_constant<T, S>{})), ...);
}

template <typename T>
T ConstructObjectFromJson(const JsonObject&);

// unserialize function
template <typename T>
T fromJson(const JsonObject& data) {
  T object;
  constexpr auto properties = T::properties;

  // We expect T has a tuple called "properties" indicating which
  // members should be serialized.
  constexpr auto kNumProperties = std::tuple_size<decltype(properties)>::value;

  // Use factory function ConstructObjectFromJson<Type> to construct object
  // from type.
  for_sequence(std::make_index_sequence<kNumProperties>{}, [&](auto i) {
    constexpr auto property = std::get<i>(properties);
    using Type = typename decltype(property)::Type;
    object.*(property.member) = ConstructObjectFromJson<Type>(data[property.name]);
  });

  return object; 
}

template <typename T>
JsonObject toJson(const T& object) {
  JsonObject data;
  constexpr auto kNumProperties = std::tuple_size<decltype(T::properties)>::value;

  // Convert each property to a JSON string and store in struct.
  for_sequence(std::make_index_sequence<kNumProperties>{}, [&](auto i) {
    constexpr auto property = std::get<i>(T::properties);
    data[property.name] = ConvertToJson(object.*(property.member));
  });

  return data;
}

std::string ConvertToJsonString(const JsonObject& val);
JsonObject ParseJsonString(const std::string& json);

}  // namespace conex
