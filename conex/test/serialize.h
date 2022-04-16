#pragma once
#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
#include "conex/error_checking_macros.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

using std::stod;
using std::stoi;
using std::string;
using std::to_string;
using std::vector;

namespace conex {

// sequence for
template <typename T, T... S, typename F>
constexpr void for_sequence(std::integer_sequence<T, S...>, F&& f) {
  using unpack_t = int[];
  (void)unpack_t{(static_cast<void>(f(std::integral_constant<T, S>{})), 0)...,
                 0};
}

struct Value {
 private:
  struct ValueData {
    std::map<std::string, Value> children;
    std::string string = "";
   friend Value;
  };
 public:

  bool is_scalar() const { return data.children.size() == 0; }
  bool is_struct() const { return data.string.length() == 0; }
  bool is_empty() const { return data.children.size() == 0 && data.string.length() == 0; }


  Value& operator[](std::string name) { return data.children[std::move(name)]; }

  const Value& operator[](std::string name) const {
    auto it = data.children.find(std::move(name));
    if (it != data.children.end()) {
      return it->second;
    }
    throw;
  }
  
  std::map<std::string, Value>& children() { 
    CONEX_ASSERT(is_struct(), "Object is scalar.");
    return data.children; 
  }

  const std::map<std::string, Value>& children() const {
    CONEX_ASSERT(is_struct(), "Object is scalar.");
    return data.children; 
  }

  std::string& value() { 
    CONEX_ASSERT(is_scalar(), "Object is struct.");
    return data.string; 
  }
  const std::string& value() const { 
    CONEX_ASSERT(is_scalar(), "Object is struct.");
    return data.string; 
  }
 private:
  ValueData data;
};

Value ConvertToJson(const std::string& value);
Value ConvertToJson(int value);
Value ConvertToJson(double value);
Value ConvertToJson(const std::vector<int>& v);
Value ConvertToJson(const Eigen::MatrixXd& value);
Value ConvertToJson(const vector<Eigen::MatrixXd>& value);

template <typename T>
T StringToType(const std::string&);

template <typename T>
std::vector<T> CommaSeparatedStringToVector(const std::string& input) {
  std::stringstream ss(input);
  std::vector<T> result;
  while (ss.good()) {
    string substr;
    getline(ss, substr, ',');
    result.push_back(StringToType<T>(substr));
  }
  return result;
}

template <typename T>
T ConstructObjectFromJson(const Value&);


template <typename Class, typename T>
struct PropertyImpl {
  constexpr PropertyImpl(T Class::*aMember, const char* aName)
      : member{aMember}, name{aName} {}

  using Type = T;

  T Class::*member;
  const char* name;
};

// One could overload this function to accept both a getter and a setter instead
// of a member.
template <typename Class, typename T>
constexpr auto property(T Class::*member, const char* name) {
  return PropertyImpl<Class, T>{member, name};
}

// unserialize function
template <typename T>
T fromJson(const Value& data) {
  T object;

  // We first get the number of properties
  constexpr auto nbProperties = std::tuple_size<decltype(T::properties)>::value;

  // We iterate on the index sequence of size `nbProperties`
  for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
    // get the property
    constexpr auto property = std::get<i>(T::properties);

    // get the type of the property
    using Type = typename decltype(property)::Type;

    // set the value to the member
    object.*(property.member) =
        ConstructObjectFromJson<Type>(data[property.name]);
  });

  return object;
}

template <typename T>
std::string ObjectName();

template <typename T>
Value toJson(const T& object) {
  Value data;
  // We first get the number of properties
  constexpr auto nbProperties = std::tuple_size<decltype(T::properties)>::value;

  // We iterate on the index sequence of size `nbProperties`
  for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
    // get the property
    constexpr auto property = std::get<i>(T::properties);

    // set the value to the member
    data[property.name] = ConvertToJson(object.*(property.member));
  });

  return data;
}

std::string ConvertToJsonString(const Value& val);
Value ParseJsonString(const std::string& json);

}  // namespace conex
