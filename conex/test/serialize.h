#pragma once
#include <iostream>
#include <map>
#include <tuple>
#include "conex/debug_macros.h"
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

// Sample implementation of a json-like data structure. It is only there for the
// example to compile and actually produce a testable output
namespace Json {
struct Value;


std::string jsonString(const std::string& field, const std::string& value);

std::string MatrixToJsonLeaf(const Eigen::MatrixXd& value);

Value MatrixToJson(const Eigen::MatrixXd& value);

struct Value {
 private:
  struct ValueData {
    std::map<std::string, Value> children;
    std::string string = "";
   friend Value;
  };
 public:

  Value& operator[](std::string name) { return data.children[std::move(name)]; }

  const Value& operator[](std::string name) const {
    auto it = data.children.find(std::move(name));

    if (it != data.children.end()) {
      return it->second;
    }
    throw;
  }

  Value& operator=(const std::string& value) {
    data.string = value;
    return *this;
  }

  Value& operator=(int value) {
    data.string = std::to_string(value);
    return *this;
  }

  Value& operator=(double value) {
    data.string = std::to_string(value);
    return *this;
  }

  Value& operator=(const std::vector<int>& v) {
    std::stringstream buffer;
    if (v.size() > 0) {
      buffer << v.at(0);
      for (auto i = v.begin() + 1; i != v.end(); ++i) {
        buffer << "," << *i;
      }
    }
    data.string = buffer.str();
    return *this;
  }

  Value& operator=(const Eigen::MatrixXd& value) {
    data = MatrixToJson(value).data;
    return *this;
  }

  Value& operator=(const vector<Eigen::MatrixXd>& value) {
    int i = 0;
    Value constraint_matrices;
    for (auto& v : value) {
      constraint_matrices.data.children[to_string(i)].data =
          MatrixToJson(v).data;
      i++;
    }
    data = constraint_matrices.data;
    return *this;
  }
  
  std::map<std::string, Value>& children() { return data.children; }
  const std::map<std::string, Value>& children() const { return data.children; }

  std::string& value() { return data.string; }
  const std::string& value() const { return data.string; }
 private:
  ValueData data;
};

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
T ConstructFromString(const Value&);

}  // namespace Json

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
T fromJson(const Json::Value& data) {
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
        Json::ConstructFromString<Type>(data[property.name]);
  });

  return object;
}

template <typename T>
std::string ObjectName();

template <typename T>
Json::Value toJson(const T& object) {
  Json::Value data;
  // We first get the number of properties
  constexpr auto nbProperties = std::tuple_size<decltype(T::properties)>::value;

  // We iterate on the index sequence of size `nbProperties`
  for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
    // get the property
    constexpr auto property = std::get<i>(T::properties);

    // set the value to the member
    data[property.name] = object.*(property.member);
  });

  return data;
}

std::string ConvertToJsonString(const Json::Value& val);
Json::Value ParseJsonString(const std::string& json);

}  // namespace conex
