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
    std::map<std::string, Value> members;
    std::string value = "";
   friend Value;
  };
 public:

  bool is_scalar() const { return data_.members.size() == 0; }
  bool is_struct() const { return data_.value.length() == 0; }
  bool is_empty() const { return data_.members.size() == 0 && data_.value.length() == 0; }

  Value& operator[](std::string name) { return data_.members[std::move(name)]; }

  const Value& operator[](std::string name) const {
    auto it = data_.members.find(std::move(name));
    if (it != data_.members.end()) {
      return it->second;
    }
    throw;
  }
  
  std::map<std::string, Value>& members() { 
    CONEX_ASSERT(is_struct(), "Object is scalar.");
    return data_.members; 
  }

  const std::map<std::string, Value>& members() const {
    CONEX_ASSERT(is_struct(), "Object is scalar.");
    return data_.members; 
  }

  std::string& value() { 
    CONEX_ASSERT(is_scalar(), "Object is struct.");
    return data_.value; 
  }
  const std::string& value() const { 
    CONEX_ASSERT(is_scalar(), "Object is struct.");
    return data_.value; 
  }
 private:
  ValueData data_;
};

Value ConvertToJson(const std::string& value);
Value ConvertToJson(int value);
Value ConvertToJson(double value);
Value ConvertToJson(const std::vector<int>& v);
Value ConvertToJson(const Eigen::MatrixXd& value);
Value ConvertToJson(const vector<Eigen::MatrixXd>& value);


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
Value toJson(const T& object) {
  Value data;
  // We first get the number of properties
  constexpr auto kNumProperties = std::tuple_size<decltype(T::properties)>::value;

  // Convert each property to a JSON string and store in struct.
  for_sequence(std::make_index_sequence<kNumProperties>{}, [&](auto i) {
    constexpr auto property = std::get<i>(T::properties);
    data[property.name] = ConvertToJson(object.*(property.member));
  });

  return data;
}

std::string ConvertToJsonString(const Value& val);
Value ParseJsonString(const std::string& json);

}  // namespace conex
