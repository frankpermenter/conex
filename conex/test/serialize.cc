#include "conex/test/serialize.h"
#include "conex/test/json_parser.h"
#include "conex/test/test_constraint.h"

namespace conex {
namespace {
template <typename Class, typename T>
struct Property {
  constexpr Property(T Class::*aMember, const char* aName)
      : member{aMember}, name{aName} {}

  using Type = T;

  T Class::*member;
  const char* name;
};



template <typename Class, typename T>
constexpr auto MakeProperty(T Class::*member, const char* name) {
  return Property<Class, T>{member, name};
}

#if 0
template<typename T, typename T2>
std::map<string, T2 T::*> ItemsToSerialize();

    ///return std::make_tuple(MakeProperty(&Data::order, "order"),
    ///                MakeProperty(&Data::matrix, "matrix"),
    ///                MakeProperty(&Data::matrices, "matrices"),
    ///                MakeProperty(&Data::affine_term, "affine_term"),
    ///                MakeProperty(&Data::variables, "variables"));
}

template<>
  constexpr auto ItemsToSerialize<DataBase>() {
     return std::make_tuple(MakeProperty(&DataTwo::order, "order"),
                      MakeProperty(&DataTwo::matrix, "matrix"),
                      MakeProperty(&DataTwo::variables, "variables"));
}


template <typename T>
JsonObject toJson(const T& object) {
  JsonObject data;

  constexpr auto properties = ItemsToSerialize<T>();
  constexpr auto kNumProperties = std::tuple_size<decltype(properties)>::value;
  // Convert each property to a JSON string and store in struct.
  for_sequence(std::make_index_sequence<kNumProperties>{}, [&](auto i) {
    constexpr auto property = std::get<i>(properties);
    data[property.name] = ConvertToJson(object.*(property.member));
  });
  return data;
}
#endif


constexpr auto GetTupleData() { 
    return std::make_tuple(MakeProperty(&Data::order, "order"),
                    MakeProperty(&Data::matrix, "matrix"),
                    MakeProperty(&Data::matrices, "matrices"),
                    MakeProperty(&Data::affine_term, "affine_term"),
                    MakeProperty(&Data::variables, "variables"));
}

template<typename T>
constexpr auto GetTupleData() { 
  if constexpr(std::is_same<T, DataTwo>::value) {
     return std::make_tuple(MakeProperty(&DataTwo::order, "order"),
                      MakeProperty(&DataTwo::matrix, "matrix"),
                      MakeProperty(&DataTwo::variables, "variables"));
  }
  if constexpr(std::is_same<T, Data>::value) {
    return std::make_tuple(MakeProperty(&Data::order, "order"),
                    MakeProperty(&Data::matrix, "matrix"),
                    MakeProperty(&Data::matrices, "matrices"),
                    MakeProperty(&Data::affine_term, "affine_term"),
                    MakeProperty(&Data::variables, "variables"));
  }
}



// unserialize function

} // namespace

template <typename T>
T fromJson(const JsonObject& data) {
  T object;
  constexpr auto properties = GetTupleData<T>(); 

  // We expect T has a tuple called "properties" indicating which
  // members should be serialized.
  constexpr auto kNumProperties = std::tuple_size<decltype(properties)>::value;

  // Use factory function ConstructObjectFromJson<Type> to construct object
  // from type.
  for_sequence(std::make_index_sequence<kNumProperties>{}, [&](auto i) {
    constexpr auto property = std::get<i>(properties);
    using Type = typename decltype(property)::Type;
    object.*(property.member) =
        ConstructObjectFromJson<Type>(data[property.name]);
  });

  return object;
}

template<typename T>
JsonObject toJson(const T& input) {
  JsonObject output;
  constexpr auto properties = GetTupleData<T>();
  constexpr auto kNumProperties = std::tuple_size<decltype(properties)>::value;
  // Convert each property to a JSON string and store in struct.
  for_sequence(std::make_index_sequence<kNumProperties>{}, [&](auto i) {
    constexpr auto property = std::get<i>(properties);
    output[property.name] = ConvertToJson(input.*(property.member));
  });
  return output;
}


template Data fromJson<Data>(const JsonObject& data);
template DataTwo fromJson<DataTwo>(const JsonObject& data);

template JsonObject toJson<Data>(const Data& input);
template JsonObject toJson<DataTwo>(const DataTwo& input);


void Serializer::visit(const Data& data) {
  JsonObject value;
  value["data"] = toJson(data);
  value["id"].value() = to_string(DataOneID);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}

void Serializer::visit(const DataTwo& data) {
  JsonObject value;
  value["data"] = toJson(data);
  value["id"].value() = to_string(DataTwoID);
  int i = json_.as_map().size();
  json_[to_string(i)] = value;
}

}  // namespace conex
