#include "conex/json_parser.h"

#include <stack>
#include <string>

using std::vector;
namespace conex {

namespace {

template <typename T>
std::string ObjectName();

template <typename T>
T StringToType(const std::string&);

template <>
double StringToType<double>(const std::string& input) {
  return stod(input);
}

template <>
int StringToType<int>(const std::string& input) {
  return stoi(input);
}

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

std::string MatrixToInitializerString(const Eigen::MatrixXd& value) {
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                               ", ", ", ", "", "", "", "");
  std::stringstream buffer;
  buffer << value.format(CommaInitFmt);
  return buffer.str();
}

JsonObject MatrixToJson(const Eigen::MatrixXd& value) {
  JsonObject v;
  v["cols"] = ConvertToJson(to_string(value.cols()));
  v["rows"] = ConvertToJson(to_string(value.rows()));
  v["data"] = ConvertToJson(MatrixToInitializerString(value));
  return v;
}
}  // namespace

JsonObject ConvertToJson(const std::string& value) {
  JsonObject y;
  y.value() = value;
  return y;
}

JsonObject ConvertToJson(int value) {
  JsonObject y;
  y.value() = std::to_string(value);
  return y;
}

JsonObject ConvertToJson(double value) {
  JsonObject y;
  y.value() = std::to_string(value);
  return y;
}

JsonObject ConvertToJson(const std::vector<int>& v) {
  JsonObject y;
  std::stringstream buffer;
  if (v.size() > 0) {
    buffer << v.at(0);
    for (auto i = v.begin() + 1; i != v.end(); ++i) {
      buffer << "," << *i;
    }
  }
  y.value() = buffer.str();
  return y;
}

JsonObject ConvertToJson(const Eigen::MatrixXd& value) {
  return MatrixToJson(value);
}

JsonObject ConvertToJson(const vector<Eigen::MatrixXd>& value) {
  int i = 0;
  JsonObject constraint_matrices;
  for (auto& v : value) {
    constraint_matrices[to_string(i)] = MatrixToJson(v);
    i++;
  }
  return constraint_matrices;
}

std::string ConvertToJsonString(const JsonObject& val) {
  if (val.is_map()) {
    std::string output = "{ ";
    for (auto& v : val.as_map()) {
      output +=
          "  \"" + v.first + "\" : " + ConvertToJsonString(v.second) + " ,";
    }
    output.pop_back(); /* remove last "," */
    output += "} ";
    return output;
  } else {
    return "\"" + val.value() + "\"";
  }
}

template <>
int ConstructObjectFromJson<int>(const JsonObject& value) {
  return stoi(value.value());
}

template <>
std::string ConstructObjectFromJson<std::string>(const JsonObject& value) {
  return value.value();
}

template <>
std::vector<int> ConstructObjectFromJson<std::vector<int>>(
    const JsonObject& value) {
  return CommaSeparatedStringToVector<int>(value.value());
}

template <>
Eigen::MatrixXd ConstructObjectFromJson<Eigen::MatrixXd>(
    const JsonObject& value) {
  std::string data = value["data"].value();
  int rows = stoi(value["rows"].value());
  int cols = stoi(value["cols"].value());
  std::vector<double> matrix_data = CommaSeparatedStringToVector<double>(data);
  if (rows * cols != static_cast<int>(matrix_data.size())) {
    throw std::runtime_error("Invalid data.");
  }
  bool convert_row_to_column_major = true;
  if (convert_row_to_column_major) {
    return Eigen::Map<const Eigen::MatrixXd>(matrix_data.data(), cols, rows)
        .transpose();
  } else {
    return Eigen::Map<const Eigen::MatrixXd>(matrix_data.data(), rows, cols);
  }
}

template <>
vector<Eigen::MatrixXd> ConstructObjectFromJson<vector<Eigen::MatrixXd>>(
    const JsonObject& value) {
  vector<Eigen::MatrixXd> y;
  for (const auto& v : value.as_map()) {
    y.push_back(ConstructObjectFromJson<Eigen::MatrixXd>(v.second));
  }
  return y;
}

template <>
Eigen::VectorXd ConstructObjectFromJson<Eigen::VectorXd>(
    const JsonObject& value) {
  return ConstructObjectFromJson<Eigen::MatrixXd>(value);
}

namespace {

// Find next quote delimited string.
bool FindNextToken(const std::string& string, size_t start, size_t* token_start,
                   size_t* end) {
  *token_start = string.find("\"", start);
  *end = string.find("\"", *token_start + 1);
  return *end != string::npos && *token_start != string::npos;
}
}  // namespace

JsonObject ParseJsonString(const std::string& json) {
  size_t token_end = string::npos;
  size_t token_start = 0;
  size_t next_search_start = 0;
  std::vector<string> tokens;
  std::vector<int> token_to_parent;
  std::stack<int> parent;
  std::map<int, bool> has_multiple_children;
  has_multiple_children[-1] = true;

  // Parse quote delimited substrings ("tokens") in input and arrange them in
  // a tree.  The following patterns indicate parent-child relationships:
  //
  // "parent" : "child",
  // "parent" : { "child" : "grand_child" , "child" : { "grand_child" :
  // "great_grand_child" }}
  //
  // To parse, we do depth-first search, pushing onto the stack when ":" is
  // encountered, and popping when "," or "}".
  while (FindNextToken(json, next_search_start, &token_start, &token_end)) {
    tokens.push_back(json.substr(token_start + 1, token_end - token_start - 1));
    if (parent.size() == 0) {
      token_to_parent.push_back(-1);
    } else {
      token_to_parent.push_back(parent.top());
    }

    // Read ahead in string to determine if current token
    // is the next parent or the last child of the current parent.
    size_t current_position = token_end + 1;
    while (1) {
      if (json[current_position] == '\"' || current_position >= json.length()) {
        next_search_start = current_position;
        break;
      }

      switch (json[current_position]) {
        case ' ':
          break;
        // Current token is the next parent.
        case ':':
          parent.push(tokens.size() - 1);
          has_multiple_children[parent.top()] = false; /* default assumption*/
          break;
        // Current token has multiple children.
        case '{':
          has_multiple_children[parent.top()] = true;
          break;
        // Current token is the last child.
        case '}':
        case ',':
          if (parent.size() > 0) {
            parent.pop();
          } else {
            // We have reached end of string.
          }
          break;
      }
      current_position++;
    }
  }

  // Record tree into a JsonObject.
  JsonObject json_root;
  std::map<int, JsonObject*> token_id_to_json_object;
  token_id_to_json_object[-1] = &json_root;
  for (size_t token_id = 0; token_id < tokens.size(); token_id++) {
    auto current_parent =
        token_id_to_json_object.at(token_to_parent.at(token_id));
    CONEX_ASSERT(current_parent, "Invalid JSON input.");
    if (has_multiple_children.at(token_to_parent.at(token_id))) {
      current_parent->as_map()[tokens.at(token_id)];
      // Attach node to parent using token name, but
      // node pointer to global table using unique token-id.
      token_id_to_json_object[token_id] =
          &current_parent->as_map()[tokens.at(token_id)];
    } else {
      current_parent->value() = tokens.at(token_id);
    }
  }
  return json_root;
}

}  // namespace conex
