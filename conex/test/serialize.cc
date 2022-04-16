#include "conex/test/serialize.h"
#include <stack>
#include <string>

using std::vector;
namespace conex {

Value ConvertToJson(const std::string& value) {
  Value y;
  y.value() = value;
  return y;
}

Value ConvertToJson(int value) {
  Value y;
  y.value() = std::to_string(value);
  return y;
}

Value ConvertToJson(double value) {
  Value y;
  y.value() = std::to_string(value);
  return y;
}

Value ConvertToJson(const std::vector<int>& v) {
  Value y;
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

Value ConvertToJson(const Eigen::MatrixXd& value) {
  return MatrixToJson(value);
}

Value ConvertToJson(const vector<Eigen::MatrixXd>& value) {
  int i = 0;
  Value constraint_matrices;
  for (auto& v : value) {
    constraint_matrices[to_string(i)] = MatrixToJson(v);
    i++;
  }
  return constraint_matrices;
}





std::string ConvertToJsonString(const Value& val) {
  if (!val.is_scalar()) {
    std::string output = "{ ";
    for (auto& v : val.children()) {
      output += "  \"" + v.first + "\" : " + ConvertToJsonString(v.second) + " ,";
    }
    output.pop_back(); /* remove last "," */
    output += "} ";
    return output;
  } else {
    return "\"" + val.value() + "\"";
  }
}

namespace {
std::string MatrixToInitializerString(const Eigen::MatrixXd& value) {
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                               ", ", ", ", "", "", "", "");
  std::stringstream buffer;
  buffer << value.format(CommaInitFmt);
  return buffer.str();
}
}

Value MatrixToJson(const Eigen::MatrixXd& value) {
  Value v;
  v.children()["cols"] = ConvertToJson(to_string(value.cols()));
  v.children()["rows"] = ConvertToJson(to_string(value.rows()));
  v.children()["data"] = ConvertToJson(MatrixToInitializerString(value));
  return v;
}

template <>
int ConstructObjectFromJson<int>(const Value& value) {
  return stoi(value.value());
}

template <>
double StringToType<double>(const std::string& input) {
  return stod(input);
}

template <>
int StringToType<int>(const std::string& input) {
  return stoi(input);
}

template <>
std::string ConstructObjectFromJson<std::string>(const Value& value) {
  return value.value();
}

template <>
std::vector<int> ConstructObjectFromJson<std::vector<int>>(const Value& value) {
  return CommaSeparatedStringToVector<int>(value.value());
}

template <>
Eigen::MatrixXd ConstructObjectFromJson<Eigen::MatrixXd>(const Value& value) {
  std::string data = value["data"].value();
  int rows = stoi(value["rows"].value());
  int cols = stoi(value["cols"].value());
  std::vector<double> matrix_data = CommaSeparatedStringToVector<double>(data);
  if (rows * cols != static_cast<int>(matrix_data.size())) {
    throw std::runtime_error("Invalid data.");
  }
  return Eigen::Map<const Eigen::MatrixXd>(matrix_data.data(), rows, cols);
}

template <>
vector<Eigen::MatrixXd> ConstructObjectFromJson<vector<Eigen::MatrixXd>>(
    const Value& value) {
  vector<Eigen::MatrixXd> y;
  for (const auto& v : value.children()) {
    y.push_back(ConstructObjectFromJson<Eigen::MatrixXd>(v.second));
  }
  return y;
}

template <>
Eigen::VectorXd ConstructObjectFromJson<Eigen::VectorXd>(const Value& value) {
  return ConstructObjectFromJson<Eigen::MatrixXd>(value);
}

bool ReadNextToken(const std::string& string, size_t start, size_t* token_start,
                   size_t* end) {
  *token_start = string.find("\"", start);
  *end = string.find("\"", *token_start + 1);
  return *end != string::npos && *token_start != string::npos;
}

Value ParseJsonString(const std::string& json) {
  size_t token_end = string::npos;
  size_t token_start = 0;
  size_t next_search_start = 0;
  std::vector<string> tokens;
  std::vector<int> token_to_parent;
  std::stack<int> parent;
  std::map<int, bool> has_multiple_children;
  has_multiple_children[-1] = true;

  // Build a tree of tokens, i.e., quote delimited strings in input.
  // The following patterns indicate parent child relationships:
  //
  // parent_token : child_token,
  // parent_token : { child_token, child_token, ..., child_token  }
  while (ReadNextToken(json, next_search_start, &token_start, &token_end)) {
    tokens.push_back(json.substr(token_start + 1, token_end - token_start - 1));
    next_search_start = token_end + 1;
    if (parent.size() == 0) {
      token_to_parent.push_back(-1);
    } else {
      token_to_parent.push_back(parent.top());
    }

    size_t current_position = token_end + 1;
    while (1) {
      if (json[current_position] == '\"' || current_position >= json.length()) {
        break;
      }
      switch (json[current_position]) {
        case ' ':
          break;
        // Indicates multiple children
        case '{':
          has_multiple_children[parent.top()] = true;
          break;
        //  a : { b, c, d }
        case '}':
        //  a : b, or
        case ',':
          if (parent.size() > 0) {
            parent.pop();
          } else {
            // We have reached end of string.
          }
          break;
        // (parent : children)
        case ':':
          parent.push(tokens.size() - 1);
          has_multiple_children[parent.top()] = false;
      }
      current_position++;
    }
  }

  // Encode tree using a linked-list.
  Value root;
  std::map<int, Value*> parent_nodes;
  parent_nodes[-1] = &root;
  for (size_t token_id = 0; token_id < tokens.size(); token_id++) {
    auto current_node = parent_nodes.at(token_to_parent.at(token_id));
    if (!current_node) {
      throw std::runtime_error("bad");
    }
    if (has_multiple_children.at(token_to_parent.at(token_id))) {
      current_node->children()[tokens.at(token_id)];
      // Attach node to parent using token name, but
      // node pointer to global table using unique token-id.
      parent_nodes[token_id] =
          &current_node->children()[tokens.at(token_id)];
    } else {
      current_node->value() = tokens.at(token_id);
    }
  }
  return root;
}

}  // namespace conex
