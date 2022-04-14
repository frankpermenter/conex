#include "conex/test/serialize.h"
#include <stack>
#include <string>

using std::vector;
namespace conex {

std::string MakeJsonString(const Json::Value& val) {
  std::string output = val.data.string;
  if (val.data.children.size() > 0) {
    output += "{ ";
    for (auto& v : val.data.children) {
      output =
          output + "  \"" + v.first + "\" : " + MakeJsonString(v.second) + " ,";
    }
    output.pop_back(); /* remove last "," */
    output += "} ";
  } else {
    output = "\"" + output + "\"";
  }
  return output;
}

namespace Json {

std::string MatrixToInitializerString(const Eigen::MatrixXd& value) {
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                               ", ", ", ", "", "", "", "");
  std::stringstream buffer;
  buffer << value.format(CommaInitFmt);
  return buffer.str();
}

std::string MatrixToJsonLeaf(const Eigen::MatrixXd& value) {
  return to_string(value.rows()) + "," + to_string(value.cols()) + "," +
         MatrixToInitializerString(value);
}

Value MatrixToJson(const Eigen::MatrixXd& value) {
  Value v;
  v.data.children["cols"] = to_string(value.cols());
  v.data.children["rows"] = to_string(value.rows());
  v.data.children["data"] = MatrixToInitializerString(value);
  return v;
}

template <>
int ConstructFromString<int>(const Value& value) {
  return stoi(value.data.string);
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
std::string ConstructFromString<std::string>(const Value& value) {
  return value.data.string;
}

template <>
std::vector<int> ConstructFromString<std::vector<int>>(const Value& value) {
  return CommaSeparatedStringToVector<int>(value.data.string);
}

template <>
Eigen::MatrixXd ConstructFromString<Eigen::MatrixXd>(const Value& value) {
  std::string data = value.data.children.at("data").data.string;
  int rows = stoi(value.data.children.at("rows").data.string);
  int cols = stoi(value.data.children.at("cols").data.string);
  std::vector<double> matrix_data = CommaSeparatedStringToVector<double>(data);
  if (rows * cols != static_cast<int>(matrix_data.size())) {
    throw std::runtime_error("Invalid data.");
  }
  return Eigen::Map<const Eigen::MatrixXd>(matrix_data.data(), rows, cols);
}

template <>
vector<Eigen::MatrixXd> ConstructFromString<vector<Eigen::MatrixXd>>(
    const Value& value) {
  vector<Eigen::MatrixXd> y;
  for (const auto& v : value.data.children) {
    y.push_back(ConstructFromString<Eigen::MatrixXd>(v.second));
  }
  return y;
}

template <>
Eigen::VectorXd ConstructFromString<Eigen::VectorXd>(const Value& value) {
  return ConstructFromString<Eigen::MatrixXd>(value);
}

}  // namespace Json

bool ReadNextToken(const std::string& string, size_t start, size_t* token_start,
                   size_t* end) {
  *token_start = string.find("\"", start);
  *end = string.find("\"", *token_start + 1);
  return *end != string::npos && *token_start != string::npos;
}

Json::Value MakeValue(const std::string& json) {
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
  Json::Value root;
  std::map<int, Json::Value*> parent_nodes;
  parent_nodes[-1] = &root;
  for (size_t token_id = 0; token_id < tokens.size(); token_id++) {
    auto current_node = parent_nodes.at(token_to_parent.at(token_id));
    if (!current_node) {
      throw std::runtime_error("bad");
    }
    if (has_multiple_children.at(token_to_parent.at(token_id))) {
      current_node->data.children[tokens.at(token_id)];
      // Attach node to parent using token name, but
      // node pointer to global table using unique token-id.
      parent_nodes[token_id] =
          &current_node->data.children[tokens.at(token_id)];
    } else {
      current_node->data.string = tokens.at(token_id);
    }
  }
  return root;
}

}  // namespace conex
