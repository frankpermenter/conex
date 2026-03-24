#pragma once
#include <vector>

namespace conex {

class IVariableShape {
 public:
  virtual int number_of_variables() const = 0;
  const std::vector<int>& variable_indices() const { return variable_indices_; }
  void set_variable_indices(const std::vector<int>& indices) {
    variable_indices_ = indices;
  }
  virtual std::vector<std::vector<int>> get_cliques() const {
    return {variable_indices_};
  }
  virtual ~IVariableShape() = default;

 protected:
  std::vector<int> variable_indices_;
};
}  // namespace conex
