#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <variant>
#include <vector>

namespace conex {

// Handle to a constraint registered with a Problem.
using ConstraintId = int;

// Problem: a container for optimization data (costs, constraints).
// No solver details, no tree structure — just data + variable indices.
//
//   Problem p;
//   auto c1 = p.AddLinearConstraint(A, b, vars);
//   auto c2 = p.AddQuadraticCost(Q, vars);
//   auto solver = MakeSolver(p, config);
//
class Problem {
 public:
  // Add a linear constraint: min ||Ax - b||^2 on the given variables.
  // Returns a handle for SetWeights / ComputeResidual.
  ConstraintId AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      const std::vector<int>& vars) {
    int id = static_cast<int>(constraints_.size());
    constraints_.push_back(
        LinearConstraintData{A, b, vars});
    return id;
  }

  // Dense A variant.
  ConstraintId AddLinearConstraint(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const std::vector<int>& vars) {
    Eigen::SparseMatrix<double> As = A.sparseView();
    return AddLinearConstraint(As, b, vars);
  }

  // Add a quadratic cost: x'Qx on the given variables.
  ConstraintId AddQuadraticCost(
      const Eigen::SparseMatrix<double>& Q,
      const std::vector<int>& vars) {
    int id = static_cast<int>(constraints_.size());
    constraints_.push_back(
        QuadraticCostData{Q, Eigen::MatrixXd(), vars});
    return id;
  }

  // Dense Q variant.
  ConstraintId AddQuadraticCost(
      const Eigen::MatrixXd& Q,
      const std::vector<int>& vars) {
    Eigen::SparseMatrix<double> Qs = Q.sparseView();
    return AddQuadraticCost(Qs, Q, vars);
  }

  // Add an equality constraint: Cx = d.
  ConstraintId AddEqualityConstraint(
      const Eigen::SparseMatrix<double>& C,
      const Eigen::VectorXd& d,
      const std::vector<int>& primal_vars,
      const std::vector<int>& dual_vars) {
    int id = static_cast<int>(constraints_.size());
    constraints_.push_back(
        EqualityConstraintData{C, d, primal_vars, dual_vars});
    return id;
  }

  int num_constraints() const {
    return static_cast<int>(constraints_.size());
  }

  // Constraint data types.
  struct LinearConstraintData {
    Eigen::SparseMatrix<double> A;
    Eigen::VectorXd b;
    std::vector<int> vars;
  };

  struct QuadraticCostData {
    Eigen::SparseMatrix<double> Q_sparse;
    Eigen::MatrixXd Q_dense;  // kept for dense path
    std::vector<int> vars;
  };

  struct EqualityConstraintData {
    Eigen::SparseMatrix<double> C;
    Eigen::VectorXd d;
    std::vector<int> primal_vars;
    std::vector<int> dual_vars;
  };

  using ConstraintData = std::variant<
      LinearConstraintData, QuadraticCostData, EqualityConstraintData>;

  const ConstraintData& constraint(ConstraintId id) const {
    return constraints_.at(id);
  }

  // Compute the number of variables from all constraints.
  int num_variables() const {
    int n = 0;
    for (const auto& c : constraints_) {
      std::visit([&](const auto& data) {
        if constexpr (std::is_same_v<std::decay_t<decltype(data)>,
                                     EqualityConstraintData>) {
          for (int v : data.primal_vars) n = std::max(n, v + 1);
          for (int v : data.dual_vars) n = std::max(n, v + 1);
        } else {
          for (int v : data.vars) n = std::max(n, v + 1);
        }
      }, c);
    }
    return n;
  }

 private:
  std::vector<ConstraintData> constraints_;

  // Dense+sparse Q helper.
  ConstraintId AddQuadraticCost(
      const Eigen::SparseMatrix<double>& Q_sparse,
      const Eigen::MatrixXd& Q_dense,
      const std::vector<int>& vars) {
    int id = static_cast<int>(constraints_.size());
    constraints_.push_back(QuadraticCostData{Q_sparse, Q_dense, vars});
    return id;
  }
};

}  // namespace conex
