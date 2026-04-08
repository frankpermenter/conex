#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <numeric>
#include <variant>
#include <vector>

namespace conex {

// Handle to a constraint registered with a Problem.
using ConstraintId = int;

// Constraint sense for linear inequalities.
enum class Sense { GE, LE };

// Problem: a container for optimization data (costs, constraints).
//
// Internally, all linear constraints are stored in canonical form
// Ax + b >= 0.  The Sense and double-sided overloads handle sign
// flips automatically.
//
//   Problem p;
//   p.AddLinearConstraint(A, b, Sense::GE);        // Ax + b >= 0
//   p.AddLinearConstraint(A, b, Sense::LE);         // Ax + b <= 0
//   p.AddLinearConstraint(A, b_lb, b_ub);           // b_lb <= Ax <= b_ub
//   p.SetLinearCost(c);                             // min c^T x
//
class Problem {
 public:
  // Core: add Ax + b >= 0 (canonical form, stored directly).
  ConstraintId AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      const std::vector<int>& vars) {
    int id = static_cast<int>(constraints_.size());
    constraints_.push_back(
        LinearConstraintData{A, b, vars});
    return id;
  }

  // With sense: GE stores as-is, LE negates A and b.
  ConstraintId AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      Sense sense,
      const std::vector<int>& vars) {
    if (sense == Sense::GE) {
      return AddLinearConstraint(A, b, vars);
    } else {
      Eigen::SparseMatrix<double> negA = -A;
      Eigen::VectorXd neg_b = -b;
      return AddLinearConstraint(negA, neg_b, vars);
    }
  }

  // With sense, default vars.
  ConstraintId AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      Sense sense) {
    std::vector<int> vars(A.cols());
    std::iota(vars.begin(), vars.end(), 0);
    return AddLinearConstraint(A, b, sense, vars);
  }

  // Double-sided: b_lb <= Ax <= b_ub.
  // Stored as two constraints: Ax - b_lb >= 0 and -Ax + b_ub >= 0.
  void AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b_lb,
      const Eigen::VectorXd& b_ub,
      const std::vector<int>& vars) {
    AddLinearConstraint(A, -b_lb, Sense::GE, vars);   // Ax - b_lb >= 0
    AddLinearConstraint(A, b_ub, Sense::LE, vars);     // Ax - b_ub <= 0
  }

  // Double-sided, default vars.
  void AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b_lb,
      const Eigen::VectorXd& b_ub) {
    std::vector<int> vars(A.cols());
    std::iota(vars.begin(), vars.end(), 0);
    AddLinearConstraint(A, b_lb, b_ub, vars);
  }

  // Convenience: vars = {0, 1, ..., A.cols()-1}.
  ConstraintId AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b) {
    std::vector<int> vars(A.cols());
    std::iota(vars.begin(), vars.end(), 0);
    return AddLinearConstraint(A, b, vars);
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
  // Dual variables are allocated by the Solver, not by the user.
  ConstraintId AddEqualityConstraint(
      const Eigen::SparseMatrix<double>& C,
      const Eigen::VectorXd& d,
      const std::vector<int>& primal_vars) {
    int id = static_cast<int>(constraints_.size());
    constraints_.push_back(
        EqualityConstraintData{C, d, primal_vars});
    return id;
  }

  // Set the linear cost: min c^T x.
  void SetLinearCost(const Eigen::VectorXd& c) { linear_cost_ = c; }

  // Access the linear cost (empty if not set).
  const Eigen::VectorXd& linear_cost() const { return linear_cost_; }
  bool has_linear_cost() const { return linear_cost_.size() > 0; }

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
  };

  using ConstraintData = std::variant<
      LinearConstraintData, QuadraticCostData, EqualityConstraintData>;

  const ConstraintData& constraint(ConstraintId id) const {
    return constraints_.at(id);
  }

  const std::vector<ConstraintData>& constraints() const {
    return constraints_;
  }

  // Compute the number of variables from all constraints.
  int num_variables() const {
    int n = 0;
    for (const auto& c : constraints_) {
      std::visit([&](const auto& data) {
        if constexpr (std::is_same_v<std::decay_t<decltype(data)>,
                                     EqualityConstraintData>) {
          for (int v : data.primal_vars) n = std::max(n, v + 1);
        } else {
          for (int v : data.vars) n = std::max(n, v + 1);
        }
      }, c);
    }
    return n;
  }

 private:
  Eigen::VectorXd linear_cost_;
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

// Maps a reduced solution back to the original variable space.
struct Expansion {
  std::vector<int> col_map;  // reduced_col → original_col
  int original_n;

  Eigen::VectorXd Expand(const Eigen::VectorXd& x_reduced) const {
    Eigen::VectorXd x(original_n);
    x.setZero();
    for (int i = 0; i < static_cast<int>(col_map.size()); ++i)
      x(col_map[i]) = x_reduced(i);
    return x;
  }

  Eigen::VectorXd Reduce(const Eigen::VectorXd& x_full) const {
    Eigen::VectorXd x(col_map.size());
    for (int i = 0; i < static_cast<int>(col_map.size()); ++i)
      x(i) = x_full(col_map[i]);
    return x;
  }

  bool was_reduced() const {
    return static_cast<int>(col_map.size()) < original_n;
  }
};

// Preprocess: drop structurally rank-deficient columns from linear
// constraints.  Returns (reduced_problem, expansion).
std::pair<Problem, Expansion> Preprocess(const Problem& problem);

}  // namespace conex
