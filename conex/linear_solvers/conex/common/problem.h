#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <set>
#include <unordered_map>
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

  // Return a new Problem with all linear constraints merged into one
  // and all quadratic costs merged into one.  Equality constraints
  // are copied as-is.
  Problem Consolidate() const {
    const int n = num_variables();
    Problem out;

    // Aggregate linear constraints: stack rows, map columns to global.
    std::vector<Eigen::Triplet<double>> A_trips;
    std::vector<double> b_vals;
    std::set<int> lin_vars_set;
    int total_rows = 0;
    for (const auto& c : constraints_) {
      if (auto* lc = std::get_if<LinearConstraintData>(&c)) {
        for (int k = 0; k < lc->A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(lc->A, k); it; ++it)
            A_trips.emplace_back(total_rows + it.row(),
                                  lc->vars[it.col()], it.value());
        for (int r = 0; r < static_cast<int>(lc->b.size()); ++r)
          b_vals.push_back(lc->b(r));
        total_rows += lc->A.rows();
        lin_vars_set.insert(lc->vars.begin(), lc->vars.end());
      }
    }
    if (total_rows > 0) {
      std::vector<int> lin_vars(lin_vars_set.begin(), lin_vars_set.end());
      int nv = static_cast<int>(lin_vars.size());
      std::unordered_map<int, int> g2l;
      for (int j = 0; j < nv; ++j) g2l[lin_vars[j]] = j;
      std::vector<Eigen::Triplet<double>> A_local;
      A_local.reserve(A_trips.size());
      for (const auto& t : A_trips)
        A_local.emplace_back(t.row(), g2l.at(t.col()), t.value());
      Eigen::SparseMatrix<double> A(total_rows, nv);
      A.setFromTriplets(A_local.begin(), A_local.end());
      Eigen::VectorXd b =
          Eigen::Map<const Eigen::VectorXd>(b_vals.data(), total_rows);
      out.AddLinearConstraint(A, b, lin_vars);
    }

    // Aggregate quadratic costs: sum into one Q.
    std::vector<Eigen::Triplet<double>> Q_trips;
    std::set<int> quad_vars_set;
    for (const auto& c : constraints_) {
      if (auto* qc = std::get_if<QuadraticCostData>(&c)) {
        for (int k = 0; k < qc->Q_sparse.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(qc->Q_sparse, k);
               it; ++it)
            Q_trips.emplace_back(qc->vars[it.row()], qc->vars[it.col()],
                                  it.value());
        quad_vars_set.insert(qc->vars.begin(), qc->vars.end());
      }
    }
    if (!Q_trips.empty()) {
      std::vector<int> quad_vars(quad_vars_set.begin(), quad_vars_set.end());
      int nv = static_cast<int>(quad_vars.size());
      std::unordered_map<int, int> g2l;
      for (int j = 0; j < nv; ++j) g2l[quad_vars[j]] = j;
      std::vector<Eigen::Triplet<double>> Q_local;
      Q_local.reserve(Q_trips.size());
      for (const auto& t : Q_trips)
        Q_local.emplace_back(g2l.at(t.row()), g2l.at(t.col()), t.value());
      Eigen::SparseMatrix<double> Q(nv, nv);
      Q.setFromTriplets(Q_local.begin(), Q_local.end());
      out.AddQuadraticCost(Q, quad_vars);
    }

    // Copy equality constraints.
    for (const auto& c : constraints_) {
      if (auto* ec = std::get_if<EqualityConstraintData>(&c))
        out.AddEqualityConstraint(ec->C, ec->d, ec->primal_vars);
    }

    return out;
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
