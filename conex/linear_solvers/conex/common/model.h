#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <numeric>
#include <variant>
#include <vector>

#include "conex/common/error_checking_macros.h"

namespace conex {
namespace EuclideanJordanAlgebra { class SymmetricConeOperations; }

// Handle to a constraint registered with a Model.
using ConstraintId = int;

// Constraint sense for linear inequalities.
enum class Sense { GE, LE };

// Model: a container for optimization data (costs, constraints).
//
// Internally, all linear constraints are stored in canonical form
// Ax + b >= 0.  The Sense and double-sided overloads handle sign
// flips automatically.
//
//   Model p;
//   p.AddLinearConstraint(A, b, Sense::GE);        // Ax + b >= 0
//   p.AddLinearConstraint(A, b, Sense::LE);         // Ax + b <= 0
//   p.AddLinearConstraint(A, b_lb, b_ub);           // b_lb <= Ax <= b_ub
//   p.SetLinearCost(c);                             // min c^T x
//
class Model {
 public:
  // Core: add Ax + b >= 0 (canonical form, stored directly).
  ConstraintId AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      const std::vector<int>& vars) {
    CONEX_DEMAND(static_cast<size_t>(A.cols()) == vars.size(),
                 "AddLinearConstraint: A.cols() must equal vars.size().");
    CONEX_DEMAND(A.rows() == b.size(),
                 "AddLinearConstraint: A.rows() must equal b.size().");
    int id = static_cast<int>(constraints_.size());
    constraints_.push_back(LinearConstraintData{A, b, vars});
    return id;
  }

  // With sense: internal form is Ax + b >= 0.
  //   (A, b, GE) means Ax >= b → Ax - b >= 0 → store (A, -b).
  //   (A, b, LE) means Ax <= b → -Ax + b >= 0 → store (-A, b).
  ConstraintId AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      Sense sense,
      const std::vector<int>& vars) {
    if (sense == Sense::GE) {
      Eigen::VectorXd neg_b = -b;
      return AddLinearConstraint(A, neg_b, vars);
    } else {
      Eigen::SparseMatrix<double> negA = -A;
      return AddLinearConstraint(negA, b, vars);
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
  void AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b_lb,
      const Eigen::VectorXd& b_ub,
      const std::vector<int>& vars) {
    AddLinearConstraint(A, b_lb, Sense::GE, vars);    // Ax >= b_lb
    AddLinearConstraint(A, b_ub, Sense::LE, vars);     // Ax <= b_ub
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

  // Add PSD constraint: Σ A_i x_i + B ≽ 0.
  // A_list[i] and B are n×n sparse matrices.
  // If use_chordal is true (default), the assembler exploits block-diagonal
  // sparsity via chordal decomposition.  If false, a single n×n PSD
  // constraint is created (useful for testing or dense matrices).
  void AddPSDConstraint(
      const std::vector<Eigen::SparseMatrix<double>>& A_list,
      const Eigen::SparseMatrix<double>& B,
      const std::vector<int>& vars,
      bool use_chordal = true) {
    CONEX_DEMAND(A_list.size() == vars.size(),
                 "AddPSDConstraint: A_list.size() must equal vars.size().");
    constraints_.push_back(PSDConstraintData{A_list, B, vars, use_chordal});
  }

  // Add SOC constraint: ||A₁x + b₁|| ≤ A₀x + b₀.
  // A is (1+m)×p where row 0 is A₀ and rows 1..m are A₁.
  // b is (1+m) where b(0) is b₀ and b(1..m) is b₁.
  ConstraintId AddSOCConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      const std::vector<int>& vars) {
    CONEX_DEMAND(static_cast<size_t>(A.cols()) == vars.size(),
                 "AddSOCConstraint: A.cols() must equal vars.size().");
    CONEX_DEMAND(A.rows() == b.size(),
                 "AddSOCConstraint: A.rows() must equal b.size().");
    int id = static_cast<int>(constraints_.size());
    constraints_.push_back(SOCConstraintData{A, b, vars});
    return id;
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

  // Convenience: vars = {0, 1, ..., Q.cols()-1}.
  ConstraintId AddQuadraticCost(const Eigen::SparseMatrix<double>& Q) {
    std::vector<int> vars(Q.cols());
    std::iota(vars.begin(), vars.end(), 0);
    return AddQuadraticCost(Q, vars);
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

  // Add a barrier cone constraint: Ax + b ∈ K where K has a general
  // log-homogeneous barrier.  The ops pointer must implement the z-space
  // SymmetricConeOperations methods and outlive the Model.
  ConstraintId AddBarrierConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      const std::vector<int>& vars,
      const EuclideanJordanAlgebra::SymmetricConeOperations* ops) {
    CONEX_DEMAND(static_cast<size_t>(A.cols()) == vars.size(),
                 "AddBarrierConstraint: A.cols() must equal vars.size().");
    CONEX_DEMAND(A.rows() == b.size(),
                 "AddBarrierConstraint: A.rows() must equal b.size().");
    int id = static_cast<int>(constraints_.size());
    constraints_.push_back(BarrierConstraintData{A, b, vars, ops});
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

  struct PSDConstraintData {
    std::vector<Eigen::SparseMatrix<double>> A_list;  // A_i, each n×n
    Eigen::SparseMatrix<double> B;                     // n×n
    std::vector<int> vars;
    bool use_chordal = true;
  };

  struct QuadraticCostData {
    Eigen::SparseMatrix<double> Q_sparse;
    Eigen::MatrixXd Q_dense;  // kept for dense path
    std::vector<int> vars;
  };

  struct SOCConstraintData {
    Eigen::SparseMatrix<double> A;  // (1+m) × p: [A₀; A₁]
    Eigen::VectorXd b;              // (1+m): [b₀; b₁]
    std::vector<int> vars;
  };

  struct EqualityConstraintData {
    Eigen::SparseMatrix<double> C;
    Eigen::VectorXd d;
    std::vector<int> primal_vars;
  };

  struct BarrierConstraintData {
    Eigen::SparseMatrix<double> A;
    Eigen::VectorXd b;
    std::vector<int> vars;
    const EuclideanJordanAlgebra::SymmetricConeOperations* ops;
  };

  using ConstraintData = std::variant<
      LinearConstraintData, PSDConstraintData, SOCConstraintData,
      QuadraticCostData, EqualityConstraintData, BarrierConstraintData>;

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
        using T = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T, EqualityConstraintData>) {
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

// Vectorize PSD constraint data: returns (n² × p) sparse A and n²-length b.
inline void VectorizePSD(
    const std::vector<Eigen::SparseMatrix<double>>& A_list,
    const Eigen::SparseMatrix<double>& B,
    Eigen::SparseMatrix<double>* A_vec,
    Eigen::VectorXd* b_vec) {
  const int n = B.rows();
  const int n2 = n * n;
  const int p = static_cast<int>(A_list.size());
  std::vector<Eigen::Triplet<double>> trips;
  for (int k = 0; k < p; ++k) {
    const auto& Ak = A_list[k];
    for (int outer = 0; outer < Ak.outerSize(); ++outer)
      for (Eigen::SparseMatrix<double>::InnerIterator it(Ak, outer); it; ++it)
        trips.emplace_back(it.col() * n + it.row(), k, it.value());
  }
  A_vec->resize(n2, p);
  A_vec->setFromTriplets(trips.begin(), trips.end());
  b_vec->resize(n2);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i)
      (*b_vec)(j * n + i) = B.coeff(i, j);
}

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

// Drop structurally rank-deficient columns from linear constraints.
// Returns (reduced_problem, expansion).
std::pair<Model, Expansion> RemoveStructuralRankDeficiency(
    const Model& problem);

// Legacy alias.
inline std::pair<Model, Expansion> Preprocess(const Model& problem) {
  return RemoveStructuralRankDeficiency(problem);
}

// Per-constraint row scale factors applied by RowScaleModel.
struct RowScaling {
  // row_scale[constraint_id] holds per-row scale factors for that constraint.
  // The original row i was divided by row_scale[id](i).
  // To recover original duals: lambda_original_i = lambda_scaled_i / scale_i.
  std::vector<Eigen::VectorXd> row_scale;
};

// Scale each row of each linear/SOC constraint so that b_i ≈ 1.
// For row i: scale = max(|b_i|, ||A_i||) (falls back to row norm when b≈0).
// Returns (scaled_model, scaling).
std::pair<Model, RowScaling> RowScaleModel(const Model& model);

}  // namespace conex
