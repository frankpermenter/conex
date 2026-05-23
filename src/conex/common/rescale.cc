#include "conex/common/rescale.h"

#include <cmath>
#include <numeric>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Sparse>

namespace conex {

namespace {

Eigen::SparseMatrix<double> ToSparse(const Eigen::MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> t;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i, j)) > 1e-15) t.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(t.begin(), t.end());
  return S;
}

// Row-scale a nonneg constraint: divide row i by b_i.
// Returns false if any b_i <= 0 (infeasible or degenerate).
bool RescaleNonneg(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b) {
  int m = A.rows();
  for (int i = 0; i < m; ++i) {
    if (b(i) <= 1e-15) return false;
  }
  // Scale: A_new(i,:) = A(i,:) / b(i), b_new = ones.
  Eigen::VectorXd inv_b = b.cwiseInverse();
  for (int k = 0; k < A.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      it.valueRef() *= inv_b(it.row());
    }
  }
  b.setOnes();
  return true;
}

// Row-scale a PSD constraint: B = LL^T → A_k = L^{-1} A_k L^{-T}, B = I.
bool RescalePSD(std::vector<Eigen::SparseMatrix<double>>& A_list,
                Eigen::SparseMatrix<double>& B) {
  int n = B.rows();
  Eigen::MatrixXd B_dense(B);
  B_dense = 0.5 * (B_dense + B_dense.transpose());
  Eigen::LLT<Eigen::MatrixXd> llt(B_dense);
  if (llt.info() != Eigen::Success) return false;

  Eigen::MatrixXd L = llt.matrixL();
  Eigen::MatrixXd Linv = L.inverse();

  for (auto& Ak : A_list) {
    Eigen::MatrixXd Ak_dense(Ak);
    Ak_dense = Linv * Ak_dense * Linv.transpose();
    Ak_dense = 0.5 * (Ak_dense + Ak_dense.transpose());
    Ak = ToSparse(Ak_dense);
  }

  B = ToSparse(Eigen::MatrixXd::Identity(n, n));
  return true;
}

// Row-scale a SOC constraint via automorphism: P(b^{-1/2}) maps b → e.
bool RescaleSOC(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b) {
  if (b.size() == 0 || b(0) <= 1e-15) return false;
  double b1_norm = b.tail(b.size() - 1).norm();
  if (b(0) <= b1_norm) return false;

  int dim = b.size();
  double det_b = b(0) * b(0) - b1_norm * b1_norm;
  if (det_b <= 1e-15) return false;

  double l1 = b(0) + b1_norm;
  double l2 = b(0) - b1_norm;
  double sl1 = std::sqrt(l1), sl2 = std::sqrt(l2);
  double isl1 = 1.0 / sl1, isl2 = 1.0 / sl2;

  Eigen::VectorXd b_invsqrt(dim);
  b_invsqrt(0) = 0.5 * (isl1 + isl2);
  if (b1_norm > 1e-15) {
    double x_scale = 0.5 * (isl1 - isl2) / b1_norm;
    b_invsqrt.tail(dim - 1) = x_scale * b.tail(dim - 1);
  } else {
    b_invsqrt.tail(dim - 1).setZero();
  }

  double det_binvsqrt = b_invsqrt(0) * b_invsqrt(0) -
                        b_invsqrt.tail(dim - 1).squaredNorm();

  auto applyP = [&](const Eigen::VectorXd& y) -> Eigen::VectorXd {
    double ip = 2.0 * (b_invsqrt(0) * y(0) +
                       b_invsqrt.tail(dim - 1).dot(y.tail(dim - 1)));
    Eigen::VectorXd Ry = y;
    Ry.tail(dim - 1) *= -1;
    return ip * b_invsqrt - det_binvsqrt * Ry;
  };

  Eigen::VectorXd b_new = applyP(b);
  Eigen::MatrixXd A_dense(A);
  for (int j = 0; j < A_dense.cols(); ++j) {
    A_dense.col(j) = applyP(A_dense.col(j));
  }

  A = ToSparse(A_dense);
  b = b_new;
  return true;
}

// ---------- Column norm computation helpers ----------

// Accumulate per-column norms from a sparse matrix with variable mapping.
// mode: 0 = max abs (l∞), 1 = sum of squares (for l2).
void AccumulateColumnNorms(const Eigen::SparseMatrix<double>& A,
                           const std::vector<int>& vars,
                           Eigen::VectorXd& col_norms, int mode) {
  for (int k = 0; k < A.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      int j = vars[it.col()];
      double v = std::abs(it.value());
      if (mode == 0)
        col_norms(j) = std::max(col_norms(j), v);
      else
        col_norms(j) += v * v;
    }
}

// Accumulate per-column norms from PSD A_list.
// Each A_list[k] maps to vars[k]; use Frobenius norm of A_k.
void AccumulatePSDColumnNorms(
    const std::vector<Eigen::SparseMatrix<double>>& A_list,
    const std::vector<int>& vars,
    Eigen::VectorXd& col_norms, int mode) {
  for (int k = 0; k < static_cast<int>(A_list.size()); ++k) {
    if (k >= static_cast<int>(vars.size())) break;
    double nrm = 0;
    for (int outer = 0; outer < A_list[k].outerSize(); ++outer)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A_list[k], outer);
           it; ++it) {
        double v = std::abs(it.value());
        if (mode == 0)
          nrm = std::max(nrm, v);
        else
          nrm += v * v;
      }
    if (mode == 0)
      col_norms(vars[k]) = std::max(col_norms(vars[k]), nrm);
    else
      col_norms(vars[k]) += nrm;
  }
}

// Compute per-column norms across all constraints.
// mode: 0 = l∞, 1 = l2 (returns squared norms, caller takes sqrt).
Eigen::VectorXd ComputeColumnNorms(const Model& problem, int n, int mode) {
  Eigen::VectorXd col_norms = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        AccumulateColumnNorms(data.A, data.vars, col_norms, mode);
      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        AccumulateColumnNorms(data.A, data.vars, col_norms, mode);
      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        AccumulatePSDColumnNorms(data.A_list, data.vars, col_norms, mode);
      }
    }, problem.constraint(i));
  }
  if (mode == 1) {
    for (int j = 0; j < n; ++j)
      col_norms(j) = std::sqrt(col_norms(j));
  }
  return col_norms;
}

// Compute per-row l∞ norms for each constraint (returned as vector of vectors).
// For PSD constraints, returns a single-element vector with the max over all
// A_list matrices (PSD rows aren't individually scalable without breaking
// the cone structure).
std::vector<Eigen::VectorXd> ComputeRowNorms(const Model& problem) {
  std::vector<Eigen::VectorXd> row_norms;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        int m = data.A.rows();
        Eigen::VectorXd rn = Eigen::VectorXd::Zero(m);
        for (int k = 0; k < data.A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
               it; ++it)
            rn(it.row()) = std::max(rn(it.row()), std::abs(it.value()));
        row_norms.push_back(std::move(rn));
      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        int m = data.A.rows();
        Eigen::VectorXd rn = Eigen::VectorXd::Zero(m);
        for (int k = 0; k < data.A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(data.A, k);
               it; ++it)
            rn(it.row()) = std::max(rn(it.row()), std::abs(it.value()));
        row_norms.push_back(std::move(rn));
      } else {
        // PSD, QuadraticCost, Equality: no per-row scaling in Ruiz.
        row_norms.push_back(Eigen::VectorXd());
      }
    }, problem.constraint(i));
  }
  return row_norms;
}

// Apply column scaling D to a problem: A_new[:,j] = A[:,j] * D[vars[j]].
// Also scales cost and quadratic terms appropriately.
Model ApplyColumnScaling(const Model& problem,
                           const Eigen::VectorXd& D) {
  Model scaled;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        Eigen::SparseMatrix<double> A = data.A;
        for (int k = 0; k < A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
            it.valueRef() *= D(data.vars[it.col()]);
        scaled.AddLinearConstraint(A, data.b, data.vars);

      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        auto A_list = data.A_list;
        for (int k = 0; k < static_cast<int>(A_list.size()); ++k) {
          if (k < static_cast<int>(data.vars.size()))
            A_list[k] *= D(data.vars[k]);
        }
        scaled.AddPSDConstraint(A_list, data.B, data.vars, data.use_chordal);

      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        Eigen::SparseMatrix<double> A = data.A;
        for (int k = 0; k < A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
            it.valueRef() *= D(data.vars[it.col()]);
        scaled.AddSOCConstraint(A, data.b, data.vars);

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        // x = D * x_new → x'Qx = x_new' (D*Q*D) x_new → Q_new = D*Q*D.
        Eigen::SparseMatrix<double> Q = data.Q_sparse;
        for (int k = 0; k < Q.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k); it; ++it) {
            double sr = D(data.vars[it.row()]);
            double sc = D(data.vars[it.col()]);
            it.valueRef() *= (sr * sc);
          }
        scaled.AddQuadraticCost(Q, data.vars);

      } else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        Eigen::SparseMatrix<double> C = data.C;
        for (int k = 0; k < C.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(C, k); it; ++it)
            it.valueRef() *= D(data.primal_vars[it.col()]);
        scaled.AddEqualityConstraint(C, data.d, data.primal_vars);
      }
    }, problem.constraint(i));
  }

  if (problem.has_linear_cost()) {
    // c_new = D * c (since x_new = D^{-1} x, c^T x = c^T D x_new).
    scaled.SetLinearCost(problem.linear_cost().cwiseProduct(D));
  }
  return scaled;
}

// Apply per-row scaling E_i to constraint i:
//   Nonneg/SOC: A(row,:) *= E_i(row), b(row) *= E_i(row).
// PSD constraints are not row-scaled (cone structure).
Model ApplyRowScaling(const Model& problem,
                        const std::vector<Eigen::VectorXd>& E) {
  Model scaled;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        if (E[i].size() > 0) {
          Eigen::SparseMatrix<double> A = data.A;
          Eigen::VectorXd b = data.b;
          for (int k = 0; k < A.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
              it.valueRef() *= E[i](it.row());
          b = b.cwiseProduct(E[i]);
          scaled.AddLinearConstraint(A, b, data.vars);
        } else {
          scaled.AddLinearConstraint(data.A, data.b, data.vars);
        }

      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        if (E[i].size() > 0) {
          Eigen::SparseMatrix<double> A = data.A;
          Eigen::VectorXd b = data.b;
          for (int k = 0; k < A.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
              it.valueRef() *= E[i](it.row());
          b = b.cwiseProduct(E[i]);
          scaled.AddSOCConstraint(A, b, data.vars);
        } else {
          scaled.AddSOCConstraint(data.A, data.b, data.vars);
        }

      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        scaled.AddPSDConstraint(data.A_list, data.B, data.vars,
                                data.use_chordal);
      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        scaled.AddQuadraticCost(data.Q_sparse, data.vars);
      } else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        scaled.AddEqualityConstraint(data.C, data.d, data.primal_vars);
      }
    }, problem.constraint(i));
  }
  if (problem.has_linear_cost()) {
    scaled.SetLinearCost(problem.linear_cost());
  }
  return scaled;
}

// Ruiz equilibration: iterate symmetric row/column scaling.
//
// Each iteration:
//   1. Compute row norms  E_i = diag(||A_i(row,:)||_∞)^{-1/2}
//   2. Scale rows:  A_i ← diag(E_i) A_i,  b_i ← diag(E_i) b_i
//   3. Compute column norms  D_j = ||A(:,j)||_∞^{-1/2}
//   4. Scale columns:  A(:,j) ← A(:,j) * D_j
//   5. Accumulate:  col_scale ← col_scale .* D,  row_scale ← row_scale .* E
//
// Converges when max(|log(norm)| ) < tol for all rows and columns.
// For nonneg constraints (after Phase 1 row scaling → b=1), the Ruiz
// row scaling also rescales b, so b may no longer be exactly 1.
// This is acceptable: the scaling improves Gram conditioning, which
// matters more than having b=1 exactly.
struct RuizResult {
  Model problem;
  Eigen::VectorXd col_scale;  // cumulative column scale
  std::vector<Eigen::VectorXd> row_scale;  // cumulative row scales
};

RuizResult RuizEquilibrate(const Model& problem, int n,
                           int max_iters = 10, double tol = 0.1) {
  Eigen::VectorXd cumul_D = Eigen::VectorXd::Ones(n);
  int nc = problem.num_constraints();
  std::vector<Eigen::VectorXd> cumul_E(nc);

  Model current = problem;  // copy — will be overwritten each iteration
  if (current.has_linear_cost()) {
    // Preserve cost through iterations (applied at the end).
  }

  for (int iter = 0; iter < max_iters; ++iter) {
    // Step 1: Row scaling (nonneg and SOC only).
    auto row_norms = ComputeRowNorms(current);
    std::vector<Eigen::VectorXd> E(nc);
    bool any_row_scale = false;
    for (int i = 0; i < nc; ++i) {
      if (row_norms[i].size() > 0) {
        E[i].resize(row_norms[i].size());
        for (int r = 0; r < row_norms[i].size(); ++r) {
          if (row_norms[i](r) > 1e-15)
            E[i](r) = 1.0 / std::sqrt(row_norms[i](r));
          else
            E[i](r) = 1.0;
        }
        // Check if any row needs scaling.
        for (int r = 0; r < E[i].size(); ++r)
          if (std::abs(E[i](r) - 1.0) > tol) any_row_scale = true;
        // Accumulate.
        if (cumul_E[i].size() == 0)
          cumul_E[i] = E[i];
        else
          cumul_E[i] = cumul_E[i].cwiseProduct(E[i]);
      }
    }

    if (any_row_scale)
      current = ApplyRowScaling(current, E);

    // Step 2: Column scaling.
    Eigen::VectorXd col_norms = ComputeColumnNorms(current, n, /*mode=*/0);
    Eigen::VectorXd D(n);
    bool any_col_scale = false;
    for (int j = 0; j < n; ++j) {
      if (col_norms(j) > 1e-15) {
        D(j) = 1.0 / std::sqrt(col_norms(j));
        if (std::abs(D(j) - 1.0) > tol) any_col_scale = true;
      } else {
        D(j) = 1.0;
      }
    }

    if (any_col_scale) {
      cumul_D = cumul_D.cwiseProduct(D);
      // Strip cost before column scaling (ApplyColumnScaling also scales cost).
      Eigen::VectorXd saved_cost;
      bool had_cost = current.has_linear_cost();
      if (had_cost) saved_cost = current.linear_cost();

      current = ApplyColumnScaling(current, D);

      // Restore unscaled cost — we'll scale it once at the end.
      if (had_cost) {
        // Undo the cost scaling that ApplyColumnScaling did.
        // It set c_new = D * c, but we want the original cost back.
        // We'll apply the full cumul_D to the original cost at the end.
        Eigen::VectorXd c_unscaled(n);
        for (int j = 0; j < n; ++j) {
          if (D(j) > 1e-15)
            c_unscaled(j) = current.linear_cost()(j) / D(j);
          else
            c_unscaled(j) = current.linear_cost()(j);
        }
        current.SetLinearCost(c_unscaled);
      }
    }

    // Check convergence.
    if (!any_row_scale && !any_col_scale) break;
  }

  // Apply cumulative column scaling to cost.
  if (current.has_linear_cost()) {
    current.SetLinearCost(current.linear_cost().cwiseProduct(cumul_D));
  }

  return {std::move(current), cumul_D, cumul_E};
}

}  // namespace

std::pair<Model, RescaleInfo> RescaleProblem(const Model& problem,
                                               ColumnScaling strategy) {
  int n = problem.num_variables();
  RescaleInfo info;
  info.original_n = n;
  info.col_scale = Eigen::VectorXd::Ones(n);

  Model rescaled;

  // Phase 1: Row scaling — transform b → identity for each constraint.
  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        Eigen::SparseMatrix<double> A = data.A;
        Eigen::VectorXd b = data.b;
        if (RescaleNonneg(A, b)) {
          rescaled.AddLinearConstraint(A, b, data.vars);
          info.was_rescaled = true;
        } else {
          rescaled.AddLinearConstraint(data.A, data.b, data.vars);
        }

      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        auto A_list = data.A_list;
        auto B = data.B;
        if (RescalePSD(A_list, B)) {
          rescaled.AddPSDConstraint(A_list, B, data.vars, data.use_chordal);
          info.was_rescaled = true;
        } else {
          rescaled.AddPSDConstraint(data.A_list, data.B, data.vars,
                                     data.use_chordal);
        }

      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        Eigen::SparseMatrix<double> A = data.A;
        Eigen::VectorXd b = data.b;
        if (RescaleSOC(A, b)) {
          rescaled.AddSOCConstraint(A, b, data.vars);
          info.was_rescaled = true;
        } else {
          rescaled.AddSOCConstraint(data.A, data.b, data.vars);
        }

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        rescaled.AddQuadraticCost(data.Q_sparse, data.vars);

      } else if constexpr (std::is_same_v<T, Model::EqualityConstraintData>) {
        rescaled.AddEqualityConstraint(data.C, data.d, data.primal_vars);
      }
    }, problem.constraint(i));
  }

  // Carry forward cost.
  if (problem.has_linear_cost())
    rescaled.SetLinearCost(problem.linear_cost());

  // Phase 2: Column scaling.
  if (strategy == ColumnScaling::Ruiz) {
    // Iterative Ruiz equilibration: alternates row and column scaling
    // using l∞-norm with square-root damping.
    auto result = RuizEquilibrate(rescaled, n);
    info.col_scale = result.col_scale;
    info.row_scale = std::move(result.row_scale);
    // Check if any scaling was applied.
    for (int j = 0; j < n; ++j) {
      if (std::abs(info.col_scale(j) - 1.0) > 0.01) {
        info.was_rescaled = true;
        return {std::move(result.problem), info};
      }
    }
    for (const auto& rs : info.row_scale) {
      for (int r = 0; r < rs.size(); ++r) {
        if (std::abs(rs(r) - 1.0) > 0.01) {
          info.was_rescaled = true;
          return {std::move(result.problem), info};
        }
      }
    }
    // No significant scaling — return as-is.
    info.col_scale.setOnes();
    info.row_scale.clear();
    return {std::move(rescaled), info};
  }

  // MaxAbsValue or L2Norm: one-shot column scaling.
  // D_j = 1/N_j normalizes column j.  col_scale stores N_j for Unscale.
  int mode = (strategy == ColumnScaling::L2Norm) ? 1 : 0;
  Eigen::VectorXd col_norms = ComputeColumnNorms(rescaled, n, mode);

  bool need_col_scale = false;
  Eigen::VectorXd D = Eigen::VectorXd::Ones(n);
  for (int j = 0; j < n; ++j) {
    if (col_norms(j) > 1e-15) {
      D(j) = 1.0 / col_norms(j);
      info.col_scale(j) = D(j);
      if (std::abs(col_norms(j) - 1.0) > 0.1) need_col_scale = true;
    }
  }

  if (need_col_scale) {
    Model col_scaled = ApplyColumnScaling(rescaled, D);
    info.was_rescaled = true;
    return {std::move(col_scaled), info};
  }

  info.col_scale.setOnes();
  return {std::move(rescaled), info};
}

}  // namespace conex
