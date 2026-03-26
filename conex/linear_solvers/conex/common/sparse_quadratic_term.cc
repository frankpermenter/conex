#include "conex/common/sparse_quadratic_term.h"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <set>
#include <unordered_map>

#include "conex/common/clique_ordering.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/kkt_tree_solver.h"
#include "conex/tree_solver/kkt_solver_factory.h"

namespace conex {

SparseQuadraticTermAssembler::SparseQuadraticTermAssembler(
    const Eigen::SparseMatrix<double>& Q,
    const std::vector<int>& variables)
    : SupernodalAssemblerBase(variables),
      Q_sparse_(&Q) {}

SparseQuadraticTermAssembler::SparseQuadraticTermAssembler(
    const Eigen::MatrixXd& Q,
    const std::vector<int>& variables)
    : SupernodalAssemblerBase(variables),
      Q_dense_(&Q), dense_(true) {}

std::vector<std::vector<int>> SparseQuadraticTermAssembler::get_cliques() const {
  // Return edges {i,j} as 2-element cliques.
  // For dense Q, return full row supports.
  std::vector<std::vector<int>> cliques;
  const auto& vars = primal_variables();

  if (dense_ && Q_dense_) {
    // Dense Q: each row is a clique (all variables connected).
    // Return one big clique.
    cliques.push_back(vars);
  } else if (Q_sparse_) {
    // Sparse Q: return individual edges for off-diagonal entries,
    // and singletons for diagonal-only variables.
    std::vector<bool> has_edge(vars.size(), false);
    for (int k = 0; k < Q_sparse_->outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(*Q_sparse_, k);
           it; ++it) {
        int r = it.row(), c = it.col();
        if (r > c && r < static_cast<int>(vars.size()) &&
            c < static_cast<int>(vars.size())) {
          cliques.push_back({vars[c], vars[r]});
          has_edge[r] = true;
          has_edge[c] = true;
        }
      }
    }
    // Include diagonal-only variables as singletons so they appear
    // in the clique tree.
    for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
      if (!has_edge[i] && Q_sparse_->coeff(i, i) != 0) {
        cliques.push_back({vars[i]});
      }
    }
  }
  return cliques;
}

std::vector<SupernodalAssemblerBase*> SparseQuadraticTermAssembler::Decompose(
    const std::vector<std::vector<int>>& maximal_cliques) {
  owned_sub_assemblers_.clear();
  const auto& vars = primal_variables();

  // Build variable-to-index map (original var -> Q index).
  std::unordered_map<int, int> var_to_idx;
  for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
    var_to_idx[vars[i]] = i;
  }

  // For each pair (i,j) in Q, assign it to the SMALLEST maximal clique
  // containing both i and j.  This avoids double-counting entries that
  // appear in separator variables shared between cliques.
  int num_cliques = static_cast<int>(maximal_cliques.size());

  // Build clique membership: var -> set of clique indices.
  std::unordered_map<int, std::vector<int>> var_to_cliques;
  for (int ci = 0; ci < num_cliques; ++ci) {
    for (int v : maximal_cliques[ci]) {
      var_to_cliques[v].push_back(ci);
    }
  }

  // Per-clique Q sub-blocks, initialized to zero.
  struct CliqueInfo {
    std::vector<int> q_indices;  // Q indices for vars in this clique
    std::vector<int> clique_vars;  // original variable indices
    std::unordered_map<int, int> var_to_local;  // var -> local index
  };
  std::vector<CliqueInfo> infos(num_cliques);
  std::vector<Eigen::MatrixXd> Q_blocks;

  for (int ci = 0; ci < num_cliques; ++ci) {
    auto& info = infos[ci];
    for (int v : maximal_cliques[ci]) {
      auto it = var_to_idx.find(v);
      if (it != var_to_idx.end()) {
        info.var_to_local[v] = static_cast<int>(info.q_indices.size());
        info.q_indices.push_back(it->second);
        info.clique_vars.push_back(v);
      }
    }
    int m = static_cast<int>(info.q_indices.size());
    Q_blocks.emplace_back(Eigen::MatrixXd::Zero(m, m));
  }

  // Assign each Q(i,j) to the smallest containing clique.
  auto get_Q = [&](int qi, int qj) -> double {
    if (dense_ && Q_dense_) return (*Q_dense_)(qi, qj);
    if (Q_sparse_) return Q_sparse_->coeff(qi, qj);
    return 0;
  };

  for (int qi = 0; qi < static_cast<int>(vars.size()); ++qi) {
    int vi = vars[qi];
    for (int qj = qi; qj < static_cast<int>(vars.size()); ++qj) {
      int vj = vars[qj];
      double val = get_Q(qi, qj);
      if (val == 0 && qi != qj) continue;
      if (val == 0 && qi == qj) continue;

      // Find smallest clique containing both vi and vj.
      int best_ci = -1;
      size_t best_size = std::numeric_limits<size_t>::max();
      auto it_i = var_to_cliques.find(vi);
      if (it_i == var_to_cliques.end()) continue;
      for (int ci : it_i->second) {
        if (maximal_cliques[ci].size() >= best_size) continue;
        if (infos[ci].var_to_local.count(vj)) {
          best_ci = ci;
          best_size = maximal_cliques[ci].size();
        }
      }
      if (best_ci < 0) continue;

      int li = infos[best_ci].var_to_local[vi];
      int lj = infos[best_ci].var_to_local[vj];
      Q_blocks[best_ci](li, lj) += val;
      if (li != lj) Q_blocks[best_ci](lj, li) += val;
    }
  }

  // Create sub-assemblers for non-zero blocks.
  std::vector<SupernodalAssemblerBase*> result;
  for (int ci = 0; ci < num_cliques; ++ci) {
    if (Q_blocks[ci].squaredNorm() < 1e-30) continue;
    owned_sub_assemblers_.emplace_back(
        std::move(Q_blocks[ci]), infos[ci].clique_vars);
    result.push_back(&owned_sub_assemblers_.back());
  }

  return result;
}

void SparseQuadraticTermAssembler::BindPartition(
    const KKTSolverBase& /*solver*/) {
  // No per-sub-assembler binding needed for the current approach.
  // ComputeBlockProduct gathers globally then uses the sparse Q.
  block_info_.clear();
}

Eigen::VectorXd SparseQuadraticTermAssembler::ComputeBlockProduct(
    const KKTSolverBase& solver) const {
  // Gather x from partition blocks, then compute Q*x.
  const int n = solver.number_of_variables();
  Eigen::VectorXd x_global(n);
  solver.GatherFromBlocks(x_global);
  if (Q_sparse_) return (*Q_sparse_) * x_global;
  if (Q_dense_) return (*Q_dense_) * x_global;
  return Eigen::VectorXd::Zero(n);
}

void SparseQuadraticTermAssembler::AccumulateBlockProduct(
    KKTSolverBase& solver,
    const BlockPartition& x_partition) const {
  // Compute Q*x and scatter into the solver's partition.
  const int n = solver.number_of_variables();
  Eigen::VectorXd x_global(n);
  x_partition.GatherInto(x_global);
  Eigen::VectorXd qx;
  if (Q_sparse_) qx = (*Q_sparse_) * x_global;
  else if (Q_dense_) qx = (*Q_dense_) * x_global;
  else return;
  solver.ScatterToBlocks(qx);
}

namespace {

// Identify isolated diagonal variables: Q(i,i) != 0, no off-diagonal Q
// entries, and no A rows touch variable i. These can be solved directly
// as x_i = rhs_i / Q(i,i) without entering the tree solver.
struct IsolatedDiagResult {
  std::vector<int> isolated_vars;     // original indices of isolated vars
  std::vector<double> diag_values;    // Q(i,i) for each isolated var
  std::vector<int> kept_vars;         // original indices of remaining vars
  std::vector<int> kept_to_original;  // same as kept_vars (for clarity)
};

IsolatedDiagResult FindIsolatedDiagonals(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::SparseMatrix<double>& A, int num_vars) {
  IsolatedDiagResult r;

  // Find which variables A touches.
  std::vector<bool> a_touches(num_vars, false);
  for (int k = 0; k < A.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
      a_touches[it.col()] = true;

  // Find which variables have off-diagonal Q entries.
  std::vector<bool> has_offdiag(num_vars, false);
  for (int k = 0; k < Q.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k); it; ++it)
      if (it.row() != it.col()) {
        has_offdiag[it.row()] = true;
        has_offdiag[it.col()] = true;
      }

  for (int i = 0; i < num_vars; ++i) {
    double qii = Q.coeff(i, i);
    if (!a_touches[i] && !has_offdiag[i] && qii != 0) {
      r.isolated_vars.push_back(i);
      r.diag_values.push_back(qii);
    } else {
      r.kept_vars.push_back(i);
    }
  }
  r.kept_to_original = r.kept_vars;
  return r;
}

// Dense Q: no isolated diagonals (all entries are structurally present).
IsolatedDiagResult FindIsolatedDiagonals(
    const Eigen::MatrixXd& /*Q*/,
    const Eigen::SparseMatrix<double>& /*A*/, int num_vars) {
  IsolatedDiagResult r;
  r.kept_vars.resize(num_vars);
  std::iota(r.kept_vars.begin(), r.kept_vars.end(), 0);
  r.kept_to_original = r.kept_vars;
  return r;
}

// Build a reduced Q and A with isolated variables removed.
Eigen::SparseMatrix<double> ReduceQ(
    const Eigen::SparseMatrix<double>& Q,
    const std::vector<int>& kept_vars) {
  int m = static_cast<int>(kept_vars.size());
  std::unordered_map<int, int> old_to_new;
  for (int i = 0; i < m; ++i) old_to_new[kept_vars[i]] = i;

  std::vector<Eigen::Triplet<double>> trips;
  for (int k = 0; k < Q.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k); it; ++it) {
      auto ir = old_to_new.find(it.row());
      auto ic = old_to_new.find(it.col());
      if (ir != old_to_new.end() && ic != old_to_new.end())
        trips.emplace_back(ir->second, ic->second, it.value());
    }

  Eigen::SparseMatrix<double> Q_reduced(m, m);
  Q_reduced.setFromTriplets(trips.begin(), trips.end());
  return Q_reduced;
}

Eigen::SparseMatrix<double> ReduceA(
    const Eigen::SparseMatrix<double>& A,
    const std::vector<int>& kept_vars) {
  int m = static_cast<int>(kept_vars.size());
  std::unordered_map<int, int> old_to_new;
  for (int i = 0; i < m; ++i) old_to_new[kept_vars[i]] = i;

  std::vector<Eigen::Triplet<double>> trips;
  for (int k = 0; k < A.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      auto ic = old_to_new.find(it.col());
      if (ic != old_to_new.end())
        trips.emplace_back(it.row(), ic->second, it.value());
    }

  Eigen::SparseMatrix<double> A_reduced(A.rows(), m);
  A_reduced.setFromTriplets(trips.begin(), trips.end());
  return A_reduced;
}

template <typename QType>
SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquaresImpl(
    const QType& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  using clock = std::chrono::high_resolution_clock;
  SparseQuadraticTermLeastSquaresResult result;
  const int num_vars = A.cols();

  auto t0 = clock::now();

  // Preprocess: extract isolated diagonal variables and solve them directly.
  auto isolated = FindIsolatedDiagonals(Q, A, num_vars);
  const bool has_isolated = !isolated.isolated_vars.empty();
  const int reduced_vars = static_cast<int>(isolated.kept_vars.size());

  // Pre-solve isolated variables: x_i = rhs_i / Q(i,i).
  Eigen::VectorXd x_full = Eigen::VectorXd::Zero(num_vars);
  for (size_t k = 0; k < isolated.isolated_vars.size(); ++k) {
    int i = isolated.isolated_vars[k];
    x_full(i) = rhs(i) / isolated.diag_values[k];
  }

  if (reduced_vars == 0) {
    // All variables are isolated diagonals.
    result.x = x_full;
    auto t1 = clock::now();
    auto us = [](auto a, auto b) {
      return std::chrono::duration<double, std::micro>(b - a).count();
    };
    result.construction_time_us = us(t0, t1);
    result.factor_time_us = 0;
    result.solve_time_us = 0;
    return result;
  }

  // Build reduced system.
  Eigen::SparseMatrix<double> A_reduced =
      has_isolated ? ReduceA(A, isolated.kept_vars) : A;
  // For sparse Q, reduce to kept variables. For dense Q, has_isolated
  // is always false so Q_reduced_storage is unused.
  Eigen::SparseMatrix<double> Q_reduced_storage;
  const auto& Q_for_solver = [&]() -> const QType& {
    if constexpr (std::is_same_v<QType, Eigen::SparseMatrix<double>>) {
      if (has_isolated) {
        Q_reduced_storage = ReduceQ(Q, isolated.kept_vars);
        return Q_reduced_storage;
      }
    }
    return Q;
  }();

  Eigen::VectorXd rhs_reduced(reduced_vars);
  for (int i = 0; i < reduced_vars; ++i)
    rhs_reduced(i) = rhs(isolated.kept_vars[i]);

  // Build SparseLinearConstraint for A.
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A_reduced.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A_reduced, b_zero);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  for (int i = 0; i < reduced_vars; ++i) var_set.insert(i);
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(reduced_vars);

  auto a_assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(a_assembler.get());

  auto q_assembler = std::make_unique<SparseQuadraticTermAssembler>(
      Q_for_solver, all_vars);
  cm.AddCustomAssembler(q_assembler.get());

  auto t1 = clock::now();

  SolverConfiguration config;
  auto tree_solver = MakeTreeSolver(&cm, config);

  auto t2 = clock::now();

  bool ok = tree_solver->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t3 = clock::now();

  Eigen::VectorXd x_reduced = tree_solver->Solve(rhs_reduced);

  auto t4 = clock::now();

  // Expand solution.
  for (int i = 0; i < reduced_vars; ++i)
    x_full(isolated.kept_vars[i]) = x_reduced(i);

  result.x = x_full;
  auto us = [](auto a, auto b) {
    return std::chrono::duration<double, std::micro>(b - a).count();
  };
  result.construction_time_us = us(t0, t2);
  result.factor_time_us = us(t2, t3);
  result.solve_time_us = us(t3, t4);

  return result;
}

}  // namespace

SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquares(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  return SparseQuadraticTermLeastSquaresImpl(Q, A, rhs);
}

SparseQuadraticTermLeastSquaresResult SparseQuadraticTermLeastSquares(
    const Eigen::MatrixXd& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  return SparseQuadraticTermLeastSquaresImpl(Q, A, rhs);
}

}  // namespace conex
