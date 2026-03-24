#include "conex/common/sparse_linear_constraint.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "conex/common/clique_ordering.h"
#include "conex/common/constraint_manager.h"
#include "conex/tree_solver/kkt_solver_factory.h"
#include "conex/tree_solver/kkt_tree_solver.h"
#include "conex/tree_solver/tree_utils.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
namespace {

using Clock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double, std::micro>;

double us(Clock::time_point t0, Clock::time_point t1) {
  return Duration(t1 - t0).count();
}

// Read a MatrixMarket file into a sparse matrix.
// Handles: coordinate real general, coordinate pattern symmetric.
Eigen::SparseMatrix<double> ReadMTX(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    fprintf(stderr, "Cannot open %s\n", path.c_str());
    std::exit(1);
  }

  std::string line;
  std::getline(f, line);  // header

  bool is_symmetric = line.find("symmetric") != std::string::npos;
  bool is_pattern = line.find("pattern") != std::string::npos;
  bool is_array = line.find("array") != std::string::npos;

  if (is_array) {
    fprintf(stderr, "  Skipping %s (array format, not coordinate)\n",
            path.c_str());
    return Eigen::SparseMatrix<double>(0, 0);
  }

  // Skip comments.
  while (std::getline(f, line) && line[0] == '%') {}

  int nrows, ncols, nnz;
  {
    std::istringstream ss(line);
    ss >> nrows >> ncols >> nnz;
  }

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(is_symmetric ? 2 * nnz : nnz);

  for (int i = 0; i < nnz; ++i) {
    std::getline(f, line);
    std::istringstream ss(line);
    int r, c;
    double v = 1.0;
    ss >> r >> c;
    if (!is_pattern) ss >> v;
    r--; c--;  // 1-indexed to 0-indexed
    triplets.emplace_back(r, c, v);
    if (is_symmetric && r != c) {
      triplets.emplace_back(c, r, v);
    }
  }

  Eigen::SparseMatrix<double> A(nrows, ncols);
  A.setFromTriplets(triplets.begin(), triplets.end());
  return A;
}

struct ProfileResult {
  std::string name;
  int rows, cols, nnz;
  int num_cliques;

  // Times in microseconds.
  double row_support_us;       // SparseLinearConstraint construction
  double clique_tree_us;       // MakeCliqueTreeMinDegreeFromRowSupports
  double decompose_us;         // Assembler Decompose + adapter creation
  double finalize_us;          // tree_solver->Finalize
  double assemble_factor_us;   // AssembleAndFactor (median of repeats)
  double solve_us;             // Solve (median of repeats)
  double total_setup_us;       // row_support + clique_tree + decompose + finalize
  double residual;
};

ContributionType ClassifyCliqueContribution(
    const SupernodalAssemblerBase* assembler, int number_of_primal_variables) {
  for (auto v : assembler->variables()) {
    if (v >= number_of_primal_variables) {
      return ContributionType::kIndefinite;
    }
  }
  return ContributionType::kPositiveDefinite;
}

ProfileResult ProfileMatrix(const std::string& name,
                            const Eigen::SparseMatrix<double>& A) {
  ProfileResult res;
  res.name = name;
  res.rows = A.rows();
  res.cols = A.cols();
  res.nnz = A.nonZeros();

  const int num_vars = A.cols();
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A.rows());

  // --- Stage 1: Row support computation ---
  auto t0 = Clock::now();
  auto slc = std::make_unique<SparseLinearConstraint>(A, b_zero);
  auto t1 = Clock::now();
  res.row_support_us = us(t0, t1);

  // Prepare ConstraintManager.
  std::set<int> var_set;
  for (const auto& sup : slc->row_supports())
    var_set.insert(sup.begin(), sup.end());
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(num_vars);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  auto* asm_ptr = assembler.get();
  cm.AddCustomAssembler(asm_ptr);

  auto clique_assemblers = cm.clique_assemblers();

  // Gather cliques from assemblers.
  std::vector<std::vector<int>> cliques;
  for (const auto& a : clique_assemblers) {
    auto c = a->get_cliques();
    cliques.insert(cliques.end(), c.begin(), c.end());
  }

  // --- Stage 2: Clique tree construction ---
  auto t2 = Clock::now();
  std::vector<std::vector<int>> maximal_cliques;
  SolverConfiguration cfg;
  CliqueTree clique_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      cliques, &maximal_cliques, cfg.tree.max_merge_supernode_size,
      cfg.tree.supernode_reorder_method, {});
  auto t3 = Clock::now();
  res.clique_tree_us = us(t2, t3);
  res.num_cliques = static_cast<int>(maximal_cliques.size());

  // --- Stage 3: Decompose + adapter creation ---
  auto t4 = Clock::now();
  auto tree_solver = std::make_unique<SymmetricLinearSystemTreeSolver>();

  int num_primal = cm.GetNumberOfVariables();
  for (auto* a : clique_assemblers) {
    auto subs = a->Decompose(maximal_cliques);
    for (auto* s : subs) {
      auto adapter =
          std::make_unique<KKTAssemblerToSubsystemAdapter>(s);
      adapter->set_contribution_type(
          ClassifyCliqueContribution(s, num_primal));
      tree_solver->push_back(std::move(adapter));
    }
  }
  auto t5 = Clock::now();
  res.decompose_us = us(t4, t5);

  // --- Stage 4: Finalize (elimination order, subsystem creation, arena) ---
  auto t6 = Clock::now();
  tree_solver->SetUseGenericFactorization(true);
  tree_solver->Finalize(clique_tree);
  tree_solver->SetFactorizationMode(cfg.tree.left_looking);
  tree_solver->EnableAutoUpdateAtAssemble(true);
  tree_solver->SetNumThreads(1);
  auto t7 = Clock::now();
  res.finalize_us = us(t6, t7);

  res.total_setup_us = res.row_support_us + res.clique_tree_us +
                       res.decompose_us + res.finalize_us;

  // --- Stage 5: AssembleAndFactor (repeated, take median) ---
  // Warm up.
  bool ok = tree_solver->AssembleAndFactor();
  if (!ok) {
    res.assemble_factor_us = -1;
    res.solve_us = -1;
    res.residual = -1;
    return res;
  }

  int iters = std::max(20, 2000 / std::max(1, num_vars));
  std::vector<double> af_times(iters);
  for (int i = 0; i < iters; ++i) {
    auto ta = Clock::now();
    tree_solver->AssembleAndFactor();
    auto tb = Clock::now();
    af_times[i] = us(ta, tb);
  }
  std::sort(af_times.begin(), af_times.end());
  res.assemble_factor_us = af_times[iters / 2];

  // --- Stage 6: Solve (repeated, take median) ---
  Eigen::VectorXd x_true = Eigen::VectorXd::Random(num_vars);
  Eigen::MatrixXd Ad(A);
  Eigen::VectorXd rhs = Ad.transpose() * (Ad * x_true);

  // Warm up.
  tree_solver->Solve(rhs);

  std::vector<double> s_times(iters);
  Eigen::VectorXd sol;
  for (int i = 0; i < iters; ++i) {
    auto ta = Clock::now();
    sol = tree_solver->Solve(rhs);
    auto tb = Clock::now();
    s_times[i] = us(ta, tb);
  }
  std::sort(s_times.begin(), s_times.end());
  res.solve_us = s_times[iters / 2];

  res.residual = (sol - x_true).norm() / x_true.norm();

  return res;
}

}  // namespace
}  // namespace conex

int main(int argc, char** argv) {
  using namespace conex;

  if (argc < 2) {
    fprintf(stderr, "Usage: %s <file.mtx> [file2.mtx ...]\n", argv[0]);
    return 1;
  }

  printf("\n");
  printf("%-20s %6s %6s %8s %6s | %10s %10s %10s %10s | %10s %10s | %10s | %10s\n",
         "Matrix", "rows", "cols", "nnz", "cliq",
         "rowsup", "cliqtree", "decomp", "finalize",
         "asm+fac", "solve",
         "setup_tot", "residual");
  printf("%s\n", std::string(140, '-').c_str());

  for (int i = 1; i < argc; ++i) {
    std::string path = argv[i];
    // Extract matrix name from path.
    std::string name = path;
    auto pos = name.rfind('/');
    if (pos != std::string::npos) name = name.substr(pos + 1);
    pos = name.rfind('.');
    if (pos != std::string::npos) name = name.substr(0, pos);

    // Skip _b (rhs) files.
    if (name.size() > 2 && name.substr(name.size() - 2) == "_b") continue;

    auto A = ReadMTX(path);

    if (A.rows() == 0 && A.cols() == 0) continue;  // skipped format

    // For symmetric pattern matrices (A^T A sparsity), A is square symmetric.
    // We need a rectangular A for least squares. Use it directly if rectangular,
    // otherwise skip (not a LS problem).
    if (A.rows() == A.cols()) {
      fprintf(stderr, "  Skipping %s (square, %dx%d) -- not a LS matrix\n",
              name.c_str(), (int)A.rows(), (int)A.cols());
      continue;
    }

    ProfileResult res;
    try {
      res = ProfileMatrix(name, A);
    } catch (const std::exception& e) {
      fprintf(stderr, "  %s: exception: %s\n", name.c_str(), e.what());
      continue;
    }

    if (res.assemble_factor_us < 0) {
      printf("%-20s %6d %6d %8d %6d | %9.0fus %9.0fus %9.0fus %9.0fus |  FACTOR FAILED          | %9.0fus | rank-def\n",
             res.name.c_str(), res.rows, res.cols, res.nnz, res.num_cliques,
             res.row_support_us, res.clique_tree_us,
             res.decompose_us, res.finalize_us,
             res.total_setup_us);
    } else {
      printf("%-20s %6d %6d %8d %6d | %9.0fus %9.0fus %9.0fus %9.0fus | %9.0fus %9.0fus | %9.0fus | %10.2e\n",
             res.name.c_str(), res.rows, res.cols, res.nnz, res.num_cliques,
             res.row_support_us, res.clique_tree_us,
             res.decompose_us, res.finalize_us,
             res.assemble_factor_us, res.solve_us,
             res.total_setup_us, res.residual);
    }
  }

  printf("\n");
  printf("Stages:\n");
  printf("  rowsup    = SparseLinearConstraint construction (row support grouping)\n");
  printf("  cliqtree  = Clique tree construction (min-degree ordering)\n");
  printf("  decomp    = Decompose into per-clique constraints + adapter creation\n");
  printf("  finalize  = Elimination order, subsystem creation, arena allocation\n");
  printf("  asm+fac   = AssembleAndFactor (median of repeated runs)\n");
  printf("  solve     = Triangular solve (median of repeated runs)\n");
  printf("  setup_tot = Total one-time setup (rowsup + cliqtree + decomp + finalize)\n");
  printf("\n");

  return 0;
}
