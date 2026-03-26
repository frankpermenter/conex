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
                            const Eigen::SparseMatrix<double>& A,
                            const SolverConfiguration& cfg) {
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
  cm.AddCustomAssembler(std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars));
  cm.Preprocess();
  if (cm.was_reduced()) {
    int orig = cm.GetOriginalNumberOfVariables();
    int reduced = cm.GetNumberOfVariables();
    fprintf(stderr, "  %s: structural rank %d / %d (dropped %d columns)\n",
            name.c_str(), reduced, orig, orig - reduced);
  }

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
  tree_solver->Finalize(clique_tree, cfg.rhs_cols);
  tree_solver->SetFactorizationMode(cfg.tree.left_looking);
  tree_solver->EnableAutoUpdateAtAssemble(true);
  tree_solver->SetNumThreads(cfg.num_threads);
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

using namespace conex;

void PrintHeader() {
  printf("%-20s %4s %4s %6s | %10s %10s %10s %10s | %10s %10s | %10s | %10s\n",
         "Matrix", "thrd", "merg", "cliq",
         "rowsup", "cliqtree", "decomp", "finalize",
         "asm+fac", "solve",
         "setup_tot", "residual");
  printf("%s\n", std::string(130, '-').c_str());
}

void PrintResult(const ProfileResult& res, const SolverConfiguration& cfg) {
  if (res.assemble_factor_us < 0) {
    printf("%-20s %4d %4d %6d | %9.0fus %9.0fus %9.0fus %9.0fus |  FACTOR FAILED          | %9.0fus | rank-def\n",
           res.name.c_str(), cfg.num_threads,
           cfg.tree.max_merge_supernode_size, res.num_cliques,
           res.row_support_us, res.clique_tree_us,
           res.decompose_us, res.finalize_us,
           res.total_setup_us);
  } else {
    printf("%-20s %4d %4d %6d | %9.0fus %9.0fus %9.0fus %9.0fus | %9.0fus %9.0fus | %9.0fus | %10.2e\n",
           res.name.c_str(), cfg.num_threads,
           cfg.tree.max_merge_supernode_size, res.num_cliques,
           res.row_support_us, res.clique_tree_us,
           res.decompose_us, res.finalize_us,
           res.assemble_factor_us, res.solve_us,
           res.total_setup_us, res.residual);
  }
}

struct MatrixFile {
  std::string path;
  std::string name;
  Eigen::SparseMatrix<double> A;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr,
      "Usage: %s [options] <file.mtx> [file2.mtx ...]\n"
      "\n"
      "Options:\n"
      "  --threads <n>          Number of threads (default: 1)\n"
      "  --merge <n>            max_merge_supernode_size (default: 5)\n"
      "  --reorder <n>          supernode_reorder_method (default: 0)\n"
      "  --generic              Use generic (RLDLT) factorization\n"
      "  --randomize            Replace values with random N(0,1), keep sparsity\n"
      "  --sweep-threads <list> Sweep thread counts (comma-separated, e.g. 1,2,4)\n"
      "  --sweep-merge <list>   Sweep merge sizes (comma-separated, e.g. 0,3,5,10)\n"
      , argv[0]);
    return 1;
  }

  SolverConfiguration cfg;
  bool randomize = false;
  bool drop_zero_cols = false;
  std::vector<int> sweep_threads;
  std::vector<int> sweep_merge;
  std::vector<std::string> mtx_paths;

  auto parse_list = [](const char* s) {
    std::vector<int> vals;
    std::istringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      vals.push_back(std::stoi(tok));
    }
    return vals;
  };

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--threads" && i + 1 < argc) {
      cfg.num_threads = std::stoi(argv[++i]);
    } else if (arg == "--merge" && i + 1 < argc) {
      cfg.tree.max_merge_supernode_size = std::stoi(argv[++i]);
    } else if (arg == "--reorder" && i + 1 < argc) {
      cfg.tree.supernode_reorder_method = std::stoi(argv[++i]);
    } else if (arg == "--generic") {
      cfg.tree.use_generic_factorization = true;
    } else if (arg == "--lu") {
      cfg.tree.use_lu_for_indefinite = true;
    } else if (arg == "--randomize") {
      randomize = true;
    } else if (arg == "--drop-zero-cols") {
      drop_zero_cols = true;
    } else if (arg == "--sweep-threads" && i + 1 < argc) {
      sweep_threads = parse_list(argv[++i]);
    } else if (arg == "--sweep-merge" && i + 1 < argc) {
      sweep_merge = parse_list(argv[++i]);
    } else if (arg[0] != '-') {
      mtx_paths.push_back(arg);
    } else {
      fprintf(stderr, "Unknown option: %s\n", arg.c_str());
      return 1;
    }
  }

  // Load matrices.
  std::vector<MatrixFile> matrices;
  for (const auto& path : mtx_paths) {
    std::string name = path;
    auto pos = name.rfind('/');
    if (pos != std::string::npos) name = name.substr(pos + 1);
    pos = name.rfind('.');
    if (pos != std::string::npos) name = name.substr(0, pos);

    if (name.size() > 2 && name.substr(name.size() - 2) == "_b") continue;

    auto A = ReadMTX(path);
    if (A.rows() == 0 && A.cols() == 0) continue;
    if (A.rows() == A.cols()) {
      fprintf(stderr, "  Skipping %s (square, %dx%d)\n",
              name.c_str(), (int)A.rows(), (int)A.cols());
      continue;
    }
    if (drop_zero_cols) {
      // Remove columns with no nonzeros.
      std::vector<bool> has_nz(A.cols(), false);
      for (int k = 0; k < A.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
          has_nz[it.col()] = true;
      std::vector<int> col_map;  // old col -> new col
      for (int c = 0; c < A.cols(); ++c)
        if (has_nz[c]) col_map.push_back(c);
      if (static_cast<int>(col_map.size()) < A.cols()) {
        int new_cols = static_cast<int>(col_map.size());
        std::vector<Eigen::Triplet<double>> trips;
        std::vector<int> inv_map(A.cols(), -1);
        for (int i = 0; i < new_cols; ++i) inv_map[col_map[i]] = i;
        for (int k = 0; k < A.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
            if (inv_map[it.col()] >= 0)
              trips.emplace_back(it.row(), inv_map[it.col()], it.value());
        fprintf(stderr, "  %s: dropped %d zero columns (%d -> %d)\n",
                name.c_str(), A.cols() - new_cols, (int)A.cols(), new_cols);
        Eigen::SparseMatrix<double> A2(A.rows(), new_cols);
        A2.setFromTriplets(trips.begin(), trips.end());
        A = std::move(A2);
      }
    }
    if (randomize) {
      // Replace values with N(0,1) random entries, preserving sparsity.
      srand(42);
      for (int k = 0; k < A.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
          // Box-Muller for N(0,1).
          double u1 = (static_cast<double>(rand()) + 1.0) / (RAND_MAX + 2.0);
          double u2 = static_cast<double>(rand()) / (RAND_MAX + 1.0);
          it.valueRef() = std::sqrt(-2.0 * std::log(u1)) *
                          std::cos(2.0 * M_PI * u2);
        }
      }
      name += "_rand";
    }
    matrices.push_back({path, name, std::move(A)});
  }

  // Build list of configurations to run.
  struct Run { SolverConfiguration cfg; };
  std::vector<Run> runs;

  if (!sweep_threads.empty() || !sweep_merge.empty()) {
    auto threads_list = sweep_threads.empty()
        ? std::vector<int>{cfg.num_threads} : sweep_threads;
    auto merge_list = sweep_merge.empty()
        ? std::vector<int>{cfg.tree.max_merge_supernode_size} : sweep_merge;
    for (int t : threads_list) {
      for (int m : merge_list) {
        SolverConfiguration c = cfg;
        c.num_threads = t;
        c.tree.max_merge_supernode_size = m;
        runs.push_back({c});
      }
    }
  } else {
    runs.push_back({cfg});
  }

  printf("\n");
  PrintHeader();

  for (const auto& run : runs) {
    for (const auto& mat : matrices) {
      ProfileResult res;
      try {
        res = ProfileMatrix(mat.name, mat.A, run.cfg);
      } catch (const std::exception& e) {
        fprintf(stderr, "  %s: exception: %s\n", mat.name.c_str(), e.what());
        continue;
      }
      PrintResult(res, run.cfg);
    }
    if (runs.size() > 1) printf("\n");
  }

  printf("\n");
  printf("Columns: thrd=num_threads, merg=max_merge_supernode_size, cliq=num_cliques\n");
  printf("Stages:  rowsup/cliqtree/decomp/finalize are one-time setup\n");
  printf("         asm+fac/solve are median of repeated runs\n");
  printf("\n");

  return 0;
}
