// Evaluation of ThetaContR across problem classes, switching policies,
// and tolerance settings.
//
// Usage:
//   ./eval_thetacontr                          Run all built-in tests
//   ./eval_thetacontr --sdplib <dir>           Add SDPLIB problems
//   ./eval_thetacontr --qps <dir>              Add QPS problems
//   ./eval_thetacontr --limit N                Limit files per directory
//   ./eval_thetacontr --verbose                Show per-iteration output

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_hybrid_r.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
#include "conex/common/qps_reader.h"
#include "conex/common/sdpa_reader.h"
#include "conex/common/solver.h"

namespace conex {
namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::high_resolution_clock;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// =====================================================================
// Result struct
// =====================================================================
struct EvalResult {
  const char* problem;
  const char* config;
  int factorizations;
  int iterations;
  double objective;
  double stationarity;
  double complementarity;
  double mu;
  double time_ms;
  bool converged;
};

// =====================================================================
// Run a single ThetaContR configuration
// =====================================================================
EvalResult RunConfig(const char* problem_name, const char* config_name,
                     Solver& solver, const Model& model,
                     double tol, double compl_tol,
                     ThetaContRSwitchPolicy policy,
                     bool verbose,
                     double theta_rate = 0.1) {
  ThetaContinuationR algo;
  algo.tolerance = tol;
  algo.compl_tol = compl_tol;
  algo.verbose = verbose;
  algo.policy = policy;
  algo.theta_rate = theta_rate;

  CompiledModel cm(*solver.kkt(), solver.MakeCostRHS());
  auto t0 = Clock::now();
  auto raw = algo.Run(cm);
  auto t1 = Clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // Extract solution via Solver for residuals.
  auto result = solver.Solve(algo);

  double stat = result.duals.stationarity_gradient.size() > 0 ?
      result.duals.stationarity_gradient.norm() : -1;

  // Save theta trace to CSV.
  {
    char fname[128];
    snprintf(fname, sizeof(fname), "theta_trace_%s_%s.csv",
             problem_name, config_name);
    FILE* f = fopen(fname, "w");
    if (f) {
      fprintf(f, "iter,theta,mu,d_inf,gap,factorizations\n");
      for (int i = 0; i < (int)raw.iter_stats.size(); i++) {
        const auto& s = raw.iter_stats[i];
        fprintf(f, "%d,%.15e,%.15e,%.15e,%.15e,%d\n",
                i, s.theta, s.mu, s.d_inf, s.complementarity, s.factorizations);
      }
      fclose(f);
    }
  }

  return {problem_name, config_name, result.factorizations,
          result.iterations, result.objective, stat,
          result.optimality.complementarity, result.mu, ms,
          result.mu < 1e-6 || std::abs(result.gap) < 1e-4};
}

// =====================================================================
// Configurations to evaluate
// =====================================================================
struct Config {
  const char* name;
  double tol;
  double compl_tol;
  ThetaContRSwitchPolicy policy;
  double theta_rate;
};

std::vector<Config> MakeConfigs() {
  auto gap_policy = [](double gap, double, int) { return gap < 0; };
  auto dinf_policy = [](double, double d_inf, int) { return d_inf > 1.0; };
  auto never_center = [](double, double, int) { return false; };

  return {
    // Switching policies (tol=1e-8, compl_tol=1e-12, theta_rate=0.1).
    {"gap_t8_c12",       1e-8,  1e-12, gap_policy,    0.1},
    {"dinf_t8_c12",      1e-8,  1e-12, dinf_policy,   0.1},
    {"noW_t8",           1e-8,  1e30,  never_center,  0.0},

    // Theta-rate sweep (gap policy, tol=1e-12, compl_tol=1e-12).
    {"rate_0.5",         1e-12, 1e-12, gap_policy,    0.5},
    {"rate_0.1",         1e-12, 1e-12, gap_policy,    0.1},
    {"rate_0.01",        1e-12, 1e-12, gap_policy,    0.01},
    {"rate_off",         1e-12, 1e-12, gap_policy,    0.0},

    // compl_tol sweep (gap policy, tol=1e-8, theta_rate=0.1).
    {"gap_t8_c8",        1e-8,  1e-8,  gap_policy,    0.1},
    {"gap_t8_c10",       1e-8,  1e-10, gap_policy,    0.1},
    {"gap_t8_cINF",      1e-8,  1e30,  gap_policy,    0.1},

    // tol sweep (gap policy, compl_tol=1e-12, theta_rate=0.1).
    {"gap_t10_c12",      1e-10, 1e-12, gap_policy,    0.1},
    {"gap_t12_c12",      1e-12, 1e-12, gap_policy,    0.1},
  };
}

// =====================================================================
// Problem sources
// =====================================================================
struct Problem {
  std::string name;
  Model model;
  double objective_constant;
};

std::vector<Problem> MakeSyntheticLPs() {
  std::vector<Problem> problems;
  for (int n : {5, 10, 20, 50}) {
    int m = n / 2 > 0 ? n / 2 : 1;
    srand(42);
    MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
    VectorXd x0 = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);
    VectorXd b = Ad * x0;
    VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < m; i++)
      for (int j = 0; j < n; j++)
        trips.emplace_back(i, j, Ad(i, j));
    Eigen::SparseMatrix<double> A(m, n);
    A.setFromTriplets(trips.begin(), trips.end());
    Eigen::SparseMatrix<double> I(n, n);
    I.setIdentity();
    std::vector<int> vars(n);
    std::iota(vars.begin(), vars.end(), 0);

    Model model;
    model.AddLinearConstraint(I, VectorXd::Zero(n), vars);
    model.AddEqualityConstraint(A, b, vars);
    model.SetLinearCost(c);

    char name[32];
    snprintf(name, sizeof(name), "LP_%dx%d", n, m);
    problems.push_back({name, std::move(model), 0});
  }
  return problems;
}

std::vector<Problem> LoadQPS(const std::string& dir, int limit) {
  std::vector<Problem> problems;
  std::vector<std::pair<uintmax_t, std::string>> files;
  for (const auto& entry : fs::directory_iterator(dir)) {
    auto ext = entry.path().extension().string();
    if (ext == ".QPS" || ext == ".qps")
      files.push_back({entry.file_size(), entry.path().string()});
  }
  std::sort(files.begin(), files.end());
  int count = 0;
  for (const auto& [sz, path] : files) {
    if (limit > 0 && count >= limit) break;
    try {
      auto [model, info] = ReadQPS(path);
      problems.push_back({
          fs::path(path).stem().string(),
          std::move(model),
          info.objective_constant});
      count++;
    } catch (...) {}
  }
  return problems;
}

std::vector<Problem> LoadSDPLIB(const std::string& dir, int limit) {
  std::vector<Problem> problems;
  std::vector<std::pair<uintmax_t, std::string>> files;
  for (const auto& entry : fs::directory_iterator(dir)) {
    auto ext = entry.path().extension().string();
    if (ext == ".dat-s" || entry.path().string().find(".dat-s") != std::string::npos)
      files.push_back({entry.file_size(), entry.path().string()});
  }
  std::sort(files.begin(), files.end());
  int count = 0;
  for (const auto& [sz, path] : files) {
    if (limit > 0 && count >= limit) break;
    try {
      auto [model, info] = ReadSDPA(path);
      problems.push_back({
          fs::path(path).stem().string(),
          std::move(model),
          0});
      count++;
    } catch (...) {}
  }
  return problems;
}

// =====================================================================
// Main
// =====================================================================
void PrintHeader() {
  printf("%-20s %-16s %5s %5s %14s %10s %10s %10s %8s %s\n",
         "Problem", "Config", "fac", "iter", "objective",
         "stat", "compl", "mu", "ms", "ok");
  printf("%s\n", std::string(120, '-').c_str());
}

void PrintResult(const EvalResult& r) {
  printf("%-20s %-16s %5d %5d %14.6e %10.2e %10.2e %10.2e %8.1f %s\n",
         r.problem, r.config, r.factorizations, r.iterations,
         r.objective, r.stationarity, r.complementarity,
         r.mu, r.time_ms, r.converged ? "yes" : "NO");
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  std::string qps_dir, sdplib_dir;
  int limit = 15;
  bool verbose = false;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--qps" && i + 1 < argc) qps_dir = argv[++i];
    else if (arg == "--sdplib" && i + 1 < argc) sdplib_dir = argv[++i];
    else if (arg == "--limit" && i + 1 < argc) limit = std::atoi(argv[++i]);
    else if (arg == "--verbose" || arg == "-v") verbose = true;
  }

  auto configs = conex::MakeConfigs();

  // Collect problems.
  std::vector<conex::Problem> problems = conex::MakeSyntheticLPs();

  if (!qps_dir.empty()) {
    auto qps = conex::LoadQPS(qps_dir, limit);
    problems.insert(problems.end(),
                    std::make_move_iterator(qps.begin()),
                    std::make_move_iterator(qps.end()));
  }
  if (!sdplib_dir.empty()) {
    auto sdp = conex::LoadSDPLIB(sdplib_dir, limit);
    problems.insert(problems.end(),
                    std::make_move_iterator(sdp.begin()),
                    std::make_move_iterator(sdp.end()));
  }

  printf("ThetaContR Evaluation: %d problems x %d configs\n\n",
         (int)problems.size(), (int)configs.size());

  // CSV output for visualization.
  FILE* csv = fopen("eval_thetacontr.csv", "w");
  if (csv) {
    fprintf(csv, "problem,config,factorizations,iterations,objective,"
                 "stationarity,complementarity,mu,time_ms\n");
  }

  conex::PrintHeader();

  // Run all combinations.
  std::vector<conex::EvalResult> all_results;
  for (auto& prob : problems) {
    for (const auto& cfg : configs) {
      try {
        conex::SolverConfiguration scfg;
        auto solver = conex::Solver::Build(prob.model, scfg);
        auto result = conex::RunConfig(
            prob.name.c_str(), cfg.name,
            solver, prob.model,
            cfg.tol, cfg.compl_tol, cfg.policy, verbose, cfg.theta_rate);
        result.objective += prob.objective_constant;
        conex::PrintResult(result);
        all_results.push_back(result);
        if (csv) {
          fprintf(csv, "%s,%s,%d,%d,%.10e,%.10e,%.10e,%.10e,%.2f\n",
                  result.problem, result.config, result.factorizations,
                  result.iterations, result.objective, result.stationarity,
                  result.complementarity, result.mu, result.time_ms);
        }
      } catch (const std::exception& e) {
        printf("%-20s %-16s  SKIP: %s\n", prob.name.c_str(), cfg.name, e.what());
      }
    }
    printf("\n");
  }

  if (csv) {
    fclose(csv);
    printf("CSV written to eval_thetacontr.csv\n");
  }

  // Summary: per-config aggregates.
  printf("\n=== SUMMARY ===\n");
  printf("%-16s %6s %6s %6s %6s\n",
         "Config", "ok", "total", "avg_fac", "avg_ms");
  printf("%s\n", std::string(50, '-').c_str());
  for (const auto& cfg : configs) {
    int ok = 0, total = 0;
    double sum_fac = 0, sum_ms = 0;
    for (const auto& r : all_results) {
      if (std::string(r.config) == cfg.name) {
        total++;
        if (r.converged) ok++;
        sum_fac += r.factorizations;
        sum_ms += r.time_ms;
      }
    }
    if (total > 0) {
      printf("%-16s %4d/%-2d %6.1f %6.1f\n",
             cfg.name, ok, total, sum_fac / total, sum_ms / total);
    }
  }

  return 0;
}
