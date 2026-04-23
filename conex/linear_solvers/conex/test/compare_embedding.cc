#include <cstdio>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/conex.h"
#include "conex/common/extended_embedding.h"
#include "conex/common/model.h"
#include "conex/common/qps_reader.h"
#include "conex/common/solver.h"

using namespace conex;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char* argv[]) {
  bool verbose = false;
  bool use_lu = false;
  int n_only = 0;
  int m_only = 0;
  std::string qps_file;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--verbose" || arg == "-v") verbose = true;
    else if (arg == "--lu" || arg == "-lu") use_lu = true;
    else if ((arg == "--n" || arg == "-n") && i + 1 < argc) n_only = std::atoi(argv[++i]);
    else if ((arg == "--m" || arg == "-m") && i + 1 < argc) m_only = std::atoi(argv[++i]);
    else if ((arg == "--qps" || arg == "-qps") && i + 1 < argc) qps_file = argv[++i];
  }
  SolverConfiguration cfg;
  if (use_lu) cfg.tree.use_lu_for_indefinite = true;
  printf("%-5s %-8s %-12s %4s %12s %10s %10s %10s\n",
         "n", "method", "algo", "fac", "obj", "stat", "compl", "eq/theta");
  printf("%s\n", std::string(80, '-').c_str());

  // --- QPS file mode ---
  if (!qps_file.empty()) {
    auto [qps_model, qps_info] = ReadQPS(qps_file);
    printf("QPS: %s  n=%d  eq=%d  ineq=%d  Q=%d\n",
           qps_info.name.c_str(), qps_info.num_variables,
           qps_info.num_equality_rows, qps_info.num_inequality_rows,
           qps_info.num_quadratic_entries);
    if (qps_info.num_quadratic_entries > 0) {
      printf("ERROR: QPS has quadratic cost (%d entries), not a pure LP.\n",
             qps_info.num_quadratic_entries);
      return 1;
    }

    // Direct solve.
    {
      auto solver = Solver::Build(qps_model, cfg);
      auto r1 = solver.Solve(GeodesicLP{1e-10, 30, 0, verbose});
      double eq1 = 0;
      for (const auto& er : r1.duals.eq_residual)
        eq1 = std::max(eq1, er.norm());
      printf("      DIRECT   GeodesicLP  %4d %12.6f %10.2e %10.2e %10.2e\n",
             r1.factorizations, r1.objective,
             r1.duals.stationarity_gradient.norm(),
             r1.optimality.complementarity, eq1);

      auto r2 = solver.Solve(HybridOnly{1e-10, 500, verbose});
      double eq2 = 0;
      for (const auto& er : r2.duals.eq_residual)
        eq2 = std::max(eq2, er.norm());
      printf("      DIRECT   Hybrid      %4d %12.6f %10.2e %10.2e %10.2e\n",
             r2.factorizations, r2.objective,
             r2.duals.stationarity_gradient.norm(),
             r2.optimality.complementarity, eq2);

      auto r3 = solver.Solve(ThetaContinuation{1e-10, 500, 1, verbose});
      double eq3 = 0;
      for (const auto& er : r3.duals.eq_residual)
        eq3 = std::max(eq3, er.norm());
      printf("      DIRECT   ThetaCont   %4d %12.6f %10.2e %10.2e %10.2e\n",
             r3.factorizations, r3.objective,
             r3.duals.stationarity_gradient.norm(),
             r3.optimality.complementarity, eq3);
    }

    // Embedding solve (from Model).
    {
      auto emb = BuildExtendedEmbedding(qps_model);
      if (emb.model.num_constraints() == 0) {
        printf("      EMBED    (skipped: m > n)\n");
      } else {
        auto solver = Solver::Build(emb.model, emb.tree, cfg);
        int n = emb.info.n;
        const auto& c = qps_model.linear_cost();

        auto r1 = solver.Solve(GeodesicLP{1e-10, 30, 0, verbose});
        double tau1 = r1.x(emb.info.tau_idx());
        double obj1 = (tau1 > 1e-8) ?
            c.dot(r1.x.segment(emb.info.x_start(), n) / tau1) : NAN;
        double eq1 = 0;
        for (const auto& er : r1.duals.eq_residual)
          eq1 = std::max(eq1, er.norm());
        printf("      EMBED    GeodesicLP  %4d %12.6f %10.2e %10.2e %10.2e\n",
               r1.factorizations, obj1,
               r1.duals.stationarity_gradient.norm(),
               r1.optimality.complementarity, eq1);

        auto r2 = solver.Solve(HybridOnly{1e-10, 500, verbose});
        double tau2 = r2.x(emb.info.tau_idx());
        double obj2 = (tau2 > 1e-8) ?
            c.dot(r2.x.segment(emb.info.x_start(), n) / tau2) : NAN;
        double eq2 = 0;
        for (const auto& er : r2.duals.eq_residual)
          eq2 = std::max(eq2, er.norm());
        printf("      EMBED    Hybrid      %4d %12.6f %10.2e %10.2e %10.2e\n",
               r2.factorizations, obj2,
               r2.duals.stationarity_gradient.norm(),
               r2.optimality.complementarity, eq2);
      }
    }
    return 0;
  }

  // --- Random LP mode ---
  std::vector<int> sizes = {5, 10, 20, 50};
  if (n_only > 0) sizes = {n_only};
  for (int n : sizes) {
    int m = m_only > 0 ? m_only : (n / 2 > 0 ? n / 2 : 1);
    srand(42);
    MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
    // Standard form LP: min c'x s.t. Ax = b, x >= 0.
    // b = A*x0 for random x0 > 0 (x0 is strictly feasible).
    // c = random positive.
    VectorXd x0 = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);
    VectorXd b = Ad * x0;
    VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < m; i++)
      for (int j = 0; j < n; j++)
        trips.emplace_back(i, j, Ad(i, j));
    Eigen::SparseMatrix<double> A(m, n);
    A.setFromTriplets(trips.begin(), trips.end());

    // Identity for x >= 0.
    Eigen::SparseMatrix<double> I(n, n);
    I.setIdentity();

    std::vector<int> vars(n);
    std::iota(vars.begin(), vars.end(), 0);

    // --- Direct solve: min c'x s.t. Ax = b, x >= 0 ---
    {
      Model model;
      model.AddLinearConstraint(I, VectorXd::Zero(n), vars);  // x >= 0
      model.AddEqualityConstraint(A, b, vars);                 // Ax = b
      model.SetLinearCost(c);

      auto solver = Solver::Build(model, cfg);

      auto r1 = solver.Solve(GeodesicLP{1e-10, 30, 0, verbose});
      double eq1 = 0;
      for (const auto& er : r1.duals.eq_residual)
        eq1 = std::max(eq1, er.norm());
      printf("n=%2d  DIRECT   GeodesicLP  %4d %12.6f %10.2e %10.2e %10.2e\n",
             n, r1.factorizations, r1.objective,
             r1.duals.stationarity_gradient.norm(),
             r1.optimality.complementarity, eq1);

      auto r2 = solver.Solve(HybridOnly{1e-10, 500, verbose});
      double eq2 = 0;
      for (const auto& er : r2.duals.eq_residual)
        eq2 = std::max(eq2, er.norm());
      printf("n=%2d  DIRECT   Hybrid      %4d %12.6f %10.2e %10.2e %10.2e\n",
             n, r2.factorizations, r2.objective,
             r2.duals.stationarity_gradient.norm(),
             r2.optimality.complementarity, eq2);

      auto r3 = solver.Solve(ThetaContinuation{1e-10, 500, 1, verbose});
      double eq3 = 0;
      for (const auto& er : r3.duals.eq_residual)
        eq3 = std::max(eq3, er.norm());
      printf("n=%2d  DIRECT   ThetaCont   %4d %12.6f %10.2e %10.2e %10.2e\n",
             n, r3.factorizations, r3.objective,
             r3.duals.stationarity_gradient.norm(),
             r3.optimality.complementarity, eq3);

      auto r4 = solver.Solve(GeodesicHSD{1e-10, 30, verbose});
      double eq4 = 0;
      for (const auto& er : r4.duals.eq_residual)
        eq4 = std::max(eq4, er.norm());
      printf("n=%2d  DIRECT   GeodesicHSD %4d %12.6f %10.2e %10.2e %10.2e\n",
             n, r4.factorizations, r4.objective,
             r4.duals.stationarity_gradient.norm(),
             r4.optimality.complementarity, eq4);

      auto r5 = solver.Solve(HybridR{1e-10, 500, verbose});
      double eq5 = 0;
      for (const auto& er : r5.duals.eq_residual)
        eq5 = std::max(eq5, er.norm());
      printf("n=%2d  DIRECT   HybridR     %4d %12.6f %10.2e %10.2e %10.2e\n",
             n, r5.factorizations, r5.objective,
             r5.duals.stationarity_gradient.norm(),
             r5.optimality.complementarity, eq5);
    }

    // --- Embedding solve (from A,b,c) ---
    auto run_embedding = [&](const char* label, const ExtendedEmbedding& emb) {
      auto solver = Solver::Build(emb.model, emb.tree, cfg);

      auto r1 = solver.Solve(GeodesicLP{1e-10, 30, 0, verbose});
      double tau1 = r1.x(emb.info.tau_idx());
      double obj1 = (tau1 > 1e-8) ?
          c.dot(r1.x.segment(emb.info.x_start(), n) / tau1) : NAN;
      double eq1 = 0;
      for (const auto& er : r1.duals.eq_residual)
        eq1 = std::max(eq1, er.norm());
      printf("n=%2d  %-8s GeodesicLP  %4d %12.6f %10.2e %10.2e %10.2e\n",
             n, label, r1.factorizations, obj1,
             r1.duals.stationarity_gradient.norm(),
             r1.optimality.complementarity, eq1);

      auto r2 = solver.Solve(HybridOnly{1e-10, 500, verbose});
      double tau2 = r2.x(emb.info.tau_idx());
      double obj2 = (tau2 > 1e-8) ?
          c.dot(r2.x.segment(emb.info.x_start(), n) / tau2) : NAN;
      double eq2 = 0;
      for (const auto& er : r2.duals.eq_residual)
        eq2 = std::max(eq2, er.norm());
      printf("n=%2d  %-8s Hybrid      %4d %12.6f %10.2e %10.2e %10.2e\n",
             n, label, r2.factorizations, obj2,
             r2.duals.stationarity_gradient.norm(),
             r2.optimality.complementarity, eq2);
    };

    // From (A, b, c) directly.
    auto emb_abc = BuildExtendedEmbedding(A, b, c);
    run_embedding("EMB(Abc)", emb_abc);

    // From Model.
    {
      Model lp_model;
      lp_model.AddLinearConstraint(I, VectorXd::Zero(n), vars);
      lp_model.AddEqualityConstraint(A, b, vars);
      lp_model.SetLinearCost(c);
      auto emb_model = BuildExtendedEmbedding(lp_model);
      run_embedding("EMB(Mod)", emb_model);
    }
    printf("\n");
  }
  return 0;
}
