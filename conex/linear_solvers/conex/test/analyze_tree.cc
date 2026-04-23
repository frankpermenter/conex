#include <cstdio>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/extended_embedding.h"
#include "conex/common/solver.h"
#include "conex/tree_solver/kkt_tree_solver.h"

using namespace conex;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
  // n=6, m=3, p=2.  A is 3x6, C is 2x3.
  srand(42);
  const int n = 6, m = 3, p = 2;
  MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  Eigen::SparseMatrix<double> A = Ad.sparseView();
  VectorXd b = Ad * VectorXd::Ones(n);
  VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

  MatrixXd Cd = MatrixXd::Random(p, m).cwiseAbs() + 0.1 * MatrixXd::Ones(p, m);
  Eigen::SparseMatrix<double> C = Cd.sparseView();
  VectorXd d = VectorXd::Random(p).cwiseAbs() + 0.1 * VectorXd::Ones(p);

  auto [emb_model, info, emb_tree] = BuildExtendedEmbedding(A, b, c, C, d);

  printf("Embedding: n=%d m=%d p=%d\n", n, m, p);
  printf("Variable layout:\n");
  printf("  x:     [%d, %d)\n", info.x_start(), info.x_start() + n);
  printf("  y:     [%d, %d)\n", info.y_start(), info.y_start() + m);
  printf("  w:     [%d, %d)\n", info.w_start(), info.w_start() + p);
  printf("  s:     [%d, %d)\n", info.s_start(), info.s_start() + n);
  printf("  tau:   %d\n", info.tau_idx());
  printf("  kappa: %d\n", info.kappa_idx());
  printf("  theta: %d\n", info.theta_idx());
  printf("  total: %d\n", info.total_vars());

  int N = info.total_vars();
  printf("Duals (allocated in constraint order):\n");
  printf("  nu1 (E1, %d rows): [%d, %d)\n", m, N, N + m);
  printf("  nu2 (E2, %d rows): [%d, %d)\n", n, N + m, N + m + n);
  printf("  nu3 (E3, %d rows): [%d, %d)\n", p, N + m + n, N + m + n + p);
  printf("  nu_gap (E4):       %d\n", N + m + n + p);
  printf("  nu_norm (E5):      %d\n", N + m + n + p + 1);

  // Build with AMD.
  auto solver = Solver::Build(emb_model);
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(solver.kkt());
  if (!ts) { printf("Not a tree solver\n"); return 1; }

  auto amd_tree = ts->GetCliqueTree();
  const auto& pinv = ts->perm_inv();
  int ns = (int)amd_tree.supernodes.size();
  printf("\nAMD tree: %d cliques, %d total KKT vars\n", ns, ts->number_of_variables());

  auto var_name = [&](int v) -> std::string {
    if (v < info.x_start() + n) return "x" + std::to_string(v - info.x_start());
    if (v < info.y_start() + m) return "y" + std::to_string(v - info.y_start());
    if (v < info.w_start() + p) return "w" + std::to_string(v - info.w_start());
    if (v < info.s_start() + n) return "s" + std::to_string(v - info.s_start());
    if (v == info.tau_idx()) return "tau";
    if (v == info.kappa_idx()) return "kappa";
    if (v == info.theta_idx()) return "theta";
    int dv = v - N;
    if (dv < m) return "nu1_" + std::to_string(dv);
    dv -= m;
    if (dv < n) return "nu2_" + std::to_string(dv);
    dv -= n;
    if (dv < p) return "nu3_" + std::to_string(dv);
    dv -= p;
    if (dv == 0) return "nu_gap";
    if (dv == 1) return "nu_norm";
    return "?" + std::to_string(v);
  };

  for (int k = 0; k < ns; ++k) {
    printf("\n  clique %d (parent=%d):\n", k, amd_tree.node_to_parent[k]);
    printf("    sn(%d)=[", (int)amd_tree.supernodes[k].size());
    for (int v : amd_tree.supernodes[k])
      printf("%s,", var_name(pinv(v)).c_str());
    printf("]\n    sep(%d)=[", (int)amd_tree.separators[k].size());
    for (int v : amd_tree.separators[k])
      printf("%s,", var_name(pinv(v)).c_str());
    printf("]\n");
  }

  return 0;
}
