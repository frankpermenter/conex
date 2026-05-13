// Canary: deterministic numerical fingerprint for each geodesic IPM algorithm.
// Run before and after cleanup to verify byte-identical output.
#include <cstdio>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/kkt_solver_dense.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/linear_solvers/kkt_tree_solver.h"
#include "conex/common/equality_constraint.h"
using namespace conex;

// Fixed 4-variable LP: min [1,2,3,4]'x  s.t.  x >= 0, with b=0.
static Solver MakeLP4() {
  Model p;
  Eigen::SparseMatrix<double> A(4, 4); A.setIdentity();
  Eigen::VectorXd b = Eigen::VectorXd::Zero(4);
  std::vector<int> v = {0, 1, 2, 3};
  p.AddLinearConstraint(A, b, v);
  Eigen::VectorXd c(4); c << 1, 2, 3, 4;
  p.SetLinearCost(c);
  return Solver::Build(p);
}

// Fixed 3x3 SDP: min Tr(C X) s.t. X ≽ 0 with B = -I.
static Solver MakeSDP3() {
  const int n = 3;
  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars;
  int vi = 0;
  for (int i = 0; i < n; ++i)
    for (int j = i; j < n; ++j) {
      Eigen::SparseMatrix<double> Aij(n, n);
      if (i == j) {
        Aij.insert(i, i) = 1.0;
      } else {
        Aij.insert(i, j) = 1.0;
        Aij.insert(j, i) = 1.0;
      }
      Aij.makeCompressed();
      A_list.push_back(Aij);
      vars.push_back(vi++);
    }
  Eigen::SparseMatrix<double> B(n, n);
  for (int i = 0; i < n; ++i) B.insert(i, i) = -1.0;
  B.makeCompressed();

  Model p;
  p.AddPSDConstraint(A_list, B, vars, false);
  Eigen::VectorXd c(vi);
  c << 1, 0.5, 0.3, 2, 0.1, 3;  // cost on upper triangle
  p.SetLinearCost(c);
  return Solver::Build(p);
}

// LP with equality: min [1,2,1,2]'x s.t. x>=0, x0+x1=1, x2+x3=2
static Solver MakeLPEq() {
  Model p;
  const int n = 4;
  Eigen::SparseMatrix<double> A(n, n); A.setIdentity();
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
  std::vector<int> v = {0, 1, 2, 3};
  p.AddLinearConstraint(A, b, v);

  // Two equality constraints.
  Eigen::SparseMatrix<double> C1(1, 2);
  C1.insert(0, 0) = 1; C1.insert(0, 1) = 1; C1.makeCompressed();
  Eigen::VectorXd d1(1); d1(0) = 1.0;
  std::vector<int> eq1 = {0, 1};
  p.AddEqualityConstraint(C1, d1, eq1);

  Eigen::SparseMatrix<double> C2(1, 2);
  C2.insert(0, 0) = 1; C2.insert(0, 1) = 1; C2.makeCompressed();
  Eigen::VectorXd d2(1); d2(0) = 2.0;
  std::vector<int> eq2 = {2, 3};
  p.AddEqualityConstraint(C2, d2, eq2);

  Eigen::VectorXd c(n); c << 1, 2, 1, 2;
  p.SetLinearCost(c);
  return Solver::Build(p);
}

static void PrintResult(const char* label, const GeodesicResult& r) {
  printf("[%s] iters=%d mu=%.16e dinf=%.16e dsq=%.16e compl=%.16e\n",
         label, r.iterations, r.mu, r.d_inf_norm, r.d_sq_norm,
         r.complementarity);
  if (r.x.size() > 0) {
    printf("[%s] x=", label);
    for (int i = 0; i < r.x.size(); ++i) printf(" %.16e", r.x[i]);
    printf("\n");
  }
  printf("[%s] opt: dual_res=%.16e compl=%.16e mins=%.16e mind=%.16e\n",
         label, r.optimality.dual_residual, r.optimality.complementarity,
         r.optimality.min_slack, r.optimality.min_dual);
}

static SolverRHS BuildCostRHS(KKTSolverBase& kkt,
                               const Eigen::VectorXd& cost) {
  int nv = kkt.number_of_variables();
  Eigen::VectorXd cf = Eigen::VectorXd::Zero(nv);
  int nc = std::min((int)cost.size(), nv);
  cf.head(nc) = cost.head(nc);
  auto rhs = kkt.MakeSolverRHS();
  rhs = MakeBlockVariable(kkt, cf);
  return rhs;
}

int main() {
  // === LP4: GeodesicCenter + GeodesicLineSearch + SolveGeodesicLP ===
  {
    auto solver = MakeLP4();
    auto* kkt = solver.kkt();
    Eigen::VectorXd c(4); c << 1, 2, 3, 4;
    auto cost_rhs = kkt->MakeSolverRHS();
    cost_rhs = MakeBlockVariable(*kkt, c);

    CompiledModel cm(*kkt, cost_rhs);

    // GeodesicCenter at k=1.
    RowSpace W1 = kkt->MakeRowSpace(); setOnes(W1);
    auto rc = GeodesicCenter(cm, W1, 1.0, 20, 1e-10);
    PrintResult("LP4_Center", rc);

    // GeodesicLineSearch.
    double k_new = GeodesicLineSearch(cm, W1);
    printf("[LP4_LineSearch] k=%.16e\n", k_new);

    // SolveGeodesicLP.
    RowSpace W2 = kkt->MakeRowSpace(); setOnes(W2);
    auto rlp = SolveGeodesicLP(cm, W2, 30, 0, 1e-8);
    PrintResult("LP4_LP", rlp);
  }

  // === LP4: ThetaContinuation ===
  {
    auto solver = MakeLP4();
    auto* kkt = solver.kkt();
    Eigen::VectorXd c(4); c << 1, 2, 3, 4;
    auto cost_rhs = kkt->MakeSolverRHS();
    cost_rhs = MakeBlockVariable(*kkt, c);
    CompiledModel cm(*kkt, cost_rhs);
    RowSpace W = kkt->MakeRowSpace(); setOnes(W);
    kkt->SetScaling(W); kkt->AssembleAndFactor();
    auto r = SolveGeodesicThetaContinuation(cm, W, 100, 1, 1e-8);
    PrintResult("LP4_ThetaCont", r);
  }

  // === LP4: PhaseOne ===
  {
    auto solver = MakeLP4();
    auto* kkt = solver.kkt();
    Eigen::VectorXd c(4); c << 1, 2, 3, 4;
    auto cost_rhs = kkt->MakeSolverRHS();
    cost_rhs = MakeBlockVariable(*kkt, c);
    CompiledModel cm(*kkt, cost_rhs);
    RowSpace W = kkt->MakeRowSpace(); setOnes(W);
    kkt->SetScaling(W); kkt->AssembleAndFactor();
    auto r = SolveGeodesicPhaseOne(cm, W, 100, 1, 1e-8);
    PrintResult("LP4_PhaseOne", r);
  }

  // === LP4: Hybrid ===
  {
    auto solver = MakeLP4();
    auto* kkt = solver.kkt();
    Eigen::VectorXd c(4); c << 1, 2, 3, 4;
    auto cost_rhs = kkt->MakeSolverRHS();
    cost_rhs = MakeBlockVariable(*kkt, c);
    CompiledModel cm(*kkt, cost_rhs);
    RowSpace W = kkt->MakeRowSpace(); setOnes(W);
    kkt->SetScaling(W); kkt->AssembleAndFactor();
    auto r = SolveGeodesicHybrid(cm, W, 100, 1e-8);
    PrintResult("LP4_Hybrid", r);
  }

  // === SDP3: SolveGeodesicLP ===
  {
    auto solver = MakeSDP3();
    auto* kkt = solver.kkt();
    Eigen::VectorXd c(6); c << 1, 0.5, 0.3, 2, 0.1, 3;
    auto cost_rhs = kkt->MakeSolverRHS();
    cost_rhs = MakeBlockVariable(*kkt, c);
    CompiledModel cm(*kkt, cost_rhs);
    RowSpace W = kkt->MakeRowSpace(); setOnes(W);
    auto r = SolveGeodesicLP(cm, W, 30, 0, 1e-8);
    PrintResult("SDP3_LP", r);
  }

  // === SDP3: ThetaContinuation ===
  {
    auto solver = MakeSDP3();
    auto* kkt = solver.kkt();
    Eigen::VectorXd c(6); c << 1, 0.5, 0.3, 2, 0.1, 3;
    auto cost_rhs = kkt->MakeSolverRHS();
    cost_rhs = MakeBlockVariable(*kkt, c);
    CompiledModel cm(*kkt, cost_rhs);
    RowSpace W = kkt->MakeRowSpace(); setOnes(W);
    kkt->SetScaling(W); kkt->AssembleAndFactor();
    auto r = SolveGeodesicThetaContinuation(cm, W, 100, 1, 1e-8);
    PrintResult("SDP3_ThetaCont", r);
  }

  // === LP with equalities: ThetaContinuation ===
  {
    auto solver = MakeLPEq();
    auto* kkt = solver.kkt();
    Eigen::VectorXd c(4); c << 1, 2, 1, 2;
    auto cost_rhs = BuildCostRHS(*kkt, c);
    CompiledModel cm(*kkt, cost_rhs);
    RowSpace W = kkt->MakeRowSpace(); setOnes(W);
    kkt->SetScaling(W); kkt->AssembleAndFactor();
    auto r = SolveGeodesicThetaContinuation(cm, W, 200, 1, 1e-8);
    PrintResult("LPEq_ThetaCont", r);
  }

  printf("CANARY DONE\n");
}
