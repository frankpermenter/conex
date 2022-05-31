#include <fstream>

#include "conex/cone_program.h"
#include "conex/equality_constraint.h"
#include "conex/linear_constraint.h"
#include "conex/quadratic_cone_constraint.h"
#include "conex/serialize.h"
#include <Eigen/Dense>

namespace conex {

inline void AddQuadraticCostEpigraph(conex::Program* conex_prog,
                                     const Eigen::MatrixXd& Qi,
                                     const std::vector<int>& z, int epigraph) {
  // Set inner-product matrix Q of Lorentz cone L.
  Eigen::MatrixXd Q(z.size() + 1, z.size() + 1);
  Q.setZero();
  Q(0, 0) = 1;
  Q.bottomRightCorner(z.size(), z.size()) = Qi;

  // Build (A, b) satisfying b - A(x, t) \in L <=> t >= 1/2 x^T Q x.
  int num_vars = z.size();
  Eigen::MatrixXd Ai(num_vars + 2, num_vars + 1);
  Eigen::MatrixXd b(num_vars + 2, 1);
  Ai.setZero();
  b.setZero();
  Ai.topRightCorner(2, 1) << -0.5, -0.5;
  Ai.bottomLeftCorner(z.size(), z.size()) =
      Eigen::MatrixXd::Identity(num_vars, num_vars);
  b(0) = 1;
  b(1) = -1;

  // (.5 t+1)^2 >= (.5t-1)^2 + x^T Q x.
  // .25 t^2 + t + 1  >= .25 t^2 - t + 1 + x^T Q x
  // => 2t >= x^T Q x.
  auto z_indices = z;
  z_indices.push_back(epigraph);
  conex_prog->AddConstraint(conex::QuadraticConstraint(Q, Ai, b), z_indices);
}
SolverConfiguration DefaultTestConfiguration() { return SolverConfiguration(); }

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Creates KKT matrix of the form:
//  G << 1, 1,  1,
//       1, 1, -1,
//       1, -1, 0;
// which we fail to factor.
void EqualityConstraintFailingLDLT() {
  int num_vars = 2;
  int num_equalities = 1;
  int num_inequalities = 1;

  MatrixXd A = MatrixXd::Random(num_inequalities, num_vars);
  A << 1, 1;
  MatrixXd C(num_inequalities, 1);
  C.setConstant(1);
  LinearConstraint linear_inequality{A, C};

  MatrixXd eq = MatrixXd::Random(num_equalities, num_vars);
  MatrixXd eq_affine(num_equalities, 1);
  eq_affine.setZero();
  eq << 1, -1;

  Program prog(num_vars);
  prog.AddConstraint(EqualityConstraints{eq, eq_affine}, {0, 1});
  prog.AddConstraint(linear_inequality);

  VectorXd linear_cost(num_vars);
  linear_cost = A.transpose() * C;

  VectorXd solution(num_vars);
  Solve(linear_cost, prog, conex::SolverConfiguration(), solution.data());

  DUMP(eq * solution - eq_affine);
  DUMP(solution);
}

void IncrementSubVector(const std::vector<int>& cliques, const VectorXd& x,
                        VectorXd* y) {
  int i = 0;
  for (auto e : cliques) {
    (*y)(e) += x(i++);
  }
}

void EqualityConstraintForceEqualityConstraintsToLeafNodes2() {
  int num_vars = 6;

  std::vector<std::vector<int>> cliques{
      {0, 3, 4}, {1, 2, 5}, {0, 1, 2, 3}}; /* equality*/

  MatrixXd B1(1, 2);
  B1 << 1, -1;

  MatrixXd A1(2, 3);
  A1 << 1, 2, 3, -1, -2, -3;
  VectorXd b1 = A1.transpose() * VectorXd::Constant(3, 1);

  MatrixXd A2(2, 3);
  A2 << 3, 1, 5, -3, -4, -5;
  VectorXd b2 = A2.transpose() * VectorXd::Constant(3, 1);

  MatrixXd B2(2, 4);
  B2 << 0, 0, 1, -1, 1, -1, 0, 0;
  VectorXd linear_cost(num_vars);

  linear_cost.setConstant(0);
  IncrementSubVector(cliques.at(0), b1, &linear_cost);
  IncrementSubVector(cliques.at(1), b2, &linear_cost);

  VectorXd solution(num_vars);

  Program prog(num_vars);
  prog.AddConstraint(LinearConstraint{A1, VectorXd::Constant(2, 100)},
                     cliques.at(0));
  prog.AddConstraint(LinearConstraint{A2, VectorXd::Constant(2, 100)},
                     cliques.at(1));
  prog.AddConstraint(EqualityConstraints{B2, VectorXd::Zero(2)}, cliques.at(2));

  conex::SolverConfiguration config = DefaultTestConfiguration();
  config.max_iterations = 1;
  Solve(linear_cost, prog, config, solution.data());
}

void EqualityConstraintForceEqualityConstraintsToLeafNodes(
    bool failure_from_fill_in) {
  int num_vars = 6;

  std::vector<std::vector<int>> cliques;
  if (failure_from_fill_in) {
    /* Cliques an internal clique tree with nodes:
     *
     *   {0 1 equality_dual_var_1}
     *   {0 3 4}
     *   {1 2 5}
     *   {2 3 equality_dual_var_2}
     *
     *        {0 1 equality_dual_var_1} + {3}
     *        /                 \
     *   {0 3 4} + {3}      {1 2 5} + {3}
     *                            \
     *                 {2 3 equality_dual_var_2}
     *
     * Here the +{3} indicates fill-in. We also see that equality_dual_var_2 is
     * the only supernode of {2 3 equality_dual_var_2}, which causes the LDLT
     * factorization to fail.
     *
     *  */
    cliques = std::vector<std::vector<int>>{{0, 1}, /* equality*/
                                            {0, 3, 4},
                                            {1, 2, 5},
                                            {2, 3}}; /* equality*/
  } else {
    /* In this simpler example, the same issue arises, even without fill-in.*/
    cliques = std::vector<std::vector<int>>{{0, 1}, /* equality*/
                                            {0, 2, 3},
                                            {3, 4, 5},
                                            {4, 5}}; /* equality*/
  }

  MatrixXd B1(1, 2);
  B1 << 1, -1;

  MatrixXd A1(2, 3);
  A1 << 1, 2, 3, -1, -2, -3;
  VectorXd b1 = A1.transpose() * VectorXd::Constant(2, 1);

  MatrixXd A2(2, 3);
  A2 << 3, 1, 5, -3, -4, -5;
  VectorXd b2 = A2.transpose() * VectorXd::Constant(2, 1);

  MatrixXd B2(1, 2);
  B2 << 1, -1;
  VectorXd linear_cost(num_vars);

  linear_cost.setConstant(0);
  IncrementSubVector(cliques.at(1), b1, &linear_cost);
  IncrementSubVector(cliques.at(2), b2, &linear_cost);

  VectorXd solution(num_vars);

  Program prog(num_vars);
  prog.AddConstraint(EqualityConstraints{B1, VectorXd::Zero(1)}, cliques.at(0));
  prog.AddConstraint(LinearConstraint{A1, VectorXd::Constant(2, 100)},
                     cliques.at(1));
  prog.AddConstraint(LinearConstraint{A2, VectorXd::Constant(2, 100)},
                     cliques.at(2));
  prog.AddConstraint(EqualityConstraints{B2, VectorXd::Zero(1)}, cliques.at(3));

  conex::SolverConfiguration config = DefaultTestConfiguration();
  config.max_iterations = 2;
  Solve(linear_cost, prog, config, solution.data());
}

/* Illustrate problem: we must eliminate dual variables before we eliminate
 * variables with no quadratic term. The following generates clique tree:
 *
 * {0, 2, 3}
 *     |
 * {3, 4, 5}
 *     |
 * {0, 1, 4, 5, dual, dual}.
 *
 * To avoid LDLT failure, the last clique must be permuted as:
 *
 * {0, dual, 1, 4, 5, dual}
 *
 * */
void EqualityConstraintsNoQuadraticPenalty() {
  int num_vars = 6;

  std::vector<std::vector<int>> cliques{{0, 2, 3}, {3, 4, 5}};
  std::vector<int> clique_eq{0, 1, 4, 5};

  MatrixXd B1(1, 2);
  B1 << 1, -1;

  MatrixXd A1(2, 3);
  A1 << 1, 2, 3, -1, -2, -3;
  VectorXd b1 = A1.transpose() * VectorXd::Constant(2, 1);

  MatrixXd A2(2, 3);
  A2 << 3, 1, 5, -3, -4, -5;
  VectorXd b2 = A2.transpose() * VectorXd::Constant(2, 1);
  VectorXd linear_cost(num_vars);
  linear_cost.setConstant(0);
  IncrementSubVector(cliques.at(0), b1, &linear_cost);
  IncrementSubVector(cliques.at(1), b2, &linear_cost);

  VectorXd solution(num_vars);

  Program prog(num_vars);
  MatrixXd B(2, 4);
  B << 1, -1, 0, 0, 0, 0, 1, -1;
  prog.AddConstraint(EqualityConstraints{B, VectorXd::Zero(2)}, clique_eq);
  prog.AddConstraint(LinearConstraint{A1, VectorXd::Constant(2, 100)},
                     cliques.at(0));
  prog.AddConstraint(LinearConstraint{A2, VectorXd::Constant(2, 100)},
                     cliques.at(1));

  conex::SolverConfiguration config = DefaultTestConfiguration();
  config.max_iterations = 2;
  Solve(linear_cost, prog, config, solution.data());
}

// The centering parameter moves around.
void DoBadInitialization(bool fail) {
  int num_vars = 5;
  int num_inequalities = num_vars + 10;
  int num_equalities = 1;

  MatrixXd A = MatrixXd::Random(num_inequalities, num_vars);
  MatrixXd C(num_inequalities, 1);

  VectorXd optimal_slack(num_inequalities);
  VectorXd optimal_dual(num_inequalities);
  VectorXd optimal_y(num_vars);

  optimal_slack.setConstant(1);
  optimal_dual.setConstant(1);
  int m = num_inequalities * .5;
  optimal_slack.topRows(m).setConstant(1e-3);
  optimal_dual.bottomRows(num_inequalities - m).setConstant(1e-3);

  optimal_y = Eigen::MatrixXd::Random(num_vars, 1);

  C = optimal_slack + A * optimal_y;

  LinearConstraint linear_inequality{A, C};

  Program prog(num_vars);
  MatrixXd eq = MatrixXd::Zero(num_equalities, num_vars);
  Eigen::MatrixXd Bi(1, 3);
  Bi << 1, 2, 3;
  for (int i = 0; i < num_equalities; i++) {
    std::vector<int> vars{0, i + 1, num_vars - 1};
    for (size_t j = 0; j < vars.size(); j++) {
      eq(i, vars.at(j)) = Bi(0, j);
    }
    // prog.AddConstraint(EqualityConstraints{Bi, eq.row(i) * optimal_y    },
    // vars);
  }

  MatrixXd eq_affine(num_equalities, 1);
  eq_affine = eq * optimal_y;

  prog.AddConstraint(EqualityConstraints{eq, eq_affine});
  prog.AddConstraint(linear_inequality);

  VectorXd linear_cost(num_vars);
  linear_cost = A.transpose() * optimal_dual;

  VectorXd solution(num_vars);
  auto config = conex::SolverConfiguration();
  config.final_centering_steps = 10;

  // TODO(FrankPermenter): Understand occasional poor convergence when lowered.
  config.initial_centering_steps_coldstart = 0;
  if (!fail) {
    config.initial_centering_steps_coldstart = 10;
  }
  config.max_iterations = 40;
  config.divergence_upper_bound = .5;
  Solve(linear_cost, prog, config, solution.data());
}

class MPCFailingLDLT {
 public:
  template <typename T>
  using vector = std::vector<T>;
  static constexpr int T = 3;
  static constexpr int nu = 1;
  static constexpr int nx = 2;

  auto InputVars(int i) {
    int offset = T * nx + i * nu;
    // x, u x u
    vector<int> y;
    for (int i = 0; i < nu; i++) {
      y.push_back(i + offset);
    }
    return y;
  }

  auto StateVars(int i) {
    assert(i >= 1);
    int offset = (i - 1) * nx;
    vector<int> y;
    for (int i = 0; i < nx; i++) {
      y.push_back(i + offset);
    }
    return y;
  }

  auto StageVars(int i, bool next_state) {
    vector<int> y;
    if (i > 0) {
      for (const auto& c : StateVars(i)) {
        y.push_back(c);
      }
    }

    for (const auto& c : InputVars(i)) {
      y.push_back(c);
    }

    for (const auto& c : StateVars(i + 1)) {
      y.push_back(c);
    }
    return y;
  }

  auto DynamicsConstraint(const MatrixXd& Ai, const MatrixXd& Bi, int i) {
    if (i > 0) {
      MatrixXd A(nx, 2 * nx + nu);
      A << Ai, Bi, -MatrixXd::Identity(nx, nx);
      return A;
    } else {
      MatrixXd A(nx, nx + nu);
      A << Bi, -MatrixXd::Identity(nx, nx);
      return A;
    }
  }

  void Run(bool fail) {
    int num_vars = T * (nu + nx + 2);
    int epigraph_start = T * (nu + nx);

    MatrixXd Ai = MatrixXd::Random(nx, nx);
    MatrixXd Bi = MatrixXd::Random(nx, nu);
    MatrixXd f = MatrixXd::Random(nx, 1);
    MatrixXd Hxu = MatrixXd::Random(2, nu);
    MatrixXd gxu = MatrixXd::Random(2, 1);

    conex::Program prog(num_vars);
    for (int i = 0; i < T; i++) {
      MatrixXd M(nx, nu + 2 * nx);
      prog.AddConstraint(
          conex::EqualityConstraints{DynamicsConstraint(Ai, Bi, i), f},
          StageVars(i, true));

      if (fail) {
        if (i > 0 && i < T - 1) {
          prog.AddConstraint(conex::LinearConstraint{Hxu, gxu},
                             StageVars(i, false));
        }
      }

      AddQuadraticCostEpigraph(&prog, MatrixXd::Identity(nu, nu), InputVars(i),
                               epigraph_start++);
      AddQuadraticCostEpigraph(&prog, MatrixXd::Identity(nx, nx),
                               StateVars(i + 1), epigraph_start++);
    }

    Eigen::VectorXd var(num_vars);
    VectorXd linear_cost(num_vars);
    linear_cost.setConstant(-1);
    auto config = conex::SolverConfiguration();
    config.inv_sqrt_mu_max = 1e4;
    config.final_centering_steps = 10;
    config.max_iterations = 50;
    conex::Solve(linear_cost, prog, config, var.data());
    DUMP(var);
  }
};

void LPFailSlater(int number_of_implicit_equations) {
  double distance_to_infeasible = 0;
  SolverConfiguration config = DefaultTestConfiguration();
  config.prepare_dual_variables = true;
  config.inv_sqrt_mu_max = 100;
  config.final_centering_tolerance = 1;
  config.infeasibility_threshold = 2000000;
  config.final_centering_steps = 5;

  int m = 10;
  int n1 = number_of_implicit_equations;
  int n2 = 8;
  int n = 2 * n1 + n2;
  Eigen::MatrixXd yref = DenseMatrix::Random(m, 1);

  DenseMatrix A1 = DenseMatrix::Random(n1, m);
  DenseMatrix C1 = A1 * yref;
  DenseMatrix A2 = DenseMatrix::Random(n2, m);
  DenseMatrix C2 = A2 * yref;
  C2.array() += 2;

  DenseMatrix A = DenseMatrix::Random(n, m);
  DenseMatrix C = DenseMatrix::Random(n, 1);
  A << A1, -A1, A2;

  DenseMatrix offset(n1, 1);
  offset.setConstant(distance_to_infeasible);
  C << C1, -(C1 - offset), C2;

  LinearConstraint _constraint{A, C};

  Program prog(m);
  prog.SetNumberOfVariables(m);
  prog.AddConstraint(_constraint);

  VectorXd b(2);
  VectorXd xref = VectorXd::Random(n);
  xref = xref.array().abs();
  b = A.transpose() * xref;

  DenseMatrix y(m, 1);
  Solve(b, prog, config, y.data());
}
// Builds the program with x1 = x2 constraint
// with no quadratic penalty or inequality on x2 if
// regularization_used = true. This leads to regularization
// in the LDLT factorization.
VectorXd SimpleBadLDLTHelper(const VectorXd& linear_cost,
                             bool require_regularation) {
  Program prog(2);
  DenseMatrix A(1, 2);
  A << 1, 0;
  DenseMatrix c(1, 1);
  c << 1;
  DenseMatrix B(1, 2);
  B << 1, -1;
  DenseMatrix f(1, 1);
  f << 0;
  DenseMatrix Q(2, 2);
  Q << 1, 0, 0, 1;
  if (require_regularation) {
    Q(1, 1) = 0;
  }

  prog.AddConstraint(LinearConstraint(A, c));
  prog.AddConstraint(EqualityConstraints(B, f));
  prog.AddQuadraticCost(Q);
  prog.AddLinearCost(linear_cost);
  VectorXd y(2);
  SolverConfiguration config;
  config.enable_rescaling = 0;
  config.enable_line_search = 1;
  config.verbose = 1;
  Solve(prog, config, y.data());
  return y;
}
// Shows effect of LDLT regularization on solution.
void SimpleBadLDLT() {
  double value = 100;
  VectorXd linear_cost(2);
  // Solve problem with x1 = x2 constraint, moving the
  // linear cost from variable x2 to variable x1.
  for (int i = 0; i < 2; i++) {
    linear_cost << 0, -value;
    VectorXd sol1 =
        SimpleBadLDLTHelper(linear_cost, /*require regularation*/ i == 1);
    linear_cost << -value, 0;
    VectorXd sol2 =
        SimpleBadLDLTHelper(linear_cost, /*require regularation*/ i == 1);
    DUMP(sol1 - sol2);
  }
}

void GraphOfConvexSets() {
  std::ifstream t("conex/test/graph_of_convex_sets_fails_slater.json");
  std::stringstream buffer;
  ConstraintManager c;
  buffer << t.rdbuf();
  JsonObject prog = ParseJsonString(buffer.str());
  DUMP(prog["constraints"]["0"]["id"].value());
  DUMP(prog["num_constraints"].value());
  DeserializeConeProgram(prog, &c);
  Eigen::VectorXd y(c.GetNumberOfVariables());
  DUMP(c.GetLinearCostVector());
  Program program(std::move(c));

  SolverConfiguration config;
  config.initial_centering_steps_coldstart = 0;
  config.prepare_dual_variables = 0;
  config.infeasibility_threshold = 9e6;
  config.divergence_upper_bound = 100;
  config.final_centering_steps = 5;
  config.final_centering_tolerance = 1;
  config.max_iterations = 200;
  config.verbose = true;
  config.dinf_upper_bound = 1.1;
  // config.kkt_solver = conex::CONEX_KKT_SOLVER_SPARSE_QR;
  config.kkt_solver = conex::CONEX_KKT_SOLVER_SUPERNODAL_QR;
  // config.enable_line_search = !psd_constraints_found;
  config.enable_line_search = 0;
  config.enable_rescaling = !config.enable_line_search;
  config.inv_sqrt_mu_max = 2000;
  config.maximum_mu = 100;
  config.kkt_error_tolerance = 4;

  Solve(program, config, y.data());
}

}  // namespace conex

int main() {
  conex::GraphOfConvexSets();
  conex::SimpleBadLDLT();
  conex::EqualityConstraintForceEqualityConstraintsToLeafNodes(
      false /*fill-in induced failure*/);
  conex::EqualityConstraintsNoQuadraticPenalty();
  conex::EqualityConstraintForceEqualityConstraintsToLeafNodes(
      true /*fill-in induced failure*/);
  conex::EqualityConstraintFailingLDLT();
  conex::MPCFailingLDLT().Run(true /*trigger fail*/);
  srand(0);
  for (int i = 0; i < 5; i++) {
    conex::DoBadInitialization(true /*trigger fail*/);
  }
  // Triggers factorization failure
  srand(0);
  conex::LPFailSlater(1 /*num implicit eqs*/);
  // Triggers bad convergence
  srand(0);
  conex::LPFailSlater(2 /*num implicit eqs*/);
}
