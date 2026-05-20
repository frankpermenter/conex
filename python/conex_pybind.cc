// pybind11 bindings for conex Model and Solver.
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/solve_result.h"
#include "conex/common/exp_cone_ops.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/algorithms/geodesic_ipm_helpers.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

namespace py = pybind11;
using namespace conex;

PYBIND11_MODULE(_conex, m) {
  m.doc() = "conex: geodesic interior-point solver for conic optimization";

  // --- SolveResult ---
  py::class_<OptimalitySummary>(m, "OptimalitySummary")
      .def_readonly("dual_residual", &OptimalitySummary::dual_residual)
      .def_readonly("complementarity", &OptimalitySummary::complementarity)
      .def_readonly("min_slack", &OptimalitySummary::min_slack)
      .def_readonly("min_dual", &OptimalitySummary::min_dual);

  py::class_<ConstraintDuals>(m, "ConstraintDuals")
      .def_readonly("lambda_", &ConstraintDuals::lambda)
      .def_readonly("slack", &ConstraintDuals::slack)
      .def_readonly("nu", &ConstraintDuals::nu)
      .def_readonly("eq_residual", &ConstraintDuals::eq_residual)
      .def_readonly("stationarity_gradient",
                    &ConstraintDuals::stationarity_gradient);

  py::class_<SolveResult>(m, "SolveResult")
      .def_readonly("x", &SolveResult::x)
      .def_readonly("objective", &SolveResult::objective)
      .def_readonly("mu", &SolveResult::mu)
      .def_readonly("gap", &SolveResult::gap)
      .def_readonly("iterations", &SolveResult::iterations)
      .def_readonly("factorizations", &SolveResult::factorizations)
      .def_readonly("converged", &SolveResult::converged)
      .def_readonly("optimality", &SolveResult::optimality)
      .def_readonly("duals", &SolveResult::duals);

  // --- Model ---
  py::class_<Model>(m, "Model")
      .def(py::init<>())
      .def("add_linear_constraint",
           [](Model& self, const Eigen::SparseMatrix<double>& A,
              const Eigen::VectorXd& b, const std::vector<int>& vars) {
             self.AddLinearConstraint(A, b, vars);
           },
           py::arg("A"), py::arg("b"), py::arg("vars"),
           "Add Ax + b >= 0 (nonneg cone constraint)")
      .def("add_soc_constraint",
           [](Model& self, const Eigen::SparseMatrix<double>& A,
              const Eigen::VectorXd& b, const std::vector<int>& vars) {
             self.AddSOCConstraint(A, b, vars);
           },
           py::arg("A"), py::arg("b"), py::arg("vars"),
           "Add ||A₁x + b₁|| ≤ A₀x + b₀ (SOC constraint)")
      .def("add_psd_constraint",
           [](Model& self,
              const std::vector<Eigen::SparseMatrix<double>>& A_list,
              const Eigen::SparseMatrix<double>& B,
              const std::vector<int>& vars) {
             self.AddPSDConstraint(A_list, B, vars, false);
           },
           py::arg("A_list"), py::arg("B"), py::arg("vars"),
           "Add Σ A_i x_i + B ≽ 0 (PSD constraint)")
      .def("add_equality_constraint",
           [](Model& self, const Eigen::SparseMatrix<double>& C,
              const Eigen::VectorXd& d, const std::vector<int>& vars) {
             self.AddEqualityConstraint(C, d, vars);
           },
           py::arg("C"), py::arg("d"), py::arg("vars"),
           "Add Cx = d (equality constraint)")
      .def("add_barrier_constraint",
           [](Model& self, const Eigen::SparseMatrix<double>& A,
              const Eigen::VectorXd& b, const std::vector<int>& vars,
              const std::string& cone_type) {
             const EuclideanJordanAlgebra::BarrierConeOperations* ops = nullptr;
             if (cone_type == "exp") {
               ops = &EuclideanJordanAlgebra::expConeOps();
             } else {
               throw std::invalid_argument("Unknown cone type: " + cone_type
                   + ". Supported: 'exp'");
             }
             self.AddBarrierConstraint(A, b, vars, ops);
           },
           py::arg("A"), py::arg("b"), py::arg("vars"),
           py::arg("cone_type") = "exp",
           "Add barrier constraint (A*x + b in cone). cone_type: 'exp'")
      .def("add_quadratic_cost",
           [](Model& self, const Eigen::SparseMatrix<double>& Q,
              const std::vector<int>& vars) {
             self.AddQuadraticCost(Q, vars);
           },
           py::arg("Q"), py::arg("vars"),
           "Add (1/2) x'Qx to the objective")
      .def("set_linear_cost", &Model::SetLinearCost,
           py::arg("c"), "Set c'x as the linear cost")
      .def("num_variables", &Model::num_variables)
      .def("num_constraints", &Model::num_constraints);

  m.def("lift_equalities_to_penalty", &LiftEqualitiesToPenalty,
        py::arg("model"), py::arg("alpha") = 1e6,
        "Lift Cx=d into alpha*|Cx-d|^2 penalty; returns model without equalities");

  // --- Solver ---
  py::class_<Solver>(m, "Solver")
      .def_static("build",
           [](const Model& model, bool use_lu_for_indefinite,
              bool use_lapack_for_indefinite, double penalty_alpha) {
             SolverConfiguration config;
             config.tree.use_lu_for_indefinite = use_lu_for_indefinite;
             config.tree.use_lapack_for_indefinite = use_lapack_for_indefinite;
             config.penalty_alpha = penalty_alpha;
             return Solver::Build(model, config);
           },
           py::arg("model"), py::arg("use_lu_for_indefinite") = false,
           py::arg("use_lapack_for_indefinite") = false,
           py::arg("penalty_alpha") = 0.0,
           "Build solver from model")
      .def_static("build_with_clique_tree",
           [](const Model& model,
              const std::vector<std::vector<int>>& supernodes,
              const std::vector<std::vector<int>>& separators,
              const std::vector<int>& node_to_parent,
              bool use_lu_for_indefinite) {
             CliqueTree ct;
             ct.supernodes = supernodes;
             ct.separators = separators;
             ct.node_to_parent = node_to_parent;
             int nc = static_cast<int>(supernodes.size());
             ct.post_order_position_to_clique.resize(nc);
             for (int i = 0; i < nc; ++i)
               ct.post_order_position_to_clique[i] = i;
             SolverConfiguration config;
             config.tree.use_lu_for_indefinite = use_lu_for_indefinite;
             return Solver::Build(model, ct, config);
           },
           py::arg("model"), py::arg("supernodes"),
           py::arg("separators"), py::arg("node_to_parent"),
           py::arg("use_lu_for_indefinite") = false,
           "Build solver from model with user-provided clique tree")
      .def("clique_tree_info",
           [](Solver& self) {
             auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(
                 const_cast<SymmetricLinearSystemTreeSolver*>(self.tree_solver()));
             if (!ts) return py::dict();
             auto ct = ts->GetCliqueTree();
             py::list supernodes, separators, children_list;
             int ns = ct.supernodes.size();
             // Build children map
             std::vector<std::vector<int>> ch(ns);
             for (int i = 0; i < ns; ++i) {
               int p = ct.node_to_parent[i];
               if (p >= 0) ch[p].push_back(i);
             }
             for (int i = 0; i < ns; ++i) {
               supernodes.append(py::cast(ct.supernodes[i]));
               separators.append(py::cast(ct.separators[i]));
               children_list.append(py::cast(ch[i]));
             }
             py::dict result;
             result["supernodes"] = supernodes;
             result["separators"] = separators;
             result["parent"] = py::cast(ct.node_to_parent);
             result["children"] = children_list;
             result["perm"] = py::cast(std::vector<int>(
                 ts->perm().begin(), ts->perm().end()));
             result["perm_inv"] = py::cast(std::vector<int>(
                 ts->perm_inv().begin(), ts->perm_inv().end()));
             result["num_demotions"] = ts->num_demotions();
             return result;
           },
           "Get clique tree structure")
      .def("try_factor",
           [](Solver& self) -> py::dict {
             // Set W=I, assemble, factor. Return success + failing clique info.
             auto model = self.MakeCompiledModel();
             auto W = model.MakeRowSpace();
             EuclideanJordanAlgebra::setOnes(W);
             model.SetScaling(W);
             bool ok = model.AssembleAndFactor();
             py::dict result;
             result["success"] = ok;
             if (!ok) {
               auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(
                   &model.kkt());
               if (ts) {
                 int fi = ts->last_failed_subsystem();
                 result["failed_clique"] = fi;
                 if (fi >= 0) {
                   auto ct = ts->GetCliqueTree();
                   result["failed_supernodes"] = py::cast(ct.supernodes[fi]);
                   result["failed_separators"] = py::cast(ct.separators[fi]);
                 }
               }
             }
             return result;
           },
           "Try W=I factorization, return failing clique info")
      .def("check_rcond",
           [](Solver& self, double rcond_tol) -> py::dict {
             // Assemble at W=I (no factor), check rcond of each supernode.
             auto model = self.MakeCompiledModel();
             auto W = model.MakeRowSpace();
             EuclideanJordanAlgebra::setOnes(W);
             model.SetScaling(W);
             // Assemble only (recursive post-order, no factoring).
             auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(
                 &model.kkt());
             py::dict result;
             if (!ts) { result["success"] = true; return result; }
             ts->Assemble();
             auto ct = ts->GetCliqueTree();
             int ns = static_cast<int>(ct.supernodes.size());
             for (int k = 0; k < ns; ++k) {
               auto sn_mat = ts->subsystem(k)->supernode_submatrix();
               int nr = sn_mat.rows();
               if (nr <= 1) continue;
               Eigen::MatrixXd full(nr, nr);
               full.triangularView<Eigen::Lower>() = sn_mat;
               full.triangularView<Eigen::StrictlyUpper>() = full.transpose();
               Eigen::JacobiSVD<Eigen::MatrixXd> svd(full);
               double smax = svd.singularValues()(0);
               double smin = svd.singularValues()(nr - 1);
               double rc = (smax > 0) ? smin / smax : 0;
               if (!(rc > rcond_tol)) {
                 result["success"] = false;
                 result["failed_clique"] = k;
                 result["failed_supernodes"] = py::cast(ct.supernodes[k]);
                 result["failed_separators"] = py::cast(ct.separators[k]);
                 result["rcond"] = rc;
                 return result;
               }
             }
             result["success"] = true;
             return result;
           },
           py::arg("rcond_tol") = 1e-12,
           "Check rcond of assembled supernodes at W=I")
      .def("solve_geodesic_lp",
           [](Solver& self, double tol, int max_iter) {
             return self.Solve(GeodesicLP{tol, max_iter});
           },
           py::arg("tol") = 1e-8, py::arg("max_iter") = 30,
           "Solve with geodesic LP (line-search + center)")
      .def("solve_theta_continuation",
           [](Solver& self, double tol, int max_iter, int max_centering) {
             return self.Solve(ThetaContinuation{tol, max_iter, max_centering});
           },
           py::arg("tol") = 1e-8, py::arg("max_iter") = 500,
           py::arg("max_centering") = 1,
           "Solve with theta-continuation")
      .def("solve_theta_continuation_r",
           [](Solver& self, double tol, int max_iter) {
             return self.Solve(ThetaContinuationR{tol, max_iter});
           },
           py::arg("tol") = 1e-8, py::arg("max_iter") = 500,
           "Solve with theta-continuation + r-updates")
      .def("solve_hybrid_r",
           [](Solver& self, double tol, int max_iter) {
             return self.Solve(HybridR{tol, max_iter});
           },
           py::arg("tol") = 1e-8, py::arg("max_iter") = 500,
           "Solve with hybrid r-update algorithm")
      .def("solve_barrier_lp",
           [](Solver& self, double tol, int max_iter) {
             return self.Solve(GeodesicBarrierLP{tol, max_iter});
           },
           py::arg("tol") = 1e-8, py::arg("max_iter") = 30,
           "Solve with barrier LP (z-space, works for all cones)")
      .def("solve_barrier_theta_continuation",
           [](Solver& self, double tol, int max_iter, int max_centering) {
             return self.Solve(GeodesicBarrierThetaContinuation{
                 tol, max_iter, max_centering});
           },
           py::arg("tol") = 1e-8, py::arg("max_iter") = 500,
           py::arg("max_centering") = 1,
           "Solve with barrier theta-continuation (z-space)")
      // Solve and expand x to original variable space (for CVXPY).
      // Skips ComputeOptimality which crashes on barrier cones.
      .def("solve_raw",
           [](Solver& self, const std::string& algo,
              double tol, int max_iter, int max_centering) {
             auto model = self.MakeCompiledModel();
             GeodesicResult raw;
             if (algo == "barrier_theta_cont")
               raw = GeodesicBarrierThetaContinuation{tol, max_iter, max_centering}.Run(model);
             else if (algo == "theta_cont")
               raw = ThetaContinuation{tol, max_iter, max_centering}.Run(model);
             else if (algo == "barrier_lp")
               raw = GeodesicBarrierLP{tol, max_iter}.Run(model);
             else if (algo == "geodesic_lp")
               raw = GeodesicLP{tol, max_iter, max_centering}.Run(model);
             else if (algo == "theta_cont_r")
               raw = ThetaContinuationR{tol, max_iter}.Run(model);
             else if (algo == "hybrid_r")
               raw = HybridR{tol, max_iter}.Run(model);
             else if (algo == "hsde")
               raw = GeodesicHSDE{tol, max_iter, max_centering}.Run(model);
             else if (algo == "direct_solve") {
               // Direct linear solve using cost + equality RHS.
               auto duality_cost = MakeDualityCost(model);
               int nr = model.number_of_variables();
               std::vector<double> rhs_vec(nr);
               Eigen::Map<Eigen::VectorXd> rhs_map(rhs_vec.data(), nr);
               duality_cost.supernodes->GatherInto(rhs_map);
               raw = DirectSolve{rhs_vec}.Run(model);
             } else
               throw std::invalid_argument("Unknown algorithm: " + algo);
             // Expand x to original variable space.
             Eigen::Map<const Eigen::VectorXd> raw_x(raw.x.data(), raw.x.size());
             Eigen::VectorXd x_expanded = self.ExpandSolution(raw_x);
             py::dict result;
             result["x"] = py::array_t<double>(x_expanded.size(), x_expanded.data());
             result["converged"] = raw.mu < tol;
             result["iterations"] = raw.iterations;
             result["mu"] = raw.mu;
             result["gap"] = raw.complementarity;
             return result;
           },
           py::arg("algo"), py::arg("tol") = 1e-8,
           py::arg("max_iter") = 500, py::arg("max_centering") = 1,
           "Solve and return x in original variable space (for CVXPY)")
      // Full solve returning SolveResult with duals.
      .def("solve",
           [](Solver& self, const std::string& algo,
              double tol, int max_iter, int max_centering) -> SolveResult {
             if (algo == "theta_cont")
               return self.Solve(ThetaContinuation{tol, max_iter, max_centering});
             else if (algo == "theta_cont_r")
               return self.Solve(ThetaContinuationR{tol, max_iter});
             else if (algo == "hybrid_r")
               return self.Solve(HybridR{tol, max_iter});
             else if (algo == "hsde")
               return self.Solve(GeodesicHSDE{tol, max_iter, max_centering});
             else if (algo == "geodesic_lp")
               return self.Solve(GeodesicLP{tol, max_iter, max_centering});
             else if (algo == "barrier_theta_cont")
               return self.Solve(GeodesicBarrierThetaContinuation{tol, max_iter, max_centering});
             else
               throw std::invalid_argument("Unknown algorithm: " + algo);
           },
           py::arg("algo") = "theta_cont", py::arg("tol") = 1e-8,
           py::arg("max_iter") = 500, py::arg("max_centering") = 1,
           "Solve and return full SolveResult with duals");
}
