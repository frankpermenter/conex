// pybind11 bindings for conex Model and Solver.
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/solve_result.h"
#include "conex/algorithms/solve_strategies.h"

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

  py::class_<SolveResult>(m, "SolveResult")
      .def_readonly("x", &SolveResult::x)
      .def_readonly("objective", &SolveResult::objective)
      .def_readonly("mu", &SolveResult::mu)
      .def_readonly("gap", &SolveResult::gap)
      .def_readonly("iterations", &SolveResult::iterations)
      .def_readonly("factorizations", &SolveResult::factorizations)
      .def_readonly("converged", &SolveResult::converged)
      .def_readonly("optimality", &SolveResult::optimality);

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

  // --- Solver ---
  py::class_<Solver>(m, "Solver")
      .def_static("build",
           [](const Model& model) { return Solver::Build(model); },
           py::arg("model"), "Build solver from model")
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
           "Solve with barrier theta-continuation (z-space)");
}
