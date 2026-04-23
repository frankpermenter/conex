#include <cstdint>
#include <stdexcept>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "../../conex.h"

namespace py = pybind11;

namespace {

void* PtrFromPy(std::uintptr_t p) { return reinterpret_cast<void*>(p); }
std::uintptr_t PtrToPy(void* p) { return reinterpret_cast<std::uintptr_t>(p); }

class IntPtr {
 public:
  IntPtr(int value = 0) : value_(value) {}
  int value() const { return value_; }
  void set(int v) { value_ = v; }
  int* ptr() { return &value_; }

 private:
  int value_;
};

template <int N>
py::array_t<double, py::array::forcecast> AsArray(const py::array& a) {
  auto out = py::array_t<double, py::array::forcecast>(a);
  if (out.ndim() != N) {
    throw std::runtime_error("Expected array with " + std::to_string(N) +
                             " dimensions");
  }
  return out;
}

py::array_t<double, py::array::f_style | py::array::forcecast> AsFArray2(
    const py::array& a) {
  auto out =
      py::array_t<double, py::array::f_style | py::array::forcecast>(a);
  if (out.ndim() != 2) {
    throw std::runtime_error("Expected 2D array");
  }
  return out;
}

py::array_t<double, py::array::f_style | py::array::forcecast> AsFArray3(
    const py::array& a) {
  auto out =
      py::array_t<double, py::array::f_style | py::array::forcecast>(a);
  if (out.ndim() != 3) {
    throw std::runtime_error("Expected 3D array");
  }
  return out;
}

py::array_t<double, py::array::c_style | py::array::forcecast> AsMutable1D(
    py::array& a) {
  auto out = py::array_t<double, py::array::c_style | py::array::forcecast>(a);
  if (out.ndim() != 1) {
    throw std::runtime_error("Expected 1D array");
  }
  return out;
}

py::array_t<double, py::array::c_style | py::array::forcecast> AsMutable2D(
    py::array& a) {
  auto out = py::array_t<double, py::array::c_style | py::array::forcecast>(a);
  if (out.ndim() != 2) {
    throw std::runtime_error("Expected 2D array");
  }
  return out;
}

}  // namespace

PYBIND11_MODULE(_conex, m) {
  py::class_<IntPtr>(m, "intp")
      .def(py::init<int>(), py::arg("value") = 0)
      .def("value", &IntPtr::value)
      .def("set", &IntPtr::set);

  py::class_<CONEX_SolverConfiguration>(m, "CONEX_SolverConfiguration")
      .def(py::init<>())
      .def_readwrite("prepare_dual_variables",
                     &CONEX_SolverConfiguration::prepare_dual_variables)
      .def_readwrite("initialization_mode",
                     &CONEX_SolverConfiguration::initialization_mode)
      .def_readwrite("inv_sqrt_mu_max",
                     &CONEX_SolverConfiguration::inv_sqrt_mu_max)
      .def_readwrite("minimum_mu", &CONEX_SolverConfiguration::minimum_mu)
      .def_readwrite("maximum_mu", &CONEX_SolverConfiguration::maximum_mu)
      .def_readwrite("divergence_upper_bound",
                     &CONEX_SolverConfiguration::divergence_upper_bound)
      .def_readwrite("enable_line_search",
                     &CONEX_SolverConfiguration::enable_line_search)
      .def_readwrite("dinf_upper_bound",
                     &CONEX_SolverConfiguration::dinf_upper_bound)
      .def_readwrite("final_centering_steps",
                     &CONEX_SolverConfiguration::final_centering_steps)
      .def_readwrite("final_centering_tolerance",
                     &CONEX_SolverConfiguration::final_centering_tolerance)
      .def_readwrite("initial_centering_steps_warmstart",
                     &CONEX_SolverConfiguration::initial_centering_steps_warmstart)
      .def_readwrite("initial_centering_steps_coldstart",
                     &CONEX_SolverConfiguration::initial_centering_steps_coldstart)
      .def_readwrite("warmstart_abort_threshold",
                     &CONEX_SolverConfiguration::warmstart_abort_threshold)
      .def_readwrite("max_iterations", &CONEX_SolverConfiguration::max_iterations)
      .def_readwrite("iterative_refinement_iterations",
                     &CONEX_SolverConfiguration::iterative_refinement_iterations)
      .def_readwrite("infeasibility_threshold",
                     &CONEX_SolverConfiguration::infeasibility_threshold)
      .def_readwrite("kkt_error_tolerance",
                     &CONEX_SolverConfiguration::kkt_error_tolerance)
      .def_readwrite("enable_rescaling",
                     &CONEX_SolverConfiguration::enable_rescaling)
      .def_readwrite("enable_scale_correction",
                     &CONEX_SolverConfiguration::enable_scale_correction)
      .def_readwrite("kkt_solver", &CONEX_SolverConfiguration::kkt_solver)
      .def_readwrite("verbosity", &CONEX_SolverConfiguration::verbosity);

  py::class_<CONEX_IterationStats>(m, "CONEX_IterationStats")
      .def(py::init<>())
      .def_readwrite("mu", &CONEX_IterationStats::mu)
      .def_readwrite("iteration_number", &CONEX_IterationStats::iteration_number);

  py::class_<CONEX_SolutionStats>(m, "CONEX_SolutionStats")
      .def(py::init<>())
      .def_readwrite("iterations", &CONEX_SolutionStats::iterations)
      .def_readwrite("duality_gap", &CONEX_SolutionStats::duality_gap);

  m.def("CONEX_CreateConeProgram",
        []() { return PtrToPy(CONEX_CreateConeProgram()); });
  m.def("CONEX_DeleteConeProgram",
        [](std::uintptr_t p) { CONEX_DeleteConeProgram(PtrFromPy(p)); });
  m.def("CONEX_SetNumberOfVariables", [](std::uintptr_t p, int n) {
    return CONEX_SetNumberOfVariables(PtrFromPy(p), n);
  });
  m.def("CONEX_SetDefaultOptions",
        [](CONEX_SolverConfiguration& cfg) { CONEX_SetDefaultOptions(&cfg); });

  m.def("CONEX_AddDenseLinearConstraint",
        [](std::uintptr_t p, const py::array& A, const py::array& c) {
          auto a = AsFArray2(A);
          auto cv = AsArray<1>(c);
          auto ai = a.request();
          auto ci = cv.request();
          return CONEX_AddDenseLinearConstraint(
              PtrFromPy(p), static_cast<double*>(ai.ptr), static_cast<int>(ai.shape[0]),
              static_cast<int>(ai.shape[1]), static_cast<double*>(ci.ptr),
              static_cast<int>(ci.shape[0]));
        });

  m.def("CONEX_AddLinearInequalities",
        [](std::uintptr_t p, const py::array& A, const py::array& lb,
           const py::array& ub) {
          auto a = AsFArray2(A);
          auto l = AsArray<1>(lb);
          auto u = AsArray<1>(ub);
          auto ai = a.request();
          auto li = l.request();
          auto ui = u.request();
          return CONEX_AddLinearInequalities(
              PtrFromPy(p), static_cast<double*>(ai.ptr), static_cast<int>(ai.shape[0]),
              static_cast<int>(ai.shape[1]), static_cast<double*>(li.ptr),
              static_cast<int>(li.shape[0]), static_cast<double*>(ui.ptr),
              static_cast<int>(ui.shape[0]));
        });

  m.def("CONEX_AddQuadraticCost", [](std::uintptr_t p, const py::array& A) {
    auto a = AsFArray2(A);
    auto ai = a.request();
    return CONEX_AddQuadraticCost(PtrFromPy(p), static_cast<double*>(ai.ptr),
                                  static_cast<int>(ai.shape[0]),
                                  static_cast<int>(ai.shape[1]));
  });

  m.def("CONEX_AddDenseLMIConstraint",
        [](std::uintptr_t p, const py::array& Aarray, const py::array& cmat) {
          auto a = AsFArray3(Aarray);
          auto c = AsFArray2(cmat);
          auto ai = a.request();
          auto ci = c.request();
          return CONEX_AddDenseLMIConstraint(
              PtrFromPy(p), static_cast<double*>(ai.ptr), static_cast<int>(ai.shape[0]),
              static_cast<int>(ai.shape[1]), static_cast<int>(ai.shape[2]),
              static_cast<double*>(ci.ptr), static_cast<int>(ci.shape[0]),
              static_cast<int>(ci.shape[1]));
        });

  m.def("CONEX_AddSparseLMIConstraint",
        [](std::uintptr_t p, const py::array& Aarray, const py::array& cmat,
           const py::array& vars) {
          auto a = AsFArray3(Aarray);
          auto c = AsFArray2(cmat);
          auto v = py::array_t<long, py::array::forcecast>(vars);
          if (v.ndim() != 1) throw std::runtime_error("Expected 1D vars array");
          auto ai = a.request();
          auto ci = c.request();
          auto vi = v.request();
          return CONEX_AddSparseLMIConstraint(
              PtrFromPy(p), static_cast<double*>(ai.ptr), static_cast<int>(ai.shape[0]),
              static_cast<int>(ai.shape[1]), static_cast<int>(ai.shape[2]),
              static_cast<double*>(ci.ptr), static_cast<int>(ci.shape[0]),
              static_cast<int>(ci.shape[1]), static_cast<long*>(vi.ptr),
              static_cast<int>(vi.shape[0]));
        });

  m.def("CONEX_AddLinearCost", [](std::uintptr_t p, const py::array& b) {
    auto bv = AsArray<1>(b);
    auto bi = bv.request();
    return CONEX_AddLinearCost(PtrFromPy(p), static_cast<double*>(bi.ptr),
                               static_cast<int>(bi.shape[0]));
  });

  m.def("CONEX_Solve",
        [](std::uintptr_t p, const CONEX_SolverConfiguration& cfg,
           py::array y) {
          auto yy = AsMutable1D(y);
          auto yi = yy.request();
          py::gil_scoped_release release;
          return CONEX_Solve(PtrFromPy(p), &cfg, static_cast<double*>(yi.ptr),
                             static_cast<int>(yi.shape[0]));
        });

  m.def("CONEX_Maximize",
        [](std::uintptr_t p, const py::array& b, const CONEX_SolverConfiguration& cfg,
           py::array y) {
          auto bv = AsArray<1>(b);
          auto yy = AsMutable1D(y);
          auto bi = bv.request();
          auto yi = yy.request();
          py::gil_scoped_release release;
          return CONEX_Maximize(PtrFromPy(p), static_cast<double*>(bi.ptr),
                                static_cast<int>(bi.shape[0]), &cfg,
                                static_cast<double*>(yi.ptr),
                                static_cast<int>(yi.shape[0]));
        });

  m.def("CONEX_GetDualVariable", [](std::uintptr_t p, int i, py::array x) {
    auto xx = AsMutable2D(x);
    auto xi = xx.request();
    CONEX_GetDualVariable(PtrFromPy(p), i, static_cast<double*>(xi.ptr),
                          static_cast<int>(xi.shape[0]),
                          static_cast<int>(xi.shape[1]));
  });

  m.def("CONEX_GetIterationStats",
        [](std::uintptr_t p, CONEX_IterationStats& stats, int iter_num) {
          CONEX_GetIterationStats(PtrFromPy(p), &stats, iter_num);
        });

  m.def("CONEX_NewLinearMatrixInequality",
        [](std::uintptr_t p, int order, int hyper_complex_dim, IntPtr& out) {
          return CONEX_NewLinearMatrixInequality(PtrFromPy(p), order,
                                                 hyper_complex_dim, out.ptr());
        });
  m.def("CONEX_UpdateLinearOperator",
        [](std::uintptr_t p, int constraint, double value, int variable, int row,
           int col, int hyper_complex_dim) {
          return CONEX_UpdateLinearOperator(PtrFromPy(p), constraint, value, variable,
                                            row, col, hyper_complex_dim);
        });
  m.def("CONEX_UpdateAffineTerm",
        [](std::uintptr_t p, int constraint, double value, int row, int col,
           int hyper_complex_dim) {
          return CONEX_UpdateAffineTerm(PtrFromPy(p), constraint, value, row, col,
                                        hyper_complex_dim);
        });
  m.def("CONEX_NewLorentzConeConstraint",
        [](std::uintptr_t p, int order, IntPtr& out) {
          return CONEX_NewLorentzConeConstraint(PtrFromPy(p), order, out.ptr());
        });
  m.def("CONEX_NewLinearInequality",
        [](std::uintptr_t p, int rows, IntPtr& out) {
          return CONEX_NewLinearInequality(PtrFromPy(p), rows, out.ptr());
        });
  m.def("CONEX_NewQuadraticCost",
        [](std::uintptr_t p, IntPtr& out) {
          return CONEX_NewQuadraticCost(PtrFromPy(p), out.ptr());
        });
  m.def("CONEX_UpdateQuadraticCostMatrix",
        [](std::uintptr_t p, int id, double value, int row, int col) {
          return CONEX_UpdateQuadraticCostMatrix(PtrFromPy(p), id, value, row, col);
        });

  m.def("CONEX_QP_Solver",
        [](const py::array& Q, const py::array& cost, const py::array& ineq,
           const py::array& ub, const py::array& lb,
           const CONEX_SolverConfiguration& cfg, py::array solution,
           CONEX_SolutionStats& stats) {
          auto q = AsArray<2>(Q);
          auto qf = AsFArray2(Q);
          auto c = AsArray<1>(cost);
          auto a = AsFArray2(ineq);
          auto up = AsArray<1>(ub);
          auto lo = AsArray<1>(lb);
          auto y = AsMutable1D(solution);
          auto qi = qf.request();
          auto ci = c.request();
          auto ai = a.request();
          auto ui = up.request();
          auto li = lo.request();
          auto yi = y.request();
          py::gil_scoped_release release;
          return CONEX_QP_Solver(
              static_cast<double*>(qi.ptr), static_cast<int>(qi.shape[0]),
              static_cast<int>(qi.shape[1]), static_cast<double*>(ci.ptr),
              static_cast<int>(ci.shape[0]), static_cast<double*>(ai.ptr),
              static_cast<int>(ai.shape[0]), static_cast<int>(ai.shape[1]),
              static_cast<double*>(ui.ptr), static_cast<int>(ui.shape[0]),
              static_cast<double*>(li.ptr), static_cast<int>(li.shape[0]), &cfg,
              static_cast<double*>(yi.ptr), static_cast<int>(yi.shape[0]), &stats);
        });

  m.def("CONEX_QP_GetCanonicalProblemData",
        [](const py::array& Q, const py::array& cost, const py::array& ineq,
           const py::array& ub, const py::array& lb, IntPtr& num_ineq,
           IntPtr& num_eq, py::array matrix_A, py::array vector_b,
           py::array matrix_B, py::array vector_d) {
          auto q = AsArray<2>(Q);
          auto qf = AsFArray2(Q);
          auto c = AsArray<1>(cost);
          auto a = AsFArray2(ineq);
          auto up = AsArray<1>(ub);
          auto lo = AsArray<1>(lb);
          auto A = AsMutable2D(matrix_A);
          auto b = AsMutable1D(vector_b);
          auto B = AsMutable2D(matrix_B);
          auto d = AsMutable1D(vector_d);
          auto qi = qf.request();
          auto ci = c.request();
          auto ai = a.request();
          auto ui = up.request();
          auto li = lo.request();
          auto Ai = A.request();
          auto bi = b.request();
          auto Bi = B.request();
          auto di = d.request();
          return CONEX_QP_GetCanonicalProblemData(
              static_cast<double*>(qi.ptr), static_cast<int>(qi.shape[0]),
              static_cast<int>(qi.shape[1]), static_cast<double*>(ci.ptr),
              static_cast<int>(ci.shape[0]), static_cast<double*>(ai.ptr),
              static_cast<int>(ai.shape[0]), static_cast<int>(ai.shape[1]),
              static_cast<double*>(ui.ptr), static_cast<int>(ui.shape[0]),
              static_cast<double*>(li.ptr), static_cast<int>(li.shape[0]),
              num_ineq.ptr(), num_eq.ptr(), static_cast<double*>(Ai.ptr),
              static_cast<int>(Ai.shape[0]), static_cast<int>(Ai.shape[1]),
              static_cast<double*>(bi.ptr), static_cast<int>(bi.shape[0]),
              static_cast<double*>(Bi.ptr), static_cast<int>(Bi.shape[0]),
              static_cast<int>(Bi.shape[1]), static_cast<double*>(di.ptr),
              static_cast<int>(di.shape[0]));
        });
}
