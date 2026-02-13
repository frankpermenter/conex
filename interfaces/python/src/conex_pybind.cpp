#include <algorithm>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <vector>

#include <Eigen/Sparse>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "../../conex.h"
#include "../../../conex/cone_program.h"

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
py::array_t<double, py::array::c_style | py::array::forcecast> AsArray(
    const py::array& a) {
  auto out =
      py::array_t<double, py::array::c_style | py::array::forcecast>(a);
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

Eigen::VectorXd ToEigenVector(const py::array& a) {
  auto vec = AsArray<1>(a);
  auto vi = vec.request();
  const auto* vptr = static_cast<const double*>(vi.ptr);
  const ssize_t stride = vi.strides[0] / static_cast<ssize_t>(sizeof(double));
  Eigen::VectorXd out(static_cast<int>(vi.shape[0]));
  for (int i = 0; i < out.rows(); ++i) {
    out(i) = vptr[i * stride];
  }
  return out;
}

py::array_t<double> ToPyArray(const Eigen::VectorXd& v) {
  py::array_t<double> out({v.rows()}, {static_cast<ssize_t>(sizeof(double))});
  auto oi = out.request();
  auto* optr = static_cast<double*>(oi.ptr);
  for (int i = 0; i < v.rows(); ++i) {
    optr[i] = v(i);
  }
  return out;
}

using RowSparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;

std::vector<int> RowSupport(const RowSparseMatrix& A, int row) {
  std::vector<int> support;
  for (RowSparseMatrix::InnerIterator it(A, row); it; ++it) {
    support.push_back(it.col());
  }
  std::sort(support.begin(), support.end());
  support.erase(std::unique(support.begin(), support.end()), support.end());
  return support;
}

RowSparseMatrix BuildSparseFromCSR(const py::array& indptr, const py::array& indices,
                                   const py::array& data, int m, int n) {
  auto indptr_arr =
      py::array_t<long long, py::array::c_style | py::array::forcecast>(indptr);
  auto indices_arr =
      py::array_t<long long, py::array::c_style | py::array::forcecast>(indices);
  auto data_arr =
      py::array_t<double, py::array::c_style | py::array::forcecast>(data);
  if (indptr_arr.ndim() != 1 || indices_arr.ndim() != 1 || data_arr.ndim() != 1) {
    throw std::runtime_error("indptr, indices, data must be 1D arrays.");
  }
  if (indptr_arr.shape(0) != m + 1) {
    throw std::runtime_error("indptr length must equal m + 1.");
  }
  if (indices_arr.shape(0) != data_arr.shape(0)) {
    throw std::runtime_error("indices and data lengths must match.");
  }
  auto indptr_u = indptr_arr.unchecked<1>();
  auto indices_u = indices_arr.unchecked<1>();
  auto data_u = data_arr.unchecked<1>();
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(data_arr.shape(0)));
  for (int row = 0; row < m; ++row) {
    const long long start = indptr_u(row);
    const long long end = indptr_u(row + 1);
    for (long long k = start; k < end; ++k) {
      const int col = static_cast<int>(indices_u(k));
      if (col < 0 || col >= n) {
        throw std::runtime_error("indices out of bounds for shape.");
      }
      triplets.emplace_back(row, col, data_u(k));
    }
  }
  RowSparseMatrix A(m, n);
  A.setFromTriplets(triplets.begin(), triplets.end());
  A.makeCompressed();
  return A;
}

RowSparseMatrix BuildSparseFromCOO(const py::array& rows, const py::array& cols,
                                   const py::array& data, int m, int n) {
  auto rows_arr =
      py::array_t<long long, py::array::c_style | py::array::forcecast>(rows);
  auto cols_arr =
      py::array_t<long long, py::array::c_style | py::array::forcecast>(cols);
  auto data_arr =
      py::array_t<double, py::array::c_style | py::array::forcecast>(data);
  if (rows_arr.ndim() != 1 || cols_arr.ndim() != 1 || data_arr.ndim() != 1) {
    throw std::runtime_error("rows, cols, data must be 1D arrays.");
  }
  if (rows_arr.shape(0) != cols_arr.shape(0) || rows_arr.shape(0) != data_arr.shape(0)) {
    throw std::runtime_error("rows/cols/data must have same length.");
  }
  auto rows_u = rows_arr.unchecked<1>();
  auto cols_u = cols_arr.unchecked<1>();
  auto data_u = data_arr.unchecked<1>();
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(data_arr.shape(0)));
  for (ssize_t k = 0; k < data_arr.shape(0); ++k) {
    const int r = static_cast<int>(rows_u(k));
    const int c = static_cast<int>(cols_u(k));
    if (r < 0 || r >= m || c < 0 || c >= n) {
      throw std::runtime_error("COO row/col out of bounds.");
    }
    triplets.emplace_back(r, c, data_u(k));
  }
  RowSparseMatrix A(m, n);
  A.setFromTriplets(triplets.begin(), triplets.end());
  A.makeCompressed();
  return A;
}

RowSparseMatrix BuildSparseFromDense(const py::array& A_dense) {
  auto A = py::array_t<double, py::array::c_style | py::array::forcecast>(A_dense);
  if (A.ndim() != 2) {
    throw std::runtime_error("A must be 2D.");
  }
  auto ai = A.request();
  const int m = static_cast<int>(ai.shape[0]);
  const int n = static_cast<int>(ai.shape[1]);
  const auto* ptr = static_cast<double*>(ai.ptr);
  const ssize_t rs = ai.strides[0] / static_cast<ssize_t>(sizeof(double));
  const ssize_t cs = ai.strides[1] / static_cast<ssize_t>(sizeof(double));
  std::vector<Eigen::Triplet<double>> triplets;
  for (int r = 0; r < m; ++r) {
    for (int c = 0; c < n; ++c) {
      double v = ptr[r * rs + c * cs];
      if (v != 0.0) {
        triplets.emplace_back(r, c, v);
      }
    }
  }
  RowSparseMatrix S(m, n);
  S.setFromTriplets(triplets.begin(), triplets.end());
  S.makeCompressed();
  return S;
}

Eigen::VectorXd SparseLeastSquaresViaTree(const RowSparseMatrix& A,
                                          const Eigen::VectorXd& b,
                                          int num_threads) {
  if (A.rows() != b.rows()) {
    throw std::runtime_error("A and b dimension mismatch.");
  }
  std::vector<std::vector<int>> cliques;
  std::vector<std::vector<int>> rows_per_clique;
  std::map<std::vector<int>, std::vector<int>> grouped_rows;
  for (int row = 0; row < A.rows(); ++row) {
    auto support = RowSupport(A, row);
    if (!support.empty()) {
      grouped_rows[support].push_back(row);
    }
  }
  for (const auto& entry : grouped_rows) {
    cliques.push_back(entry.first);
    rows_per_clique.push_back(entry.second);
  }
  // Ensure every variable appears in at least one singleton clique so post-order
  // labeling always defines a full permutation over variables.
  for (int col = 0; col < A.cols(); ++col) {
    cliques.push_back({col});
    rows_per_clique.push_back({});
  }
  if (cliques.empty()) {
    throw std::runtime_error("No cliques constructed from A.");
  }

  conex::Program prog(A.cols());
  for (size_t k = 0; k < cliques.size(); ++k) {
    const auto& vars = cliques.at(k);
    const auto& rows = rows_per_clique.at(k);
    Eigen::MatrixXd block(vars.size(), vars.size());
    block.setZero();
    std::vector<int> column_to_local(A.cols(), -1);
    for (size_t i = 0; i < vars.size(); ++i) {
      column_to_local.at(vars.at(i)) = static_cast<int>(i);
    }
    for (int row : rows) {
      std::vector<std::pair<int, double>> local_entries;
      for (RowSparseMatrix::InnerIterator it(A, row); it; ++it) {
        int local = column_to_local.at(it.col());
        if (local >= 0) {
          local_entries.emplace_back(local, it.value());
        }
      }
      for (size_t i = 0; i < local_entries.size(); ++i) {
        for (size_t j = 0; j <= i; ++j) {
          int r = local_entries.at(i).first;
          int c = local_entries.at(j).first;
          double v = local_entries.at(i).second * local_entries.at(j).second;
          block(r, c) += v;
          if (r != c) {
            block(c, r) += v;
          }
        }
      }
    }
    if (vars.empty()) {
      continue;
    }
    // Objective is 0.5*x'Qx + c'x. For ||Ax-b||^2, use Q = 2*A'A.
    prog.AddQuadraticCost(2.0 * block, vars);
  }

  Eigen::VectorXd rhs = A.transpose() * b;
  prog.AddLinearCost((-2.0 * rhs).eval());

  conex::SolverConfiguration config;
  config.kkt_solver = conex::CONEX_KKT_SOLVER_TREE;
  config.num_threads = num_threads;
  config.enable_line_search = 1;
  config.enable_rescaling = 0;
  config.verbose = 0;
  Eigen::VectorXd x(A.cols());
  if (!Solve(prog, config, x.data())) {
    throw std::runtime_error("Conex tree least-squares solve failed.");
  }
  return x;
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
      .def_readwrite("num_threads", &CONEX_SolverConfiguration::num_threads)
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

  m.def(
      "sparse_ls_csr",
      [](const py::array& indptr, const py::array& indices, const py::array& data,
         int m_rows, int n_cols, const py::array& b, int num_threads) {
        Eigen::VectorXd b_eig = ToEigenVector(b);
        if (b_eig.rows() != m_rows) {
          throw std::runtime_error("b length must equal number of rows in A.");
        }
        RowSparseMatrix A =
            BuildSparseFromCSR(indptr, indices, data, m_rows, n_cols);
        Eigen::VectorXd x;
        {
          py::gil_scoped_release release;
          x = SparseLeastSquaresViaTree(A, b_eig, num_threads);
        }
        return ToPyArray(x);
      },
      py::arg("indptr"), py::arg("indices"), py::arg("data"), py::arg("m_rows"),
      py::arg("n_cols"), py::arg("b"), py::arg("num_threads") = 1);

  m.def(
      "sparse_ls_coo",
      [](const py::array& rows, const py::array& cols, const py::array& data,
         int m_rows, int n_cols, const py::array& b, int num_threads) {
        Eigen::VectorXd b_eig = ToEigenVector(b);
        if (b_eig.rows() != m_rows) {
          throw std::runtime_error("b length must equal number of rows in A.");
        }
        RowSparseMatrix A = BuildSparseFromCOO(rows, cols, data, m_rows, n_cols);
        Eigen::VectorXd x;
        {
          py::gil_scoped_release release;
          x = SparseLeastSquaresViaTree(A, b_eig, num_threads);
        }
        return ToPyArray(x);
      },
      py::arg("rows"), py::arg("cols"), py::arg("data"), py::arg("m_rows"),
      py::arg("n_cols"), py::arg("b"), py::arg("num_threads") = 1);

  m.def(
      "sparse_ls_dense",
      [](const py::array& A_dense, const py::array& b, int num_threads) {
        RowSparseMatrix A = BuildSparseFromDense(A_dense);
        Eigen::VectorXd b_eig = ToEigenVector(b);
        if (b_eig.rows() != A.rows()) {
          throw std::runtime_error("b length must equal number of rows in A.");
        }
        Eigen::VectorXd x;
        {
          py::gil_scoped_release release;
          x = SparseLeastSquaresViaTree(A, b_eig, num_threads);
        }
        return ToPyArray(x);
      },
      py::arg("A_dense"), py::arg("b"), py::arg("num_threads") = 1);

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
